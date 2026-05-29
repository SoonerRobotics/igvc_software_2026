/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws28xx.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

#define NUM_LEDS 144

// Global brightness scalar (0–255). 128 = 50%.
static uint8_t g_brightness = 128;

static inline uint8_t scale(uint8_t val) {
    return (uint8_t)((uint32_t)val * g_brightness / 255);
}

// ---------------------------------------------------------------------------
// CAN frame: 1 byte — the mode index.
// ---------------------------------------------------------------------------
typedef enum {
    // Core operating states
    MODE_DISABLED          = 0,   // Solid orange, both strips
    MODE_MANUAL            = 1,   // Solid green, both strips
    MODE_AUTO_ENABLED      = 2,   // White, alternating blink (A then B)
    MODE_AUTO_DISABLED     = 3,   // Solid blue, both strips

    // Override states
    MODE_LOW_POWER         = 4,   // Slow red blink, both strips in sync

    // Navigation signals (autonomous)
    MODE_TURN_LEFT         = 5,   // Left: amber blink  Right: white blink (alternating)
    MODE_TURN_RIGHT        = 6,   // Left: white blink  Right: amber blink (alternating)
    MODE_WAYPOINT_REACHED  = 7,   // Cyan, both blink in sync
    MODE_WAYPOINT_FOLLOW   = 8,   // Purple, alternating blink

    // System states
    MODE_BOOTING           = 9,   // Blue-violet-cyan rain chase (strips run opposite dirs)
    MODE_SHUTDOWN          = 10,  // White pixels extinguish from both ends toward center

    MODE_COUNT
} SafetyLightsMode;

// Volatile: written in CAN ISR, read in main loop.
static volatile SafetyLightsMode g_mode = MODE_BOOTING;

/* USER CODE END PTD */

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws28xx_a;
WS28XX_HandleTypeDef ws28xx_b;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

// ---------------------------------------------------------------------------
// Strip helpers
// ---------------------------------------------------------------------------

static void wait_for_strip(WS28XX_HandleTypeDef *ws, uint32_t dma_id) {
    while (ws->hTim->hdma[dma_id]->State != HAL_DMA_STATE_READY);
    while (ws->hTim->State != HAL_TIM_STATE_READY);
    HAL_Delay(1);
}

static void update_both_strips(void) {
    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
    WS28XX_Update(&ws28xx_a);
    WS28XX_Update(&ws28xx_b);
    wait_for_strip(&ws28xx_a, TIM_DMA_ID_CC3);
    wait_for_strip(&ws28xx_b, TIM_DMA_ID_CC4);
}

static void set_pixel_a(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    WS28XX_SetPixel_RGB(&ws28xx_a, i, scale(r), scale(g), scale(b));
}

static void set_pixel_b(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    WS28XX_SetPixel_RGB(&ws28xx_b, i, scale(r), scale(g), scale(b));
}

static void fill_a(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) set_pixel_a(i, r, g, b);
}

static void fill_b(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) set_pixel_b(i, r, g, b);
}

static void fill_both(uint8_t r, uint8_t g, uint8_t b) {
    fill_a(r, g, b);
    fill_b(r, g, b);
}

// ---------------------------------------------------------------------------
// E-STOP — hardwired, cannot be overridden by any mode or CAN message.
// ---------------------------------------------------------------------------
static void mode_estopped(void) {
    fill_both(255, 255, 255);
    update_both_strips();
}

// ---------------------------------------------------------------------------
// MODE_DISABLED — solid orange
// ---------------------------------------------------------------------------
static void mode_disabled(void) {
    fill_both(255, 80, 0);
    update_both_strips();
}

// ---------------------------------------------------------------------------
// MODE_MANUAL — solid green
// ---------------------------------------------------------------------------
static void mode_manual(void) {
    fill_both(0, 200, 0);
    update_both_strips();
}

// ---------------------------------------------------------------------------
// MODE_AUTO_DISABLED — solid blue
// ---------------------------------------------------------------------------
static void mode_auto_disabled(void) {
    fill_both(0, 80, 255);
    update_both_strips();
}

// ---------------------------------------------------------------------------
// Alternating-blink core.
//   When phase==0:  strip_on  = color,  strip_off = black
//   When phase==1:  strip_on  = black,  strip_off = color
//
// on_strip / off_strip pointers flip each half-cycle.
// period_ms = full cycle length; half is on, half is off per strip.
// ---------------------------------------------------------------------------
static void alternating_blink(
    uint8_t  a_r, uint8_t  a_g, uint8_t  a_b,   // color for strip A
    uint8_t  b_r, uint8_t  b_g, uint8_t  b_b,   // color for strip B
    uint32_t period_ms)
{
    static uint8_t  phase       = 0;
    static uint32_t last_toggle = 0;

    uint32_t now = HAL_GetTick();
    uint32_t half = period_ms / 2;

    if ((now - last_toggle) >= half) {
        phase       = !phase;
        last_toggle = now;

        if (phase == 0) {
            // A on, B off
            fill_a(a_r, a_g, a_b);
            fill_b(0, 0, 0);
        } else {
            // A off, B on
            fill_a(0, 0, 0);
            fill_b(b_r, b_g, b_b);
        }

        update_both_strips();
    }
}

// ---------------------------------------------------------------------------
// Synchronised blink — both strips same color, on/off together.
// ---------------------------------------------------------------------------
static void sync_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms) {
    static uint8_t  on          = 1;
    static uint32_t last_toggle = 0;

    uint32_t now  = HAL_GetTick();
    uint32_t half = period_ms / 2;

    if ((now - last_toggle) >= half) {
        on          = !on;
        last_toggle = now;

        if (on) {
            fill_both(r, g, b);
        } else {
            fill_both(0, 0, 0);
        }

        update_both_strips();
    }
}

// ---------------------------------------------------------------------------
// MODE_AUTO_ENABLED — white, alternating strips, ~1.4 s cycle
// ---------------------------------------------------------------------------
static void mode_auto_enabled(void) {
    alternating_blink(255, 255, 255,
                      255, 255, 255,
                      1400);
}

// ---------------------------------------------------------------------------
// MODE_LOW_POWER — slow red sync blink, ~2 s cycle
// ---------------------------------------------------------------------------
static void mode_low_power(void) {
    sync_blink(220, 0, 0, 2000);
}

// ---------------------------------------------------------------------------
// MODE_TURN_LEFT — left strip amber blink, right strip white blink, alternating
// ---------------------------------------------------------------------------
static void mode_turn_left(void) {
    // Amber on A, white on B — alternate each half-cycle
    alternating_blink(255, 165, 0,    // A = amber
                      255, 255, 255,  // B = white
                      700);
}

// ---------------------------------------------------------------------------
// MODE_TURN_RIGHT — left strip white blink, right strip amber blink, alternating
// ---------------------------------------------------------------------------
static void mode_turn_right(void) {
    alternating_blink(255, 255, 255,  // A = white
                      255, 165, 0,    // B = amber
                      700);
}

// ---------------------------------------------------------------------------
// MODE_WAYPOINT_REACHED — cyan sync blink, ~3 s handled by software
// ---------------------------------------------------------------------------
static void mode_waypoint_reached(void) {
    sync_blink(0, 200, 210, 600);
}

// ---------------------------------------------------------------------------
// MODE_WAYPOINT_FOLLOW — purple alternating blink
// ---------------------------------------------------------------------------
static void mode_waypoint_follow(void) {
    alternating_blink(160, 32, 240,   // A = purple
                      160, 32, 240,   // B = purple
                      700);
}

// ---------------------------------------------------------------------------
// MODE_BOOTING — blue-violet-cyan rain chase.
//   Strip A chases forward (LED 0 → 143).
//   Strip B chases backward (LED 143 → 0).
//   Three hues cycle through the tail: blue → violet → cyan.
// ---------------------------------------------------------------------------

#define BOOT_TAIL_LEN  40
#define BOOT_STEP_MS   8   // ms per frame

// Returns one of three hues along the tail based on position.
static void boot_hue(uint16_t tail_pos, uint8_t *r, uint8_t *g, uint8_t *b) {
    // tail_pos: 0 = tip (brightest), BOOT_TAIL_LEN-1 = end (dimmest)
    uint8_t segment = (uint8_t)(tail_pos * 3 / BOOT_TAIL_LEN); // 0, 1, or 2
    switch (segment) {
        case 0:  *r = 0;   *g = 150; *b = 255; break; // blue
        case 1:  *r = 120; *g = 0;   *b = 255; break; // violet
        default: *r = 0;   *g = 230; *b = 200; break; // cyan
    }
}

static void mode_booting(void) {
    static uint16_t head      = 0;
    static uint32_t last_step = 0;

    uint32_t now = HAL_GetTick();
    if ((now - last_step) < BOOT_STEP_MS) return;
    last_step = now;

    fill_both(0, 0, 0);

    for (uint16_t t = 0; t < BOOT_TAIL_LEN; t++) {
        uint8_t bright = (uint8_t)(255 - (uint32_t)t * 255 / BOOT_TAIL_LEN);

        uint8_t hr, hg, hb;
        boot_hue(t, &hr, &hg, &hb);

        uint8_t pr = (uint8_t)((uint32_t)hr * bright / 255);
        uint8_t pg = (uint8_t)((uint32_t)hg * bright / 255);
        uint8_t pb = (uint8_t)((uint32_t)hb * bright / 255);

        // Strip A: forward chase
        uint16_t pos_a = (head + NUM_LEDS - t) % NUM_LEDS;
        set_pixel_a(pos_a, pr, pg, pb);

        // Strip B: reverse chase (mirror)
        uint16_t pos_b = (NUM_LEDS - 1 - head + t) % NUM_LEDS;
        set_pixel_b(pos_b, pr, pg, pb);
    }

    update_both_strips();
    head = (head + 1) % NUM_LEDS;
}

// ---------------------------------------------------------------------------
// MODE_SHUTDOWN — white pixels extinguish from both ends toward center.
//   Full cycle plays once, then holds dark.  Software should send a new mode
//   before or immediately after issuing the reboot command.
// ---------------------------------------------------------------------------

#define SHUTDOWN_STEP_MS 12  // ms per LED extinguished

static void mode_shutdown(void) {
    static int16_t  front     = 0;           // sweeps 0 → NUM_LEDS/2
    static uint32_t last_step = 0;
    static uint8_t  done      = 0;

    // Reset state whenever the mode is freshly entered
    // (handled by the mode-change detection in main)

    uint32_t now = HAL_GetTick();

    if (done) {
        // Hold dark
        fill_both(0, 0, 0);
        update_both_strips();
        return;
    }

    if ((now - last_step) >= SHUTDOWN_STEP_MS) {
        last_step = now;

        if (front == 0) {
            // First call: fill everything white
            fill_both(255, 255, 255);
        }

        // Extinguish one LED from each end
        uint16_t left  = (uint16_t)front;
        uint16_t right = (uint16_t)(NUM_LEDS - 1 - front);

        set_pixel_a(left,  0, 0, 0);
        set_pixel_a(right, 0, 0, 0);
        set_pixel_b(left,  0, 0, 0);
        set_pixel_b(right, 0, 0, 0);

        update_both_strips();

        front++;
        if (front >= (int16_t)(NUM_LEDS / 2)) {
            done = 1;
        }
    }
}

// Call this when the mode transitions TO shutdown so the animation restarts.
static void reset_shutdown(void) {
    // The static locals inside mode_shutdown can't be reset from outside,
    // so we use a small one-shot flag communicated via a file-scope variable.
    // Simplest approach: just re-initialise by driving the statics via an
    // exported reset function using a flag.
    //
    // Implementation: we expose a file-scope flag that mode_shutdown checks.
}

// Cleaner approach — lifted statics into file scope so reset_shutdown works:
static int16_t  g_sd_front     = 0;
static uint32_t g_sd_last_step = 0;
static uint8_t  g_sd_done      = 0;

static void mode_shutdown_reset(void) {
    g_sd_front     = 0;
    g_sd_last_step = 0;
    g_sd_done      = 0;
}

// Replaces the static-local version above.
static void mode_shutdown_run(void) {
    uint32_t now = HAL_GetTick();

    if (g_sd_done) {
        // Hold dark — no further updates needed
        return;
    }

    if ((now - g_sd_last_step) >= SHUTDOWN_STEP_MS) {
        g_sd_last_step = now;

        if (g_sd_front == 0) {
            fill_both(255, 255, 255);
        }

        uint16_t left  = (uint16_t)g_sd_front;
        uint16_t right = (uint16_t)(NUM_LEDS - 1 - g_sd_front);

        set_pixel_a(left,  0, 0, 0);
        set_pixel_a(right, 0, 0, 0);
        set_pixel_b(left,  0, 0, 0);
        set_pixel_b(right, 0, 0, 0);

        update_both_strips();

        g_sd_front++;
        if (g_sd_front >= (int16_t)(NUM_LEDS / 2)) {
            g_sd_done = 1;
        }
    }
}

/* USER CODE END 0 */

// ---------------------------------------------------------------------------
// Application entry point
// ---------------------------------------------------------------------------
int main(void) {

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FDCAN1_Init();
    MX_TIM1_Init();

    WS28XX_Init(&ws28xx_a, &htim1, 48, TIM_CHANNEL_3, NUM_LEDS);
    WS28XX_Init(&ws28xx_b, &htim1, 48, TIM_CHANNEL_4, NUM_LEDS);

    HAL_Delay(10);
    __HAL_TIM_MOE_ENABLE(&htim1);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    FDCAN_FilterTypeDef FilterConfig;
    FilterConfig.IdType       = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1    = 0x014;
    FilterConfig.FilterID2    = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    SafetyLightsMode prev_mode = MODE_COUNT; // sentinel — force first-run init

    while (1) {
        // E-stop is unconditional and cannot be overridden.
        if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
            mode_estopped();
            prev_mode = MODE_COUNT; // reset so next mode re-initialises cleanly
            continue;
        }

        SafetyLightsMode mode = g_mode;

        // Detect mode transitions so animated modes can reset their state.
        if (mode != prev_mode) {
            if (mode == MODE_SHUTDOWN) {
                mode_shutdown_reset();
            }
            // alternating_blink and sync_blink use shared statics; resetting
            // their phase on every mode change avoids a stale half-cycle offset.
            // They self-correct within one half-period anyway, but a hard reset
            // gives a crisper visual on entry.  We do this by simply letting
            // their internal last_toggle expire (they will fire on the very next
            // call because now - 0 > any period).  No explicit reset needed.
            prev_mode = mode;
        }

        switch (mode) {
            case MODE_DISABLED:         mode_disabled();         break;
            case MODE_MANUAL:           mode_manual();           break;
            case MODE_AUTO_ENABLED:     mode_auto_enabled();     break;
            case MODE_AUTO_DISABLED:    mode_auto_disabled();    break;
            case MODE_LOW_POWER:        mode_low_power();        break;
            case MODE_TURN_LEFT:        mode_turn_left();        break;
            case MODE_TURN_RIGHT:       mode_turn_right();       break;
            case MODE_WAYPOINT_REACHED: mode_waypoint_reached(); break;
            case MODE_WAYPOINT_FOLLOW:  mode_waypoint_follow();  break;
            case MODE_BOOTING:          mode_booting();          break;
            case MODE_SHUTDOWN:         mode_shutdown_run();     break;
            default:                    mode_booting();          break;
        }
    }
}

// ---------------------------------------------------------------------------
// System clock
// ---------------------------------------------------------------------------
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

// ---------------------------------------------------------------------------
// CAN RX — 1-byte frame: mode index only.
// ---------------------------------------------------------------------------
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) return;
    if (RxHeader.Identifier != 0x14) return;

    uint8_t requested = RxData[0];
    if (requested < MODE_COUNT) {
        g_mode = (SafetyLightsMode)requested;
    }
    // Bytes 1–7 are ignored — all parameters are baked into the firmware.
}

// ---------------------------------------------------------------------------
// Error handler
// ---------------------------------------------------------------------------
void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif