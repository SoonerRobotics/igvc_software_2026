# EStop Relay PCB

This is the EStop Relay from 2024. It uses a Raspberry Pi Pico 2 (RP2350) board.

Required Libraries
- RadioHead [Arduino Library Download](https://github.com/adafruit/RadioHead/archive/master.zip)
  - IMPORTANT! You must change the following line in the RadioHead library due to a naming mismatch with `TIMER1_IRQ_1`
  
    Replace
    ```C++
    #elif defined(ARDUINO_ARCH_RP2040)
     // Raspi Pico
     #define RH_ASK_PICO_ALARM_IRQ TIMER_IRQ_1
     #define RH_ASK_PICO_ALARM_NUM 1
    #endif
    ```
    With
    ```C++
    #elif defined(ARDUINO_ARCH_RP2040)
     // Raspi Pico
     #define RH_ASK_PICO_ALARM_IRQ TIMER1_IRQ_1
     #define RH_ASK_PICO_ALARM_NUM 1
    #endif
    ```
- ACAN2515 by Pierre Molinaro