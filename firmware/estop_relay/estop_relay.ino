#include <ACAN2515.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <RHSoftwareSPI.h>

#include "estop_common.h"

#define RADIO_RESET 7
#define RADIO_MISO 12
#define RADIO_CS 5
#define RADIO_SCK 10
#define RADIO_MOSI 11
#define RADIO_INT 9

//SPI
static int MCP2515_MISO = 16;
static int MCP2515_CS = 17;
static int MCP2515_SCK = 18;
static int MCP2515_MOSI = 19;
static int MCP2515_INT = 26;

#define PICO_OUT 13

RHSoftwareSPI spi;
RH_RF95 rf95(RADIO_CS, RADIO_INT, spi);
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

// Parameters
static const int receive_timeout_ms = 2000;


//Quartz oscillator - 8MHz
static uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;

bool ESTOP = false;
bool isConnected = false;
unsigned long lastAck = 0;
unsigned long lastHandshake = 0;

unsigned long lastReceivedMessageMillis = 0;

RadioPacket toSend;
CANMessage frame;

uint8_t robot_voltage_signal = 255; // 0.1V increments, 255 = no data received yet

void setup() {

  pinMode(RADIO_RESET, INPUT);
  
  Serial.begin(115200);

  pinMode(PICO_OUT, OUTPUT);
  digitalWrite(PICO_OUT, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(2000);

  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite(RADIO_RESET, LOW);
  delay(10);
  digitalWrite(RADIO_RESET, HIGH);
  delay(10);

  spi.setPins(RADIO_MISO, RADIO_MOSI, RADIO_SCK);

  delay(20);

  SPI.setSCK(MCP2515_SCK);
  SPI.setTX(MCP2515_MOSI);
  SPI.setRX(MCP2515_MISO);
  SPI.begin();
  
  delay(20);

  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 125UL * 1000UL);  // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve);
  if (errorCode == 0) {
    Serial.println("CAN Configured");
  }
  else{
    Serial.print("Error: ");
    Serial.println(errorCode);
  }

  if (!rf95.init()) {
    Serial.println("Radio Initialization Failure");
  }
  rf95.setFrequency(912.0);
  rf95.setTxPower(23, false);

  lastHandshake = millis();
  strncpy(toSend.password, GLOBAL_PASSWORD, sizeof(GLOBAL_PASSWORD));

}

void onCanRecieve() {
  can.isr();

  if (!can.available()) {
    return;
  }

  can.receive(frame);

  switch (frame.id) {
    case 0x3f3: { // Robot voltage in millivolts (int32)
      int32_t voltage_mv;
      memcpy(&voltage_mv, frame.data, sizeof(int32_t));
      robot_voltage_signal = (uint8_t)(-voltage_mv / 100); // convert to 0.1V increments
      break;
    }
  }
}

void loop() {

  if (ESTOP) {
    return;
  }

  if (rf95.available()) {

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {

        RadioPacket msg = *(RadioPacket*)buf;

        if (strcmp(GLOBAL_PASSWORD, msg.password) == 0) {

            lastReceivedMessageMillis = millis();

            // Signal
            if (msg.id == MSG_SIGNAL_ID) {

              if (msg.signal == ESTOP_SIGNAL) {

                digitalWrite(PICO_OUT, LOW);
                ESTOP = true;
                digitalWrite(LED_BUILTIN, HIGH);

                frame.id = 0x0;
                can.tryToSend(frame);
              }

              if (msg.signal == MOB_STOP_SIGNAL) {
                frame.id = 0x1;
                can.tryToSend(frame);
              }

              if (msg.signal == MOB_START_SIGNAL) {
                frame.id = 0x9;
                can.tryToSend(frame);
              }

              toSend.id = MSG_SIGNAL_REPLY_ID;
              toSend.signal = msg.signal;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();

            }

            // Handshake
            if (!isConnected && msg.id == MSG_INIT_HANDSHAKE_ID) {
              isConnected = true;

              toSend.id = MSG_HANDSHAKE_REPLY_ID;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();
            }

            // Heartbeat
            if (isConnected && msg.id == MSG_INIT_HEARTBEAT_ID) {
              toSend.id = MSG_HEARTBEAT_REPLY_ID;
              toSend.signal = robot_voltage_signal;
              rf95.send((uint8_t*)&toSend, sizeof(toSend));
              rf95.waitPacketSent();
            }
        }
      }
  }

  // EStop if we haven't received anything in a while
  if (isConnected && (millis() - lastReceivedMessageMillis) > receive_timeout_ms) {

    digitalWrite(PICO_OUT, LOW);
    ESTOP = true;
    digitalWrite(LED_BUILTIN, HIGH);

    frame.id = 0x0;
    can.tryToSend(frame);

    isConnected = false;
  }
}
