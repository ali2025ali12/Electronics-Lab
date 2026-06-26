/*
  Transmitter_8ch_16x2_I2C_with_RTH_Menu.ino
  - 8-channel transmitter for RF24 receiver
  - Mode 2 mapping (throttle = left vertical)
  - ARM implemented as debounced toggle (latched)
  - 16x2 I2C LCD (LiquidCrystal_I2C) shows:
      - TX battery and RX battery (one line)
      - ARM and RTH UP/DOWN state (one line)
      - Page 2 shows relative altitude and RX altitude + other info
  - Menu button toggles between two LCD pages
  - RTH Up/Down buttons mapped to AUX3 (ch6) and AUX4 (ch7) (hold)
  - Sends 8 x uint16_t channels (1000..2000) as payload (16 bytes)
  - Receives ACK telemetry from receiver: uint16_t batt_mV, int16_t altitude_cm
  - Optional: if you attach a BMP280 to transmitter and define TX_BARO_ENABLED,
    transmitter will display local altitude and compute relative altitude.
  - Libraries required:
      - RF24
      - LiquidCrystal_I2C
      - (optional) Adafruit_BMP280
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Optional: enable if you have BMP280 on transmitter to measure TX altitude
// #define TX_BARO_ENABLED

#ifdef TX_BARO_ENABLED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 txBmp;
  bool txBaroPresent = false;
#endif

// ---- radio ----
RF24 radio(8, 7); // CE, CSN
const byte txAddress[6] = "00001";

// ---- LCD (I2C 16x2) ----
LiquidCrystal_I2C lcd(0x27, 16, 2); // common address 0x27; change if your module is 0x3F

// ---- Controls ----
// Joysticks (Mode2)
const int joyLeftX  = A0; // yaw
const int joyLeftY  = A1; // throttle (invert)
const int joyRightX = A2; // roll
const int joyRightY = A3; // pitch

// Buttons (use INPUT_PULLUP, connect other side of button to GND)
const int btnArmPin     = 2;  // toggle arm (latched)
const int btnAuxPin     = 3;  // AUX2 momentary
const int btnRTHUpPin   = 4;  // RTH up (hold)
const int btnRTHDownPin = 5;  // RTH down (hold)
const int btnMenuPin    = 12; // menu / page toggle (momentary)

// Battery sense (transmitter)
const int batSensePin = A4; // analog pin for battery divider
const float VREF = 5.0;     // ADC ref (UNO/Pro Mini 5V)
const float R1 = 47000.0;   // top resistor
const float R2 = 10000.0;   // bottom resistor

// Channels array
uint16_t channels[8] = {1500,1500,1500,1000,1000,1000,1000,1000};

// telemetry from receiver
uint16_t rxBattery_mV = 0;
int16_t rxAltitude_cm = 0;

// TX baro altitude (optional)
float txAltitude_m = 0.0f;

// Timing
const unsigned long SEND_INTERVAL_MS = 20; // 50Hz
unsigned long lastSendMillis = 0;

// Debounce/menu/arm
unsigned long debounceMillisArm = 0;
bool lastArmState = HIGH;
bool armLatched = false;
bool armProcessed = false;

unsigned long debounceMillisMenu = 0;
bool lastMenuState = HIGH;
bool menuProcessed = false;
uint8_t lcdPage = 0; // 0 or 1

// helper to map analog to 1000..2000
int readAnalogMapped(int pin, bool invert=false) {
  int v = analogRead(pin); // 0..1023
  if (invert) return map(v, 0, 1023, 2000, 1000);
  return map(v, 0, 1023, 1000, 2000);
}

uint16_t readTxBattery_mV() {
  uint16_t a = analogRead(batSensePin);
  float vin = ((float)a / 1023.0f) * VREF;
  float vbatt = vin * ((R1 + R2) / R2);
  return (uint16_t)(vbatt * 1000.0f + 0.5f);
}

// radio send / read ack (telemetry)
void radioInit() {
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5,15);
  radio.openWritingPipe(txAddress);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening();
}

void sendPacketAndReceiveAck() {
  bool ok = radio.write(&channels, sizeof(channels));
  if (!ok) {
    // TX failed; keep previous telemetry
  } else {
    if (radio.isAckPayloadAvailable()) {
      uint8_t len = radio.getDynamicPayloadSize();
      if (len >= 4) {
        uint8_t buf[32];
        radio.read(&buf, len);
        rxBattery_mV = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        rxAltitude_cm = (int16_t)buf[2] | ((int16_t)buf[3] << 8);
      }
    }
  }
}

// display helpers
void drawPage0(uint16_t txBatt_mV, uint16_t rxBatt_mV, bool arm, bool rthUp, bool rthDown) {
  // Line1: "TX:4.11V RX:3.70V" (two columns)
  char line1[17];
  float txV = txBatt_mV / 1000.0f;
  float rxV = rxBatt_mV / 1000.0f;
  snprintf(line1, sizeof(line1), "TX:%4.2fV RX:%4.2fV", txV, rxV);
  // Line2: "ARM:ON RTH:UP" or "RTH:OFF"
  char rthState[8];
  if (rthDown) strcpy(rthState, "DN");
  else if (rthUp) strcpy(rthState, "UP");
  else strcpy(rthState, "OFF");
  char line2[17];
  snprintf(line2, sizeof(line2), "ARM:%3s  RTH:%3s", arm ? "ON" : "OFF", rthState);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
}

void drawPage1(int16_t rxAlt_cm, float txAlt_m, uint16_t thr) {
  // compute relative altitude = rxAlt_m - txAlt_m
  float rxAlt_m = rxAlt_cm / 100.0f;
  float rel = rxAlt_m - txAlt_m;
  // Left: "REL:+1.20m" (8 chars), Right: "RX:2.30m" (8 chars)
  char left[9], right[9], line1[17], line2[17];
  snprintf(left, sizeof(left), "REL:%+4.2fm", rel);
  snprintf(right, sizeof(right), "RX:%4.2fm", rxAlt_m);
  snprintf(line1, sizeof(line1), "%-8s%8s", left, right);

  // line2: show throttle and page info
  snprintf(line2, sizeof(line2), "THR:%4d PAGE:2/2", thr);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
}

// button handlers
void handleArmButton() {
  bool cur = digitalRead(btnArmPin);
  if (cur != lastArmState) {
    debounceMillisArm = millis();
    lastArmState = cur;
  }
  if (millis() - debounceMillisArm > 50) {
    if (cur == LOW && !armProcessed) {
      armLatched = !armLatched;
      armProcessed = true;
    }
    if (cur == HIGH) armProcessed = false;
  }
}

void handleMenuButton() {
  bool cur = digitalRead(btnMenuPin);
  if (cur != lastMenuState) {
    debounceMillisMenu = millis();
    lastMenuState = cur;
  }
  if (millis() - debounceMillisMenu > 50) {
    if (cur == LOW && !menuProcessed) {
      lcdPage = (lcdPage == 0) ? 1 : 0;
      menuProcessed = true;
    }
    if (cur == HIGH) menuProcessed = false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(btnArmPin, INPUT_PULLUP);
  pinMode(btnAuxPin, INPUT_PULLUP);
  pinMode(btnRTHUpPin, INPUT_PULLUP);
  pinMode(btnRTHDownPin, INPUT_PULLUP);
  pinMode(btnMenuPin, INPUT_PULLUP);
  pinMode(batSensePin, INPUT);

  // LCD init
  lcd.init();
  lcd.backlight();

  // radio init
  radioInit();

  // optional tx barometer
  #ifdef TX_BARO_ENABLED
    if (txBmp.begin()) {
      txBaroPresent = true;
      delay(20);
    } else {
      txBaroPresent = false;
    }
  #endif

  // initial screen
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("TX 8ch I2C LCD");
  lcd.setCursor(0,1);
  lcd.print("Init...");
  delay(600);
  lcd.clear();
}

void loop() {
  // read and map controls
  handleArmButton();
  handleMenuButton();

  channels[0] = readAnalogMapped(joyRightX); // roll
  channels[1] = readAnalogMapped(joyRightY); // pitch
  channels[2] = readAnalogMapped(joyLeftX);  // yaw
  channels[3] = readAnalogMapped(joyLeftY, true); // throttle (invert)

  channels[4] = armLatched ? 2000 : 1000;                      // AUX1 ARM
  channels[5] = (digitalRead(btnAuxPin) == LOW) ? 2000 : 1000; // AUX2
  channels[6] = (digitalRead(btnRTHUpPin) == LOW) ? 2000 : 1000;   // AUX3 = RTH UP
  channels[7] = (digitalRead(btnRTHDownPin) == LOW) ? 2000 : 1000; // AUX4 = RTH DOWN

  // send at interval
  if (millis() - lastSendMillis >= SEND_INTERVAL_MS) {
    lastSendMillis = millis();
    // optional update tx altitude
    #ifdef TX_BARO_ENABLED
      if (txBaroPresent) {
        txAltitude_m = txBmp.readAltitude(1013.25);
      } else txAltitude_m = 0.0f;
    #else
      txAltitude_m = 0.0f; // assume transmitter at 0m if no baro
    #endif

    // send packet & read ack telemetry
    sendPacketAndReceiveAck();

    // update LCD depending on page
    uint16_t txBatt = readTxBattery_mV();
    bool rthUp = (channels[6] > 1600);
    bool rthDown = (channels[7] > 1600);

    if (lcdPage == 0) {
      drawPage0(txBatt, rxBattery_mV, armLatched, rthUp, rthDown);
    } else {
      drawPage1(rxAltitude_cm, txAltitude_m, channels[3]);
    }
  }
}
