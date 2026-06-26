/*
  Receiver_WithMPU_PID.ino
  - Single-file receiver + MPU6050 + simple PID rate controller (inner loop)
  - Board: Arduino Uno / Pro Mini (ATmega328P)
  - Radio: nRF24L01 using RF24 library (https://github.com/nRF24/RF24)
  - Motors: 4 brushed motors via MOSFET low-side: pins 5,6,9,10
  - RC channels: 6 (Roll, Pitch, Yaw, Throttle, AUX1, AUX2)
  - AUTO_THROTTLE enabled (simple smoothing)
  - BARO/GPS/RTH present as commented #defines (disabled)
  - Safety: remove props while testing. Use limited power supply.
*/

/* ========================= CONFIG ========================= */

// Feature toggles (disabled by default as requested)
//#define BARO_ENABLED
//#define GPS_ENABLED
//#define RTH_ENABLED

// Auto throttle
#define AUTO_THROTTLE

// Pins for nRF24L01
#define RF_CE_PIN  8
#define RF_CSN_PIN 7

// nRF pipe address
const byte address[6] = "00001";

// Payload / RC channels
#define RC_CHANNELS 6
#define PAYLOAD_BYTES (RC_CHANNELS * 2) // 12 bytes

// Motor & throttle limits
#define MINCOMMAND   1000
#define MINTHROTTLE  1000
#define MAXTHROTTLE  2000

// Motor pins (mapping)
const uint8_t MOTOR_PIN_FL = 5;   // motorOut[0]
const uint8_t MOTOR_PIN_FR = 10;  // motorOut[1]
const uint8_t MOTOR_PIN_RL = 6;   // motorOut[2]
const uint8_t MOTOR_PIN_RR = 9;   // motorOut[3]

// Serial / status
#define SERIAL_BAUD 115200
#define STATUS_INTERVAL_MS 500UL
#define FAILSAFE_MS 1000UL

/* ========================= LIBRARIES ========================= */
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

/* ========================= GLOBALS ========================= */

// Radio
RF24 radio(RF_CE_PIN, RF_CSN_PIN);

// RC channels (raw, 1000..2000)
uint16_t rcChannels[RC_CHANNELS] = {1500, 1500, 1500, 1000, 1000, 1000};

// motor outputs (1000..2000)
uint16_t motorOut[4] = {MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND};

// state
unsigned long lastPacketMillis = 0;
bool armed = false;

// status timing
unsigned long lastStatusMillis = 0;

/* ========================= MPU6050 (minimal) ========================= */


// MPU6050 I2C address
#define MPU_ADDR 0x68
// Registers
#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H  0x43
#define MPU_WHO_AM_I     0x75

// raw readings
int16_t accRaw[3], gyroRaw[3];
// calibrated gyro offsets (raw units)
float gyroOffset[3] = {0, 0, 0};
// gyro sensitivity (LSB per deg/s) for FS_SEL=3 (±2000 dps): 16.4
const float GYRO_SENS = 16.4f;
// accel sensitivity LSB/g for ±2g: 16384
const float ACCEL_SENS = 16384.0f;

// angles (degrees)
float angleRoll = 0.0f;
float anglePitch = 0.0f;

// complementary filter factor (0..1) -> acc weight
#define ACC_WEIGHT 0.02f

// timing for IMU integration
unsigned long prevIMUTime = 0;
bool imuInitialized = false;

/* ========================= PID (rate controller) ========================= */

typedef struct {
  float P;
  float I;
  float D;
  float integrator;
  float lastError;
  float integratorLimit;
} PID_t;

PID_t pidRoll = {0.035f, 0.0007f, 0.0012f, 0, 0, 300};
PID_t pidPitch = {0.035f, 0.0007f, 0.0012f, 0, 0, 300};
PID_t pidYaw = {0.03f, 0.0005f, 0.0008f, 0, 0, 300};

// compute PID (rate loop). error in deg/s, dt seconds
float pidCompute(PID_t &pid, float error, float dt) {
  // P
  float Pout = pid.P * error;
  // I
  pid.integrator += error * pid.I * dt;
  if (pid.integrator > pid.integratorLimit) pid.integrator = pid.integratorLimit;
  if (pid.integrator < -pid.integratorLimit) pid.integrator = -pid.integratorLimit;
  // D
  float derivative = (error - pid.lastError) / dt;
  float Dout = pid.D * derivative;
  pid.lastError = error;
  return Pout + pid.integrator + Dout;
}

/* ========================= HELPERS ========================= */

inline uint16_t constrainRC(int val) {
  if (val < MINCOMMAND) return MINCOMMAND;
  if (val > MAXTHROTTLE) return MAXTHROTTLE;
  return (uint16_t)val;
}
inline int16_t rcToControl(int ch) {
  return (int16_t)rcChannels[ch] - 1500; // approx -500..+500
}
void writeMotorPin(uint8_t pin, uint16_t value) {
  uint8_t pwm = (uint8_t)((value > 1000) ? ((value - 1000) >> 2) : 0);
  analogWrite(pin, pwm);
}
void setAllMotors(uint16_t cmd) {
  for (uint8_t i = 0; i < 4; i++) motorOut[i] = cmd;
  writeMotorPin(MOTOR_PIN_FL, motorOut[0]);
  writeMotorPin(MOTOR_PIN_FR, motorOut[1]);
  writeMotorPin(MOTOR_PIN_RL, motorOut[2]);
  writeMotorPin(MOTOR_PIN_RR, motorOut[3]);
}

/* ========================= RADIO ========================= */

/*
  ONLY radioInit() and radioReadIfAvailable() are changed to match the transmitter you provided.
  Transmitter's struct (you sent) layout:
    uint16_t throttle;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    bool btn1;   // 1 byte
    bool btn2;   // 1 byte
  That results in a 10-byte payload from your transmitter. This radio code detects payload size
  (dynamic payloads enabled) and maps the fields into rcChannels in the receiver's expected order:
    rcChannels: 0=Roll,1=Pitch,2=Yaw,3=Throttle,4=AUX1,5=AUX2
*/

void radioInit() {
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  radio.openReadingPipe(0, address);

  // Enable dynamic payloads so we can detect what the transmitter actually sent.
  radio.enableDynamicPayloads();
  radio.startListening();
}

void radioReadIfAvailable() {
  if (!radio.available()) return;

  uint8_t len = radio.getDynamicPayloadSize();
  if (len == 0) {
    // fallback to expected fixed size (12 bytes) if dynamic payload not used by TX
    len = PAYLOAD_BYTES;
  }

  uint8_t buf[32];
  if (len > sizeof(buf)) len = sizeof(buf);
  radio.read(buf, len);

  // Case A: 12 bytes (6 x uint16_t) — legacy/common format
  if (len == (RC_CHANNELS * 2)) {
    for (uint8_t i = 0; i < RC_CHANNELS; i++) {
      uint16_t lo = buf[i * 2];
      uint16_t hi = buf[i * 2 + 1];
      uint16_t v = (hi << 8) | lo;
      if (v < 900) v = 900;
      if (v > 2100) v = 2100;
      rcChannels[i] = v;
    }
  }
  // Case B: 10 bytes — matches your transmitter struct (throttle, pitch, roll, yaw, btn1, btn2)
  else if (len == 10) {
    uint16_t throttle = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    uint16_t pitch    = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t roll     = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    uint16_t yaw      = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    uint8_t btn1      = buf[8];
    uint8_t btn2      = buf[9];

    // Map transmitter order to rcChannels expected by the rest of the code:
    rcChannels[0] = (roll  < 900) ? 900  : (roll  > 2100) ? 2100 : roll;
    rcChannels[1] = (pitch < 900) ? 900  : (pitch > 2100) ? 2100 : pitch;
    rcChannels[2] = (yaw   < 900) ? 900  : (yaw   > 2100) ? 2100 : yaw;
    rcChannels[3] = (throttle < 900) ? 900 : (throttle > 2100) ? 2100 : throttle;

    // Convert boolean buttons into AUX channel values (1000 / 2000)
    rcChannels[4] = btn1 ? 2000 : 1000;
    rcChannels[5] = btn2 ? 2000 : 1000;
  }
  // Case C: fallback (at least 8 bytes containing 4x uint16_t: throttle,pitch,roll,yaw)
  else if (len >= 8) {
    uint16_t throttle = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    uint16_t pitch    = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t roll     = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    uint16_t yaw      = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

    rcChannels[0] = (roll  < 900) ? 900  : (roll  > 2100) ? 2100 : roll;
    rcChannels[1] = (pitch < 900) ? 900  : (pitch > 2100) ? 2100 : pitch;
    rcChannels[2] = (yaw   < 900) ? 900  : (yaw   > 2100) ? 2100 : yaw;
    rcChannels[3] = (throttle < 900) ? 900 : (throttle > 2100) ? 2100 : throttle;

    // safe defaults for AUX if not present
    rcChannels[4] = 1000;
    rcChannels[5] = 1000;
  }
  // too small or unknown format: ignore
  lastPacketMillis = millis();
}

/* ========================= MPU CODE ========================= */

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}
void mpuInit() {
  Wire.begin();
  delay(20);
  // Wake up
  mpuWrite(MPU_PWR_MGMT_1, 0x00);
  delay(100);
  // Set sample rate divider (default)
  mpuWrite(MPU_SMPLRT_DIV, 0x07);
  // DLPF config
  mpuWrite(MPU_CONFIG, 0x03); // ~44Hz
  // gyro FS = ±2000 dps
  mpuWrite(MPU_GYRO_CONFIG, 0x18);
  // accel FS = ±2g
  mpuWrite(MPU_ACCEL_CONFIG, 0x00);
  delay(50);
}

/* Read accel and gyro raw values (blocking) */
void mpuReadRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (Wire.available() >= 14) {
    accRaw[0] = (Wire.read() << 8) | Wire.read();
    accRaw[1] = (Wire.read() << 8) | Wire.read();
    accRaw[2] = (Wire.read() << 8) | Wire.read();
    uint16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
    gyroRaw[0] = (Wire.read() << 8) | Wire.read();
    gyroRaw[1] = (Wire.read() << 8) | Wire.read();
    gyroRaw[2] = (Wire.read() << 8) | Wire.read();
  }
}

/* Calibrate gyro offsets (do while board is still) */
void mpuCalibrateGyro(uint16_t samples = 200) {
  long sum[3] = {0, 0, 0};
  for (uint16_t i = 0; i < samples; i++) {
    mpuReadRaw();
    sum[0] += gyroRaw[0];
    sum[1] += gyroRaw[1];
    sum[2] += gyroRaw[2];
    delay(5);
  }
  gyroOffset[0] = (float)sum[0] / (float)samples;
  gyroOffset[1] = (float)sum[1] / (float)samples;
  gyroOffset[2] = (float)sum[2] / (float)samples;
}

/* ========================= ATTITUDE ESTIMATION (complementary) ========================= */

void imuUpdate() {
  unsigned long now = micros();
  if (!prevIMUTime) prevIMUTime = now;
  float dt = (now - prevIMUTime) * 1e-6f; // seconds
  if (dt <= 0) return;
  prevIMUTime = now;

  mpuReadRaw();

  // Convert gyro raw to deg/s (subtract offset)
  float gyroX = ((float)gyroRaw[0] - gyroOffset[0]) / GYRO_SENS;
  float gyroY = ((float)gyroRaw[1] - gyroOffset[1]) / GYRO_SENS;
  float gyroZ = ((float)gyroRaw[2] - gyroOffset[2]) / GYRO_SENS;

  // Integrate gyro to angles
  angleRoll  += gyroX * dt;
  anglePitch += gyroY * dt;

  // Compute accel angles (degrees)
  float ax = (float)accRaw[0] / ACCEL_SENS;
  float ay = (float)accRaw[1] / ACCEL_SENS;
  float az = (float)accRaw[2] / ACCEL_SENS;
  float rollAcc = atan2(ay, az) * 57.295779513f;
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.295779513f;

  // Complementary filter: correct integrated gyro with accel
  angleRoll  = angleRoll  * (1.0f - ACC_WEIGHT) + rollAcc  * ACC_WEIGHT;
  anglePitch = anglePitch * (1.0f - ACC_WEIGHT) + pitchAcc * ACC_WEIGHT;
}

/* ========================= CONTROL / MIXING ========================= */

void computeControlAndMix() {
  // update IMU
  imuUpdate();

  // read RC inputs already filled by radioReadIfAvailable()

  // failsafe logic

  if (millis() - lastPacketMillis > FAILSAFE_MS) {
    // no radio -> set throttle to MINTHROTTLE and disarm
    setAllMotors(MINCOMMAND);
    armed = false;
    return;
  }

  // arm logic: require AUX1 > 1600 and throttle low (<1100) to arm
  bool reqArm = (rcChannels[4] > 1600);
  if (reqArm && !armed && rcChannels[3] < 1100) {
    armed = true;
  } else if (!reqArm) {
    armed = false;
  }

  if (!armed) {
    setAllMotors(MINCOMMAND);
    return;
  }


  // ===== DEAD BAND برای Roll / Pitch / Yaw =====
  const int CENTER = 1498;
  const int DEADBAND = 50; // 30..60 قابل تنظیم

  for (uint8_t ch = 0; ch <= 2; ch++) { // 0=Roll,1=Pitch,2=Yaw
    int16_t delta = rcChannels[ch] - CENTER;
    if (abs(delta) < DEADBAND) {
      rcChannels[ch] = CENTER;
    }
  }






  // Map RC -> desired rate (deg/s)
  // rcToControl gives -500..+500 approx. Map to -200..200 deg/s
  float desiredRollRate  = (float)rcToControl(0) * 0.4f; // deg/s
  float desiredPitchRate = (float)rcToControl(1) * 0.4f;
  float desiredYawRate   = (float)rcToControl(2) * 0.45f;

  // Get measured rates from gyro (deg/s)
  float measuredRollRate  = ((float)gyroRaw[0] - gyroOffset[0]) / GYRO_SENS;
  float measuredPitchRate = ((float)gyroRaw[1] - gyroOffset[1]) / GYRO_SENS;
  float measuredYawRate   = ((float)gyroRaw[2] - gyroOffset[2]) / GYRO_SENS;

  // rate errors
  static unsigned long lastControlTime = 0;
  unsigned long now = micros();
  float dt = (lastControlTime == 0) ? 0.004f : (now - lastControlTime) * 1e-6f;
  if (dt <= 0) dt = 0.004f;
  lastControlTime = now;

  float errRoll  = desiredRollRate  - measuredRollRate;
  float errPitch = desiredPitchRate - measuredPitchRate;
  float errYaw   = desiredYawRate   - measuredYawRate;

  // PID outputs (corrections in same units as throttle)
  float corrRoll  = pidCompute(pidRoll,  errRoll,  dt);
  float corrPitch = pidCompute(pidPitch, errPitch, dt);
  float corrYaw   = pidCompute(pidYaw,   errYaw,   dt);

  // Throttle handling (AUTO_THROTTLE smoothing)
  int thr = rcChannels[3];
#ifdef AUTO_THROTTLE
  static float thrFiltered = MINTHROTTLE;
  float targetThr = constrain((float)thr, (float)MINTHROTTLE, (float)MAXTHROTTLE);
  thrFiltered += (targetThr - thrFiltered) * 0.08f;
  thr = (int)thrFiltered;
#endif

  // Mixing (quad X):
  // motorFL = thr + pitch + roll - yaw_corr
  // motorFR = thr + pitch - roll + yaw_corr
  // motorRL = thr - pitch + roll + yaw_corr
  // motorRR = thr - pitch - roll - yaw_corr

  int32_t mFL = (int32_t)thr + (int32_t)corrPitch + (int32_t)corrRoll - (int32_t)corrYaw;
  int32_t mFR = (int32_t)thr + (int32_t)corrPitch - (int32_t)corrRoll + (int32_t)corrYaw;
  int32_t mRL = (int32_t)thr - (int32_t)corrPitch + (int32_t)corrRoll + (int32_t)corrYaw;
  int32_t mRR = (int32_t)thr - (int32_t)corrPitch - (int32_t)corrRoll - (int32_t)corrYaw;

  motorOut[0] = constrainRC(mFL);
  motorOut[1] = constrainRC(mFR);
  motorOut[2] = constrainRC(mRL);
  motorOut[3] = constrainRC(mRR);

  // write motors
  writeMotorPin(MOTOR_PIN_FL, motorOut[0]);
  writeMotorPin(MOTOR_PIN_FR, motorOut[1]);
  writeMotorPin(MOTOR_PIN_RL, motorOut[2]);
  writeMotorPin(MOTOR_PIN_RR, motorOut[3]);
}

/* ========================= STATUS PRINTER ========================= */

void statusPrinterInit() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println(F("=== Receiver + MPU + PID started ==="));
  Serial.print(F("AUTO_THROTTLE: "));
#ifdef AUTO_THROTTLE
  Serial.println(F("ENABLED"));
#else
  Serial.println(F("DISABLED"));
#endif
  Serial.print(F("BARO: "));
#ifdef BARO_ENABLED
  Serial.println(F("ENABLED"));
#else
  Serial.println(F("DISABLED"));
#endif
  Serial.print(F("GPS: "));
#ifdef GPS_ENABLED
  Serial.println(F("ENABLED"));
#else
  Serial.println(F("DISABLED"));
#endif
  Serial.print(F("RTH: "));
#ifdef RTH_ENABLED
  Serial.println(F("ENABLED"));
#else
  Serial.println(F("DISABLED"));
#endif
}

void statusPrinterLoop(float measuredRollRate = 0, float measuredPitchRate = 0) {
  unsigned long now = millis();
  if (now - lastStatusMillis < STATUS_INTERVAL_MS) return;
  lastStatusMillis = now;

  Serial.println(F("---- STATUS ----"));
  Serial.print(F("Time(ms): ")); Serial.println(now);
  Serial.print(F("LastPkt(ms): ")); Serial.println((long)(now - lastPacketMillis));
  Serial.print(F("Armed: ")); Serial.println(armed ? F("YES") : F("NO"));

  Serial.print(F("RC: "));
  for (uint8_t i = 0; i < RC_CHANNELS; i++) {
    Serial.print(rcChannels[i]);
    if (i < RC_CHANNELS - 1) Serial.print(F(", "));
  }
  Serial.println();

  Serial.print(F("Angles deg (R,P): "));
  Serial.print(angleRoll, 2); Serial.print(F(", ")); Serial.println(anglePitch, 2);

  Serial.print(F("Gyro dps (R,P,Y): "));
  float gr = ((float)gyroRaw[0] - gyroOffset[0]) / GYRO_SENS;
  float gp = ((float)gyroRaw[1] - gyroOffset[1]) / GYRO_SENS;
  float gy = ((float)gyroRaw[2] - gyroOffset[2]) / GYRO_SENS;
  Serial.print(gr, 2); Serial.print(F(", "));
  Serial.print(gp, 2); Serial.print(F(", "));
  Serial.println(gy, 2);

  Serial.print(F("Motors: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(motorOut[i]);
    if (i < 3) Serial.print(F(", "));
  }
  Serial.println();

  Serial.println(F("----------------"));
}

/* ========================= SETUP / LOOP ========================= */

void setup() {
  // motor pins
  pinMode(MOTOR_PIN_FL, OUTPUT);
  pinMode(MOTOR_PIN_FR, OUTPUT);
  pinMode(MOTOR_PIN_RL, OUTPUT);
  pinMode(MOTOR_PIN_RR, OUTPUT);
  setAllMotors(MINCOMMAND);

  // serial
  statusPrinterInit();

  // radio
  radioInit();
  lastPacketMillis = millis();

  // IMU init & calibrate
  Wire.begin();
  mpuInit();
  delay(100);
  Serial.println(F("Calibrating gyro (stay still)..."));
  mpuCalibrateGyro(300);
  Serial.print(F("Gyro offsets: "));
  Serial.print(gyroOffset[0], 2); Serial.print(F(", "));
  Serial.print(gyroOffset[1], 2); Serial.print(F(", "));
  Serial.println(gyroOffset[2], 2);
  imuInitialized = true;
  prevIMUTime = micros();
}

void loop() {
  // radio
  radioReadIfAvailable();

  // control
  computeControlAndMix();

  // status
  statusPrinterLoop();
}
