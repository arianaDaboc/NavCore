// NavCore firmware: ESP32 robot that drives from A to B and goes around obstacles.
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <Preferences.h>
#include <esp_system.h>
#include "navcore_net.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

const float GYRO_LSB = 65.5;
const float GYRO_SCALE_DEFAULT = 1.0;
float GYRO_SCALE = GYRO_SCALE_DEFAULT;

float gyroZOffset = 0;
float yaw = 0;
float headingTotal = 0;
float pathHeading = 0;

const uint32_t SENSOR_BUDGET_US = 20000;

const uint32_t I2C_CLOCK_HZ = 100000;
const uint16_t I2C_TIMEOUT_MS = 50;

#define LED_RED   15
#define LED_GREEN 12
#define XSHUT_LEFT   19
#define XSHUT_FRONT  23
#define XSHUT_RIGHT  18

#define ADDR_LEFT    0x30
#define ADDR_FRONT   0x31
#define ADDR_RIGHT   0x32

#define OBSTACLE_DISTANCE 200

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

unsigned long lastSensorPrint = 0;
uint16_t frontStopMm = 200;
uint16_t lastLeftAvgMm = 8191;
uint16_t lastRightAvgMm = 8191;
uint16_t frontLimitMm = OBSTACLE_DISTANCE;
bool invalidIsNear = true;
uint16_t watchMinMm = 8190;

#define AIN1 27
#define AIN2 14
#define PWMA 13

#define BIN1 25
#define BIN2 26
#define PWMB 4

#define ENC_LEFT 32
#define ENC_RIGHT 33

volatile unsigned long countL = 0;
volatile unsigned long countR = 0;
volatile unsigned long lastL = 0;
volatile unsigned long lastR = 0;

const unsigned long ENC_MIN_US = 7000;

void IRAM_ATTR isrR() {
  unsigned long now = micros();
  if (now - lastL >= ENC_MIN_US && digitalRead(ENC_LEFT) == HIGH) {
    countR++;
    lastL = now;
  }
}

void IRAM_ATTR isrL() {
  unsigned long now = micros();
  if (now - lastR >= ENC_MIN_US && digitalRead(ENC_RIGHT) == HIGH) {
    countL++;
    lastR = now;
  }
}

const float PULSES_PER_CM_DEFAULT = 110.0 / 50.0;
float PULSES_PER_CM = PULSES_PER_CM_DEFAULT;

static Preferences calibPrefs;

void loadCalibration() {
  calibPrefs.begin("navcore", true);
  PULSES_PER_CM = calibPrefs.getFloat("ppcm", PULSES_PER_CM_DEFAULT);
  GYRO_SCALE = calibPrefs.getFloat("gs", GYRO_SCALE_DEFAULT);
  calibPrefs.end();
  Serial.print("Calibration: pulses/cm = ");
  Serial.print(PULSES_PER_CM, 4);
  Serial.print(" | gyro scale = ");
  Serial.println(GYRO_SCALE, 4);
}

void saveCalibration(float ppcm, float gs) {
  PULSES_PER_CM = ppcm;
  GYRO_SCALE = gs;
  calibPrefs.begin("navcore", false);
  calibPrefs.putFloat("ppcm", ppcm);
  calibPrefs.putFloat("gs", gs);
  calibPrefs.end();
}

void resetCalibration() {
  saveCalibration(PULSES_PER_CM_DEFAULT, GYRO_SCALE_DEFAULT);
}

const float TURN_ANGLE = 90.0;
const float SLOWDOWN_ANGLE = 45.0;

float rightOvershootEstimate = 12.0;
float leftOvershootEstimate = 12.0;
const float OVERSHOOT_MIN = 2.0;
const float OVERSHOOT_MAX = 22.0;
const float OVERSHOOT_ALPHA = 0.40;

const float HEADING_TOLERANCE = 2.0;
const float COARSE_THRESHOLD = 25.0;
const float COARSE_MARGIN = 10.0;
const int MAX_COARSE_ATTEMPTS = 3;

const int PULSE_PWM = 150;
const float PULSE_MS_PER_DEG = 2.0;
const int PULSE_MIN_MS = 25;
const int PULSE_MAX_MS = 100;
const int PULSE_BOOST_MS = 15;
const unsigned long PULSE_SETTLE_MS = 300;
const int MAX_PULSES = 12;

const float AVOIDANCE_ANGLE = 75.0;
const float RETURN_ANGLE = 90.0;

const float SCAN_STEP_DEG = 8.0;
const int   SCAN_MAX_STEPS = 6;
const float SCAN_MISS_MARGIN_MM = 100.0;
const float SENSOR_AHEAD_CM = 13.5;
const float ROBOT_HALF_WIDTH_CM = 8.0;
const float ROBOT_LENGTH_CM = 24.0;
const float SIDE_CLEARANCE_CM = 4.0;
const float MIN_SIDE_STEP_CM = 12.0;
const float MAX_SIDE_STEP_CM = 55.0;
const float SIDE_STEP_CM = 20.0;
const float PASS_OBSTACLE_CM = 60.0;

const float SIDE_SENSOR_AHEAD_CM = 13.5;
const float AXLE_TO_REAR_CM = ROBOT_LENGTH_CM - SENSOR_AHEAD_CM;

const float RETURN_ARC_ANGLE_DEG = 60.0;
const float OUT_ARC_ANGLE_DEG = 60.0;
const float ARC_TOL_DEG = 8.0;
const float ARC_END_TOL_DEG = 4.0;
const int   ARC_MAX_CORRECTION = 50;
const float ARC_EXIT_K_CM = 20.5;
const float ARC_EXIT_K_RET_CM = 17.5;
const float ARC_START_EARLY_CM = 10.0;
const float EDGE_MISS_MARGIN_CM = 10.0;
const float EDGE_JUMP_CM = 6.0;
const float WIDE_PROBE_CM = 25.0;
const float EDGE_NEAR_MARGIN_CM = 15.0;
const uint16_t AVOID_TRIGGER_MM = 270;
const float EDGE_CONE_COMP_DEG = 6.0;
const uint16_t AVOID_SAFETY_MM = 100;
const uint16_t NEAR_INVALID_MM = 300;
const uint16_t SIDE_NEAR_MM = 300;
const float SIDE_SENSOR_OFFSET_CM = 6.0;
const float REAR_MARGIN_CM = 1.0;
const int SIDE_CLEAR_CONFIRM = 6;
const float MIN_MEASURED_DEPTH_CM = 12.0;
float OBSTACLE_MAX_DEPTH_CM = 20.0;

const uint16_t SIDE_SEE_MM = 350;
const uint16_t SIDE_CLEAR_MM = 400;
const uint16_t LATERAL_CLEAR_MARGIN_MM = 100;
const int SIDE_CONFIRM_READS = 3;
const float MAX_LATERAL_CM = 60.0;
const float MAX_OBSTACLE_CM = 90.0;
const float SEE_SEARCH_EXTRA_CM = 25.0;
const float PASS_FALLBACK_CM = 25.0;

const int SEE_DROP_MM = 25;
const int CLEAR_RISE_MM = 60;
const int BASE_CAP_MM = 1000;

const float APPROACH_MAX_CM = 100.0;
float ROUTE_LENGTH_CM = 190.0;
const float ARRIVAL_ZONE_CM = 40.0;
const int   MAX_AVOIDANCES = 10;
const int OBSTACLE_CONFIRM_READS = 3;
const uint16_t SIDE_CLEARANCE_MIN_MM = 250;
const uint16_t FORCED_SIDE_MIN_MM = 200;

bool obstacleDetected = false;
bool gNestedPending = false;
int gNestedStreak = 0;
int gLastAvoidSide = 0;
int gForceSide = 0;

float gProbeGoal[2] = {-1.0f, -1.0f};
bool gProbeAbort = false;
bool gNoProbe = false;
bool sensorReadError = false;

const int FORWARD_PWM = 130;
const int LEFT_TRIM = 7;
const int RIGHT_TRIM = -7;
const int TURN_PWM = 150;
const int SLOW_PWM = 90;

const unsigned long TURN_TIMEOUT = 8000;
const unsigned long MEASURE_AFTER_STOP = 1500;

const float KP = 2.0;
const int MAX_CORRECTION = 20;

const float LINE_LOOKAHEAD_CM = 35.0;
const float LINE_MAX_ANGLE_DEG = 20.0;

const int DRIVE_OK = 0;
const int DRIVE_LIMIT = 1;
const int DRIVE_FAIL = 2;
const int DRIVE_BLOCKED = 3;

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readGyroZ(int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) {
    return false;
  }

  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// Wake the three VL53L0X one at a time (XSHUT) and give each its own I2C address.
bool initializeDistanceSensors() {
  Serial.println();
  Serial.println("INITIALISING VL53L0X...");

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  delay(100);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  sensorLeft.setTimeout(200);
  if (!sensorLeft.init()) {
    Serial.println("ERROR: LEFT SENSOR!");
    return false;
  }
  sensorLeft.setAddress(ADDR_LEFT);
  sensorLeft.setMeasurementTimingBudget(SENSOR_BUDGET_US);
  Serial.println("LEFT: OK - 0x30");

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  sensorFront.setTimeout(200);
  if (!sensorFront.init()) {
    Serial.println("ERROR: FRONT SENSOR!");
    return false;
  }
  sensorFront.setAddress(ADDR_FRONT);
  sensorFront.setMeasurementTimingBudget(SENSOR_BUDGET_US);
  Serial.println("FRONT: OK - 0x31");

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  sensorRight.setTimeout(200);
  if (!sensorRight.init()) {
    Serial.println("ERROR: RIGHT SENSOR!");
    return false;
  }
  sensorRight.setAddress(ADDR_RIGHT);
  sensorRight.setMeasurementTimingBudget(SENSOR_BUDGET_US);
  Serial.println("RIGHT: OK - 0x32");

  Serial.println("ALL 3 SENSORS ARE WORKING!");
  return true;
}

void brakeMotors();
void setForward();

bool sensorFailed(VL53L0X &sensor, uint16_t d) {
  return sensor.timeoutOccurred() || d > 8191;
}

void beginI2C() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

void unstickI2CBus() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
  }

  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(10);
  if (digitalRead(SDA_PIN) == LOW) netEvent("I2C: SDA stuck low - check the sensor wiring");
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}

// A supply dip resets the sensors to their default address: free the bus and set them up again.
bool recoverDistanceSensors() {
  static unsigned long lastTry = 0;

  if (!gRunning && lastTry && millis() - lastTry < 1000) return false;
  lastTry = millis();

  Serial.println("SENSORS NOT RESPONDING: RE-INITIALISING...");
  netEvent("Sensors not responding: re-initialising");

  bool ok = false;
  for (int attempt = 0; attempt < 2 && !ok; attempt++) {
    if (attempt > 0) delay(100);
    Wire.end();
    unstickI2CBus();
    beginI2C();
    writeRegister(PWR_MGMT_1, 0x00);
    writeRegister(GYRO_CONFIG, 0x08);
    ok = initializeDistanceSensors();
  }
  netEvent(ok ? "Sensors re-initialised" : "ERROR: sensors still not responding - check wiring and power");
  return ok;
}

bool readAllDistances(uint16_t &leftDistance,
                      uint16_t &frontDistance,
                      uint16_t &rightDistance) {

  leftDistance = sensorLeft.readRangeSingleMillimeters();
  bool leftTimeout = sensorFailed(sensorLeft, leftDistance);

  frontDistance = sensorFront.readRangeSingleMillimeters();
  bool frontTimeout = sensorFailed(sensorFront, frontDistance);

  rightDistance = sensorRight.readRangeSingleMillimeters();
  bool rightTimeout = sensorFailed(sensorRight, rightDistance);

  if (leftTimeout || frontTimeout || rightTimeout) {
    Serial.println("ERROR: VL53L0X TIMEOUT!");
    sensorReadError = true;
    return false;
  }

  sensorReadError = false;
  gLeft = leftDistance; gFront = frontDistance; gRight = rightDistance;
  gLeftT = gFrontT = gRightT = millis();
  updateLeds();
  return true;
}

void updateLeds() {
  const char *m = gMode;
  bool avoiding = !strcmp(m, "CHOOSE") || !strcmp(m, "BACK") || !strcmp(m, "OUT") ||
                  !strcmp(m, "RET") || !strcmp(m, "FAILED");
  int f = gFront;
  bool frontBlocked = f > 0 && f < 8000 && f < frontLimitMm;
  bool red = avoiding || frontBlocked;
  digitalWrite(LED_RED, red ? HIGH : LOW);
  digitalWrite(LED_GREEN, red ? LOW : HIGH);
}

int averageDistance(VL53L0X &sensor, int samples) {
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    uint16_t d = sensor.readRangeSingleMillimeters();

    if (sensorFailed(sensor, d)) {
      return -1;
    }

    if (d > 8000) d = 8190;

    sum += d;
    delay(10);
  }

  return (int)(sum / samples);
}

int frontCounter = 0;
uint16_t lastValidFront = 8190;

void resetFrontFilter() {
  frontCounter = 0;
  lastValidFront = 8190;
}

// Obstacle = 3 front readings in a row below the limit. "No target" right after a close reading also counts.
bool frontObstacleConfirmed(uint16_t frontDistance) {

  if (frontDistance >= 8000) {
    if (invalidIsNear && lastValidFront < NEAR_INVALID_MM) {
      frontCounter++;
    }
  } else {
    lastValidFront = frontDistance;

    if (frontDistance > 0 && frontDistance < frontLimitMm) {
      frontCounter++;
    } else {
      frontCounter = 0;
    }
  }

  if (frontCounter >= OBSTACLE_CONFIRM_READS) {
    frontCounter = 0;
    return true;
  }

  return false;
}

bool checkFrontObstacle(uint16_t &leftDistance,
                        uint16_t &frontDistance,
                        uint16_t &rightDistance) {

  leftDistance = 8191;
  rightDistance = 8191;

  frontDistance = sensorFront.readRangeSingleMillimeters();

  if (sensorFailed(sensorFront, frontDistance)) {
    Serial.println("ERROR: FRONT TIMEOUT!");
    brakeMotors();
    if (recoverDistanceSensors()) {
      setForward();
      return true;
    }
    netEvent("ERROR: front sensor timeout");
    sensorReadError = true;
    return false;
  }

  sensorReadError = false;
  gFront = frontDistance;
  gFrontT = millis();
  updateLeds();

  if (millis() - lastSensorPrint >= 250) {
    lastSensorPrint = millis();

    Serial.print("FRONT: ");
    Serial.print(frontDistance);
    Serial.println(" mm");
  }

  if (frontObstacleConfirmed(frontDistance)) {
    Serial.print("[FRONT] reading=");
    Serial.print(frontDistance);
    Serial.print(" mm | limit=");
    Serial.print(frontLimitMm);
    Serial.print(" mm | invalid=near: ");
    Serial.println(invalidIsNear ? "YES" : "NO");
    Serial.println("!!! OBSTACLE CONFIRMED AHEAD !!!");
    obstacleDetected = true;
    return false;
  }

  return true;
}

// Average both side sensors and take the side with more free space.
int chooseAvoidanceSide() {
  gMode = "CHOOSE";
  uint16_t leftDistance, frontDistance, rightDistance;

  unsigned long leftSum = 0;
  unsigned long rightSum = 0;
  unsigned long frontSum = 0;

  for (int i = 0; i < 3; i++) {
    if (!readAllDistances(leftDistance, frontDistance, rightDistance)) {
      if (!recoverDistanceSensors() || !readAllDistances(leftDistance, frontDistance, rightDistance)) {
        netEvent("ERROR: sensors not responding, cannot choose a side");
        return 0;
      }
    }

    leftSum += leftDistance;
    rightSum += rightDistance;
    frontSum += frontDistance;

    delay(40);
  }

  uint16_t leftAverage = leftSum / 3;
  uint16_t rightAverage = rightSum / 3;

  lastLeftAvgMm = leftAverage;
  lastRightAvgMm = rightAverage;

  frontStopMm = frontSum / 3;
  if (frontStopMm > 700) frontStopMm = 450;

  Serial.print("Average LEFT space: ");
  Serial.print(leftAverage);
  Serial.print(" mm | average RIGHT: ");
  Serial.print(rightAverage);
  Serial.print(" mm | FRONT: ");
  Serial.print(frontStopMm);
  Serial.println(" mm");

  bool leftClear = leftAverage >= SIDE_CLEARANCE_MIN_MM;
  bool rightClear = rightAverage >= SIDE_CLEARANCE_MIN_MM;

  int forced = gForceSide;
  gForceSide = 0;
  if (forced < 0) {
    if (leftAverage >= FORCED_SIDE_MIN_MM) {
      Serial.println("AVOIDING ON THE LEFT (requested side).");
      return -1;
    }
    gProbeGoal[0] = 999.0f;
    netEvent("No room on the left (%d mm): staying on the right", (int)leftAverage);
  }
  if (forced > 0) {
    if (rightAverage >= FORCED_SIDE_MIN_MM) {
      Serial.println("AVOIDING ON THE RIGHT (requested side).");
      return 1;
    }
    gProbeGoal[1] = 999.0f;
    netEvent("No room on the right (%d mm): staying on the left", (int)rightAverage);
  }

  if (!leftClear && !rightClear) {
    Serial.println("BOTH SIDES LOOK BLOCKED. NOT AVOIDING!");
    netEvent("Both sides blocked: left %d mm, right %d mm (minimum %d)", (int)leftAverage, (int)rightAverage, (int)SIDE_CLEARANCE_MIN_MM);
    return 0;
  }

  if (leftClear && (!rightClear || leftAverage >= rightAverage)) {
    Serial.println("AVOIDING ON THE LEFT.");
    return -1;
  }

  Serial.println("AVOIDING ON THE RIGHT.");
  return 1;
}

// Gyro zero-rate offset, measured while the robot stands still.
bool calibrateGyro() {
  long sum = 0;
  int16_t gz;
  int samples = 0;

  Serial.println("CALIBRATING MPU6050...");
  Serial.println("Do not move the robot!");

  for (int i = 0; i < 1000; i++) {
    if (readGyroZ(gz)) {
      sum += gz;
      samples++;
    }

    delay(2);
  }

  if (samples == 0) {
    return false;
  }

  gyroZOffset = (sum / (float)samples) / GYRO_LSB;

  Serial.print("Offset Z: ");
  Serial.println(gyroZOffset, 4);

  return true;
}

void stopMotors() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);

  ledcWrite(PWMA, 255);
  ledcWrite(PWMB, 255);
}

void setForward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  ledcWrite(PWMA, FORWARD_PWM + RIGHT_TRIM);
  ledcWrite(PWMB, FORWARD_PWM + LEFT_TRIM);
}

void setRightTurn(int pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

void setReverse(int pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  ledcWrite(PWMA, pwm + RIGHT_TRIM);
  ledcWrite(PWMB, pwm + LEFT_TRIM);
}

void setLeftTurn(int pwm) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

void readEncoders(unsigned long &left, unsigned long &right) {
  noInterrupts();

  left = countR;
  right = countL;

  interrupts();
}

bool gTurning = false;
bool gReversing = false;
unsigned long odoLastAvg = 0;

// Dead reckoning in the route frame (x along A->B, y to the right): x += ds*cos(th), y += ds*sin(th).
void odomStep() {
  unsigned long l, r;
  readEncoders(l, r);
  unsigned long avg = (l + r) / 2;

  if (avg < odoLastAvg) {
    odoLastAvg = avg;
    return;
  }

  if (avg > odoLastAvg) {
    float ds = (avg - odoLastAvg) / PULSES_PER_CM;
    odoLastAvg = avg;

    if (!gTurning) {
      if (gReversing) ds = -ds;
      float th = (headingTotal - pathHeading) * PI / 180.0f;
      gX = gX + ds * cosf(th);
      gY = gY + ds * sinf(th);
      netPose(gX, gY);
    }
  }
}

struct TurnGuard {
  TurnGuard() { gTurning = true; }
  ~TurnGuard() { gTurning = false; }
};

// Heading = integral of the Z gyro rate. Also advances the odometry.
bool updateYaw(unsigned long &previousTime) {
  int16_t gz;

  if (Serial.available() && Serial.peek() == 'x') {
    Serial.read();
    gAbortReq = true;
  }
  if (gAbortReq) {
    return false;
  }

  if (!readGyroZ(gz)) {
    return false;
  }

  unsigned long currentTime = micros();

  float dt = (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  float gyroZ = gz / GYRO_LSB;
  float calibratedZ = gyroZ - gyroZOffset;

  float deltaAngle = calibratedZ * dt * GYRO_SCALE;

  yaw += deltaAngle;
  headingTotal += deltaAngle;

  odomStep();

  return true;
}

void waitIntegrating(unsigned long ms) {
  unsigned long previousTime = micros();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (!updateYaw(previousTime)) {
      delay(5);
      continue;
    }
    delay(2);
  }
}

const int REVERSE_PWM = 110;

bool backUpCm(float distanceCm) {
  odomStep();
  noInterrupts();
  countL = 0;
  countR = 0;
  lastL = 0;
  lastR = 0;
  interrupts();

  unsigned long targetPulses = (unsigned long)(distanceCm * PULSES_PER_CM + 0.5f);
  unsigned long previousTime = micros();
  unsigned long startTime = millis();

  Serial.print("REVERSING: ");
  Serial.print(distanceCm, 1);
  Serial.println(" cm");

  gMode = "BACK";
  gReversing = true;
  setReverse(REVERSE_PWM);

  bool ok = true;
  while (true) {
    if (!updateYaw(previousTime)) { ok = false; break; }

    unsigned long left, right;
    readEncoders(left, right);
    if ((left + right) / 2 >= targetPulses) break;

    if (millis() - startTime >= 4000) {
      Serial.println("ERROR: REVERSE TIMEOUT!");
      ok = false;
      break;
    }
    delay(5);
  }

  brakeMotors();
  waitIntegrating(250);
  stopMotors();
  odomStep();
  gReversing = false;
  return ok;
}

float lastMoveTravelledCm() {
  unsigned long left, right;
  readEncoders(left, right);
  return ((left + right) / 2.0f) / PULSES_PER_CM;
}

bool moveForwardCm(float distanceCm,
                   float targetHeading,
                   bool detectObstacle = true,
                   bool followLine = false) {

  unsigned long targetPulses =
    (unsigned long)(distanceCm * PULSES_PER_CM + 0.5f);

  odomStep();
  noInterrupts();

  countL = 0;
  countR = 0;
  lastL = 0;
  lastR = 0;

  interrupts();

  obstacleDetected = false;
  sensorReadError = false;
  resetFrontFilter();

  Serial.println();
  Serial.print("DRIVING: ");
  Serial.print(distanceCm, 1);
  Serial.print(" cm | target heading: ");
  Serial.println(targetHeading, 1);

  Serial.print("TARGET PULSES: ");
  Serial.println(targetPulses);

  unsigned long previousTime = micros();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  unsigned long moveTimeout =
    max((unsigned long)12000,
        (unsigned long)(distanceCm * 500.0f));

  lastSensorPrint = 0;

  int leftPWM = FORWARD_PWM;
  int rightPWM = FORWARD_PWM;

  setForward();

  while (true) {

    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050!");
      if (!gAbortReq) netEvent("ERROR: MPU6050 read failed");
      delay(300);
      stopMotors();
      return false;
    }

    if (detectObstacle) {
      uint16_t leftDistance, frontDistance, rightDistance;

      if (!checkFrontObstacle(leftDistance,
                              frontDistance,
                              rightDistance)) {

        brakeMotors();
        Serial.println("SAFETY STOP!");
        delay(300);
        stopMotors();
        return false;
      }
    }

    unsigned long left, right;
    readEncoders(left, right);

    float headingGoal = targetHeading;
    if (followLine) {
      float back = atan2f((float)gY, LINE_LOOKAHEAD_CM) * 180.0f / PI;
      headingGoal = targetHeading - constrain(back, -LINE_MAX_ANGLE_DEG, LINE_MAX_ANGLE_DEG);
    }

    float error = headingTotal - headingGoal;

    int correction = (int)(KP * error);

    correction = constrain(correction,
                           -MAX_CORRECTION,
                           MAX_CORRECTION);

    leftPWM = constrain(
      FORWARD_PWM + LEFT_TRIM - correction, 0, 255);

    rightPWM = constrain(
      FORWARD_PWM + RIGHT_TRIM + correction, 0, 255);

    if (left >= targetPulses) leftPWM = 0;
    if (right >= targetPulses) rightPWM = 0;

    ledcWrite(PWMB, leftPWM);
    ledcWrite(PWMA, rightPWM);

    if (millis() - lastPrint >= 300) {
      lastPrint = millis();

      Serial.print("Left: ");
      Serial.print(left);

      Serial.print(" | Right: ");
      Serial.print(right);

      Serial.print(" | Heading: ");
      Serial.print(headingTotal, 2);

      Serial.print(" | Error: ");
      Serial.print(error, 2);

      Serial.print(" | PWM L: ");
      Serial.print(leftPWM);

      Serial.print(" | PWM R: ");
      Serial.println(rightPWM);
    }

    if (left >= targetPulses && right >= targetPulses) {
      brakeMotors();
      waitIntegrating(150);
      stopMotors();

      Serial.println("DRIVE FINISHED!");

      Serial.print("LEFT pulses: ");
      Serial.println(left);

      Serial.print("RIGHT pulses: ");
      Serial.println(right);

      Serial.print("Heading final: ");
      Serial.println(headingTotal, 2);

      waitIntegrating(250);
      return true;
    }

    if (millis() - startTime >= moveTimeout) {
      brakeMotors();
      Serial.println("ERROR: DRIVE TIMEOUT!");
      netEvent("ERROR: drive timeout, encoders L=%lu R=%lu of %lu", left, right, targetPulses);
      delay(300);
      stopMotors();
      return false;
    }

    delay(5);
  }
}

int driveWatching(float targetHeading,
                  VL53L0X &watch,
                  bool waitForClear,
                  uint16_t thresholdMm,
                  float maxCm,
                  float &eventCm,
                  float &stopCm,
                  uint16_t frontSafetyMm = OBSTACLE_DISTANCE,
                  int confirmReads = SIDE_CONFIRM_READS) {

  odomStep();
  noInterrupts();

  countL = 0;
  countR = 0;
  lastL = 0;
  lastR = 0;

  interrupts();

  obstacleDetected = false;
  sensorReadError = false;
  resetFrontFilter();

  Serial.println();
  Serial.print("DRIVING WITH SIDE WATCH | ");

  Serial.print(waitForClear
                 ? "waiting for it to clear"
                 : "waiting to see the obstacle");

  Serial.print(" | limit: ");
  Serial.print(thresholdMm);

  Serial.print(" mm | max: ");
  Serial.print(maxCm, 1);

  Serial.println(" cm");

  unsigned long previousTime = micros();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  unsigned long timeoutMs =
    max((unsigned long)8000,
        (unsigned long)(maxCm * 500.0f));

  unsigned long maxPulses =
    (unsigned long)(maxCm * PULSES_PER_CM);

  int confirm = 0;
  int frontSafetyConfirm = 0;
  watchMinMm = 8190;

  unsigned long firstPulses = 0;

  eventCm = 0;
  stopCm = 0;

  lastSensorPrint = 0;

  setForward();

  while (true) {

    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050!");
      if (!gAbortReq) netEvent("ERROR: MPU6050 read failed");
      delay(300);
      stopMotors();
      return DRIVE_FAIL;
    }

    uint16_t leftDistance, frontDistance, rightDistance;

    if (!checkFrontObstacle(leftDistance,
                            frontDistance,
                            rightDistance)) {

      brakeMotors();
      Serial.println("SAFETY STOP!");
      delay(300);
      stopMotors();
      return DRIVE_FAIL;
    }

    if (frontDistance < frontSafetyMm) {
      frontSafetyConfirm++;
    } else {
      frontSafetyConfirm = 0;
    }

    if (frontSafetyConfirm >= 2) {
      brakeMotors();
      delay(200);
      stopMotors();

      Serial.print("WAY BLOCKED AHEAD: ");
      Serial.print(frontDistance);
      Serial.println(" mm. Stopping to re-evaluate the avoidance side.");

      return DRIVE_BLOCKED;
    }

    updateYaw(previousTime);

    uint16_t d = watch.readRangeSingleMillimeters();

    if (sensorFailed(watch, d)) {
      brakeMotors();
      Serial.println("ERROR: SIDE SENSOR TIMEOUT!");
      delay(300);
      stopMotors();
      return DRIVE_FAIL;
    }

    if (d > 8000) d = 8190;

    updateYaw(previousTime);

    unsigned long left, right;
    readEncoders(left, right);

    unsigned long avg = (left + right) / 2;

    bool condition = waitForClear
                       ? (d > thresholdMm)
                       : (d < thresholdMm);

    if (condition) {
      if (confirm == 0) firstPulses = avg;
      if (d < watchMinMm) watchMinMm = d;
      confirm++;
    } else {
      confirm = 0;
    }

    float error = headingTotal - targetHeading;

    int correction = (int)(KP * error);

    correction = constrain(correction,
                           -MAX_CORRECTION,
                           MAX_CORRECTION);

    int leftPWM = constrain(
      FORWARD_PWM + LEFT_TRIM - correction, 0, 255);

    int rightPWM = constrain(
      FORWARD_PWM + RIGHT_TRIM + correction, 0, 255);

    ledcWrite(PWMB, leftPWM);
    ledcWrite(PWMA, rightPWM);

    if (millis() - lastPrint >= 300) {
      lastPrint = millis();

      Serial.print("Lateral: ");
      Serial.print(d);

      Serial.print(" mm | Pulses: ");
      Serial.print(avg);

      Serial.print(" | Heading: ");
      Serial.print(headingTotal, 2);

      Serial.print(" | Error: ");
      Serial.println(error, 2);
    }

    if (confirm >= confirmReads) {
      brakeMotors();
      waitIntegrating(150);
      stopMotors();

      readEncoders(left, right);
      avg = (left + right) / 2;

      eventCm = firstPulses / PULSES_PER_CM;
      stopCm = avg / PULSES_PER_CM;

      Serial.print("EVENT AT ");
      Serial.print(eventCm, 1);

      Serial.print(" cm | STOPPED AT ");
      Serial.print(stopCm, 1);

      Serial.println(" cm");

      waitIntegrating(250);
      return DRIVE_OK;
    }

    if (avg >= maxPulses) {
      brakeMotors();
      waitIntegrating(150);
      stopMotors();

      readEncoders(left, right);
      avg = (left + right) / 2;

      eventCm = avg / PULSES_PER_CM;
      stopCm = eventCm;

      Serial.println("DISTANCE LIMIT REACHED WITHOUT AN EVENT.");

      waitIntegrating(250);
      return DRIVE_LIMIT;
    }

    if (millis() - startTime >= timeoutMs) {
      brakeMotors();
      Serial.println("ERROR: WATCHED DRIVE TIMEOUT!");
      delay(300);
      stopMotors();
      return DRIVE_FAIL;
    }

    delay(2);
  }
}

// Pivot turn that brakes early by the learned overshoot, then measures the coast to update it.
bool turnByAngle(float angleDegrees) {

  TurnGuard turnGuard;

  yaw = 0;

  unsigned long previousTime = micros();
  unsigned long startTime = millis();

  bool turnRight = angleDegrees > 0;
  float target = fabs(angleDegrees);

  float overshootEstimate = turnRight
                              ? rightOvershootEstimate
                              : leftOvershootEstimate;

  float brakeTarget = target - overshootEstimate;

  if (brakeTarget < 1.0f) brakeTarget = 1.0f;

  Serial.println();
  Serial.print("TURN ");
  Serial.print(turnRight ? "RIGHT " : "LEFT ");
  Serial.print(target, 1);
  Serial.println(" degrees");

  Serial.print("Early brake at: ");
  Serial.println(brakeTarget, 2);

  if (turnRight) setRightTurn(TURN_PWM);
  else setLeftTurn(TURN_PWM);

  while ((turnRight && yaw < brakeTarget) ||
         (!turnRight && yaw > -brakeTarget)) {

    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050 DURING TURN!");
      delay(300);
      stopMotors();
      return false;
    }

    if (fabs(yaw) >= SLOWDOWN_ANGLE) {
      ledcWrite(PWMA, SLOW_PWM);
      ledcWrite(PWMB, SLOW_PWM);
    }

    if (millis() - startTime >= TURN_TIMEOUT) {
      brakeMotors();
      Serial.println("ERROR: TURN TIMEOUT!");
      Serial.print("Angle reached: ");
      Serial.println(yaw, 2);
      delay(300);
      stopMotors();
      return false;
    }

    delay(5);
  }

  float yawAtBrake = yaw;

  brakeMotors();

  unsigned long stopTime = millis();

  while (millis() - stopTime < MEASURE_AFTER_STOP) {
    if (!updateYaw(previousTime)) {
      stopMotors();
      Serial.println("MPU6050 ERROR AFTER TURN!");
      return false;
    }

    delay(10);
  }

  stopMotors();

  float coast = fabs(yaw) - fabs(yawAtBrake);

  if (coast < 0) coast = 0;

  float learned = constrain(coast,
                            OVERSHOOT_MIN,
                            OVERSHOOT_MAX);

  if (turnRight) {
    rightOvershootEstimate =
      (1.0f - OVERSHOOT_ALPHA) * rightOvershootEstimate +
      OVERSHOOT_ALPHA * learned;
  } else {
    leftOvershootEstimate =
      (1.0f - OVERSHOOT_ALPHA) * leftOvershootEstimate +
      OVERSHOOT_ALPHA * learned;
  }

  Serial.print("Angle at brake: ");
  Serial.println(yawAtBrake, 2);

  Serial.print("FINAL ANGLE: ");
  Serial.println(yaw, 2);

  Serial.print("Coast after brake: ");
  Serial.println(coast, 2);

  Serial.print("New estimate ");
  Serial.print(turnRight ? "RIGHT: " : "LEFT: ");

  Serial.println(
    turnRight ? rightOvershootEstimate : leftOvershootEstimate,
    2
  );

  delay(300);
  return true;
}

// Short correction pulse for the last degrees of a turn.
bool pulseTurn(float errorDegrees, int boostMs) {

  TurnGuard turnGuard;

  bool turnRight = errorDegrees > 0;

  float absError = fabs(errorDegrees);

  int durationMs = constrain(
    (int)(absError * PULSE_MS_PER_DEG) + boostMs,
    PULSE_MIN_MS,
    PULSE_MAX_MS
  );

  Serial.print("[PULSE] ");
  Serial.print(turnRight ? "right " : "left ");
  Serial.print(durationMs);
  Serial.println(" ms");

  unsigned long previousTime = micros();

  if (turnRight) setRightTurn(PULSE_PWM);
  else setLeftTurn(PULSE_PWM);

  unsigned long pulseStart = millis();

  while (millis() - pulseStart < (unsigned long)durationMs) {
    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050 DURING PULSE!");
      delay(300);
      stopMotors();
      return false;
    }

    delay(2);
  }

  brakeMotors();

  unsigned long settleStart = millis();

  while (millis() - settleStart < PULSE_SETTLE_MS) {
    if (!updateYaw(previousTime)) {
      stopMotors();
      Serial.println("MPU6050 ERROR AFTER PULSE!");
      return false;
    }

    delay(5);
  }

  stopMotors();
  return true;
}

// Coarse turn for large errors, then pulses until the error is within HEADING_TOLERANCE.
bool turnToHeading(float targetHeading) {

  int coarseAttempts = 0;
  int pulses = 0;
  int boostMs = 0;

  while (true) {

    float error = targetHeading - headingTotal;

    Serial.println();

    Serial.print("[HEADING] target: ");
    Serial.print(targetHeading, 2);

    Serial.print(" | current: ");
    Serial.print(headingTotal, 2);

    Serial.print(" | error: ");
    Serial.println(error, 2);

    if (fabs(error) <= HEADING_TOLERANCE) {
      return true;
    }

    if (fabs(error) > COARSE_THRESHOLD) {

      if (coarseAttempts >= MAX_COARSE_ATTEMPTS) {
        Serial.println("[HEADING] ERROR: the large turn does not converge.");
        return false;
      }

      coarseAttempts++;

      float angle = error > 0
                      ? error - COARSE_MARGIN
                      : error + COARSE_MARGIN;

      if (!turnByAngle(angle)) return false;

      continue;
    }

    if (pulses >= MAX_PULSES) {
      Serial.print("[HEADING] WARNING: error left after ");
      Serial.print(MAX_PULSES);
      Serial.print(" pulses: ");
      Serial.println(targetHeading - headingTotal, 2);

      return true;
    }

    pulses++;

    float headingBefore = headingTotal;

    if (!pulseTurn(error, boostMs)) return false;

    float moved = fabs(headingTotal - headingBefore);

    Serial.print("[PULSE] measured movement: ");
    Serial.println(moved, 2);

    if (moved < 1.0) {
      boostMs += PULSE_BOOST_MS;
    } else {
      boostMs = 0;
    }
  }
}

void printAvoidanceHeading(const char *stage, float startHeading) {

  Serial.print("[HEADING] ");
  Serial.print(stage);

  Serial.print(" | total heading: ");
  Serial.print(headingTotal, 2);

  Serial.print(" degrees | angle to route: ");
  Serial.print(headingTotal - startHeading, 2);

  Serial.println(" degrees");
}

// Arc back to the A->B line, using the lateral offset from odometry.
bool returnToLineArc(int side, float lateralCm, float &forwardCm) {

  gMode = "RET";

  const float outHeading = pathHeading - side * RETURN_ARC_ANGLE_DEG;

  odomStep();
  noInterrupts();
  countL = 0;
  countR = 0;
  lastL = 0;
  lastR = 0;
  interrupts();

  obstacleDetected = false;
  sensorReadError = false;
  resetFrontFilter();
  lastSensorPrint = 0;

  Serial.println();
  Serial.print("RETURN ARC | lateral to recover: ");
  Serial.print(lateralCm, 1);
  Serial.print(" cm | arc target heading: ");
  Serial.println(outHeading, 1);

  unsigned long previousTime = micros();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  unsigned long prevAvg = 0;

  float lateral = 0.0f;
  float forward = 0.0f;
  float firstArcLateral = 0.0f;
  float target = outHeading;
  int phase = 0;

  const unsigned long timeoutMs = 20000;

  setForward();

  while (true) {

    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050 IN ARC!");
      delay(300);
      stopMotors();
      return false;
    }

    uint16_t leftDistance, frontDistance, rightDistance;

    if (!checkFrontObstacle(leftDistance, frontDistance, rightDistance)) {
      brakeMotors();
      Serial.println("SAFETY STOP IN ARC!");
      netEvent("Return stopped: something ahead at %d mm", (int)gFront);
      delay(300);
      stopMotors();
      return false;
    }

    unsigned long left, right;
    readEncoders(left, right);
    unsigned long avg = (left + right) / 2;

    if (avg > prevAvg) {
      float ds = (avg - prevAvg) / PULSES_PER_CM;
      float dev = fabs(headingTotal - pathHeading);
      forward += ds * cosf(dev * PI / 180.0f);
      prevAvg = avg;
    }
    lateral = lateralCm - side * gY;

    if (lateral < -10.0f) {
      brakeMotors();
      waitIntegrating(300);
      stopMotors();
      Serial.println("ERROR: RETURN IS MOVING AWAY FROM THE ROUTE!");
      netEvent("ERROR: return is moving away from the route (offset %.1f cm)", (float)gY);
      return false;
    }

    float dev = fabs(headingTotal - pathHeading);

    if (phase == 0) {
      if (dev >= RETURN_ARC_ANGLE_DEG - ARC_TOL_DEG) {
        phase = 1;
        firstArcLateral = lateral;
        Serial.print("[ARC] entry finished | lateral gained: ");
        Serial.println(firstArcLateral, 1);
      } else if (lateral >= lateralCm / 2.0f) {
        phase = 2;
        firstArcLateral = lateral;
        target = pathHeading;
        Serial.println("[ARC] half the lateral reached before the maximum angle - leaving the arc");
      }
    }

    if (phase == 1 && lateral >= lateralCm - firstArcLateral) {
      phase = 2;
      target = pathHeading;
      Serial.print("[ARC] leaving the arc | lateral: ");
      Serial.println(lateral, 1);
    }

    if (phase < 2 && lateral + ARC_EXIT_K_RET_CM * (1.0f - cosf(dev * PI / 180.0f)) >= lateralCm) {
      phase = 2;
      target = pathHeading;
      Serial.print("[ARC] leaving the arc (from the current angle) | lateral: ");
      Serial.println(lateral, 1);
    }

    if (phase < 2 && lateral >= lateralCm) {
      phase = 2;
      target = pathHeading;
      Serial.println("[ARC] lateral reached - forced exit from the arc");
    }

    if (phase == 2 && dev <= ARC_END_TOL_DEG) {
      brakeMotors();
      waitIntegrating(150);
      stopMotors();

      forwardCm = forward;
      Serial.print("RETURN FINISHED | forward travel: ");
      Serial.print(forward, 1);
      Serial.print(" cm | estimated lateral: ");
      Serial.print(lateral, 1);
      Serial.print(" cm | angle to route: ");
      Serial.print(headingTotal - pathHeading, 2);
      Serial.println(" degrees");

      waitIntegrating(250);
      return true;
    }

    float error = headingTotal - target;
    int correction = (int)(KP * error);
    correction = constrain(correction, -ARC_MAX_CORRECTION, ARC_MAX_CORRECTION);

    int leftPWM = constrain(FORWARD_PWM + LEFT_TRIM - correction, 0, 255);
    int rightPWM = constrain(FORWARD_PWM + RIGHT_TRIM + correction, 0, 255);

    ledcWrite(PWMB, leftPWM);
    ledcWrite(PWMA, rightPWM);

    if (millis() - lastPrint >= 300) {
      lastPrint = millis();
      Serial.print("[ARC] phase ");
      Serial.print(phase);
      Serial.print(" | lateral: ");
      Serial.print(lateral, 1);
      Serial.print(" cm | heading: ");
      Serial.print(headingTotal, 2);
      Serial.print(" | target: ");
      Serial.print(target, 1);
      Serial.print(" | PWM L/R: ");
      Serial.print(leftPWM);
      Serial.print("/");
      Serial.println(rightPWM);
    }

    if (millis() - startTime >= timeoutMs) {
      brakeMotors();
      Serial.println("ERROR: RETURN ARC TIMEOUT!");
      netEvent("Return stopped: timeout");
      delay(300);
      stopMotors();
      return false;
    }

    delay(5);
  }
}

// Arc away from the route, find the obstacle edge with the front sensor,
// then drive past it while the side sensor looks for its rear.
bool avoidOutArcAndPass(int side, VL53L0X &obstacleSensor, float &lateralEndCm, float &xEndCm) {

  gMode = "OUT";
  gOx = gX; gOy = gY; gSide = side;
  gEdgeY = -999; gGoalY = -999; gRearX = -999; gFaceX = -999;

  const float outHeading = pathHeading + side * OUT_ARC_ANGLE_DEG;
  const float xFace = SENSOR_AHEAD_CM + frontStopMm / 10.0f;
  gFaceX = xFace;

  odomStep();
  noInterrupts();
  countL = 0;
  countR = 0;
  lastL = 0;
  lastR = 0;
  interrupts();

  obstacleDetected = false;
  sensorReadError = false;
  resetFrontFilter();
  lastSensorPrint = 0;

  Serial.println();
  Serial.print("ARC OUT | obstacle face at X = ");
  Serial.print(xFace, 1);
  Serial.print(" cm from the axle | arc target heading: ");
  Serial.println(outHeading, 1);

  unsigned long previousTime = micros();
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  unsigned long prevAvg = 0;

  float X = 0.0f, Y = 0.0f;
  float l1 = 0.0f;
  float yAtExit = 0.0f;
  float devAtExit = 0.0f;
  float target = outHeading;
  int phase = 0;

  bool haveHit = false;
  float lastHitY = 0.0f;
  float lastHitD = 0.0f;
  float yEdgeUsed = 0.0f;
  int missCount = 0;
  bool edgeFound = false;
  float yGoal = MAX_SIDE_STEP_CM;

  bool sideSeen = false;
  int sideFarCount = 0;
  bool rearConfirmed = false;
  float boxMaxX = -1000.0f;
  float xPassEnd = 0.0f;
  const float obstacleSideStartMm = (side > 0) ? (float)lastLeftAvgMm : (float)lastRightAvgMm;

  const unsigned long timeoutMs = 30000;

  setForward();

  while (true) {

    if (!updateYaw(previousTime)) {
      brakeMotors();
      Serial.println("ERROR READING MPU6050 IN ARC OUT!");
      delay(300);
      stopMotors();
      return false;
    }

    uint16_t leftDistance, frontDistance, rightDistance;

    if (!checkFrontObstacle(leftDistance, frontDistance, rightDistance)) {
      brakeMotors();
      Serial.println("SAFETY STOP IN ARC OUT!");
      netEvent("Arc out stopped: something ahead at %d mm", (int)gFront);
      delay(300);
      stopMotors();
      return false;
    }

    unsigned long left, right;
    readEncoders(left, right);
    unsigned long avg = (left + right) / 2;

    float devSigned = side * (headingTotal - pathHeading);
    float thetaRad = devSigned * PI / 180.0f;
    float cosT = cosf(thetaRad);
    float sinT = sinf(thetaRad);

    if (avg > prevAvg) {
      float ds = (avg - prevAvg) / PULSES_PER_CM;
      X += ds * cosT;
      Y += ds * sinT;
      prevAvg = avg;
    }

    if (!edgeFound && cosT > 0.2f) {
      float dCm = frontDistance / 10.0f;
      bool valid = frontDistance < 8000;
      float sx = X + SENSOR_AHEAD_CM * cosT;
      float dExp = (xFace - sx) / cosT;

      float hitY = Y + (SENSOR_AHEAD_CM + dCm) * sinT;

      bool jump = haveHit && fabs(hitY - lastHitY) > EDGE_JUMP_CM;

      bool tooClose = dCm < dExp - EDGE_NEAR_MARGIN_CM;

      if (valid && dCm <= dExp + EDGE_MISS_MARGIN_CM && !jump && !tooClose) {
        lastHitY = hitY;
        lastHitD = dCm;
        haveHit = true;
        missCount = 0;
      } else if (haveHit) {
        missCount++;
      }

      if (missCount >= 2) {
        edgeFound = true;
        yEdgeUsed = lastHitY - lastHitD * tanf(EDGE_CONE_COMP_DEG * PI / 180.0f);
        yGoal = yEdgeUsed + ROBOT_HALF_WIDTH_CM + SIDE_CLEARANCE_CM;
        yGoal = constrain(yGoal, MIN_SIDE_STEP_CM, MAX_SIDE_STEP_CM);
        gEdgeY = yEdgeUsed; gGoalY = yGoal;
        netEvent("Edge found at Y=%.1f cm, lateral target %.1f cm", yEdgeUsed, yGoal);

        int si = side > 0 ? 1 : 0;
        float freeSideMm = (side > 0) ? (float)lastRightAvgMm : (float)lastLeftAvgMm;
        bool fits = (yEdgeUsed + ROBOT_HALF_WIDTH_CM + SIDE_CLEARANCE_CM < MAX_SIDE_STEP_CM)
                    && (freeSideMm >= 8000.0f || yGoal <= freeSideMm / 10.0f - 6.0f);
        gProbeGoal[si] = fits ? yGoal : 999.0f;
        float otherGoal = gProbeGoal[1 - si];
        if (!fits && otherGoal >= 999.0f) {
          brakeMotors();
          waitIntegrating(150);
          stopMotors();
          netEvent("Obstacle cannot be passed on either side (%s would need %.0f cm): stopping",
                   side < 0 ? "left" : "right", yEdgeUsed + ROBOT_HALF_WIDTH_CM + SIDE_CLEARANCE_CM);
          obstacleDetected = false;
          return false;
        }
        if (!fits) netEvent("No room on the %s (would need %.0f cm sideways)", side < 0 ? "left" : "right",
                            yEdgeUsed + ROBOT_HALF_WIDTH_CM + SIDE_CLEARANCE_CM);
        if (!fits || (!gNoProbe && yGoal > WIDE_PROBE_CM)) {
          float other = otherGoal;
          if (!fits || other < 0.0f || other + 5.0f < yGoal) {
            brakeMotors();
            waitIntegrating(150);
            stopMotors();
            gProbeAbort = true;
            gForceSide = -side;
            if (other >= 0.0f) gNoProbe = true;
            if (other < 0.0f)
              netEvent("Passing on the %s would take %.0f cm: trying the other side too", side < 0 ? "left" : "right", yGoal);
            else
              netEvent("%s side would take %.0f cm, the other %.0f cm: choosing the shorter one", side < 0 ? "left" : "right", yGoal, other);
            return false;
          }
        }
        Serial.print("[OUT] EDGE FOUND at Y = ");
        Serial.print(yEdgeUsed, 1);
        Serial.print(" (raw ");
        Serial.print(lastHitY, 1);
        Serial.print(", d=");
        Serial.print(lastHitD, 1);
        Serial.print(" cm)");
        Serial.print(" cm | lateral target: ");
        Serial.print(yGoal, 1);
        Serial.println(" cm");
      }
    }

    if (phase == 0 && devSigned >= OUT_ARC_ANGLE_DEG - ARC_TOL_DEG) {
      phase = 1;
      l1 = Y;
      Serial.print("[OUT] arc entry finished | lateral gained: ");
      Serial.println(l1, 1);
    }

    if (phase < 2) {

      float g = ARC_EXIT_K_CM * (1.0f - cosT);
      bool reached = edgeFound ? (Y + g >= yGoal) : (Y + g >= MAX_SIDE_STEP_CM);
      if (reached) {
        phase = 2;
        target = pathHeading;
        yAtExit = Y;
        devAtExit = devSigned;
        Serial.print("[OUT] leaving the arc | Y = ");
        Serial.print(Y, 1);
        Serial.print(" | angle = ");
        Serial.print(devSigned, 1);
        Serial.print(" | estimated gain while straightening = ");
        Serial.println(g, 1);
      }
    }

    if (phase == 2 && fabs(devSigned) <= ARC_END_TOL_DEG) {
      phase = 3;
      target = pathHeading;
      Serial.print("[OUT] REAL gain while straightening = ");
      Serial.print(Y - yAtExit, 1);
      Serial.print(" cm (K real = ");
      Serial.print((Y - yAtExit) / (1.0f - cosf(devAtExit * PI / 180.0f)), 1);
      Serial.println(")");
      Serial.print("[OUT] aligned with the route at X = ");
      Serial.print(X, 1);
      Serial.print(" cm, Y = ");
      Serial.print(Y, 1);
      Serial.println(" cm");
    }

    if (phase >= 2) {
      uint16_t sd = obstacleSensor.readRangeSingleMillimeters();
      bool sdTimeout = sensorFailed(obstacleSensor, sd);
      if (!sdTimeout) {
        if (side > 0) { gLeft = sd; gLeftT = millis(); } else { gRight = sd; gRightT = millis(); }
      }

      float sensX = X + SENSOR_AHEAD_CM * cosT + SIDE_SENSOR_OFFSET_CM * sinT;
      float sensY = Y + SENSOR_AHEAD_CM * sinT - SIDE_SENSOR_OFFSET_CM * cosT;
      (void)sensY;

      float wallExpMm = obstacleSideStartMm + Y * 10.0f;
      float nearTh = (float)SIDE_NEAR_MM;
      if (wallExpMm - 100.0f < nearTh) nearTh = wallExpMm - 100.0f;
      if (nearTh < 80.0f) nearTh = 80.0f;

      if (!sdTimeout && sd < nearTh) {

        float hitX = sensX + (sd / 10.0f) * sinT;
        sideSeen = true;
        if (hitX > boxMaxX) boxMaxX = hitX;
        sideFarCount = 0;
      } else if (!sdTimeout && sideSeen && sensX >= xFace + 2.0f) {

        sideFarCount++;
        if (!rearConfirmed && sideFarCount >= SIDE_CLEAR_CONFIRM) {
          rearConfirmed = true;
          if (boxMaxX < xFace + MIN_MEASURED_DEPTH_CM) boxMaxX = xFace + MIN_MEASURED_DEPTH_CM;
          gRearX = boxMaxX;
          netEvent("Rear of obstacle: estimated depth %.1f cm", boxMaxX - xFace);
          Serial.print("[OUT] rear of obstacle passed | estimated depth: ");
          Serial.print(boxMaxX - xFace, 1);
          Serial.println(" cm");
        }
      }
    }

    if (phase == 3) {
      const float xCap = xFace + MAX_OBSTACLE_CM;

      xPassEnd = xFace + OBSTACLE_MAX_DEPTH_CM + (AXLE_TO_REAR_CM - ARC_START_EARLY_CM);

      if (sideSeen) {
        if (rearConfirmed) {

          xPassEnd = boxMaxX + REAR_MARGIN_CM + (AXLE_TO_REAR_CM - ARC_START_EARLY_CM);
        } else {
          xPassEnd = xCap;
        }
      }

      if (xPassEnd > xCap) xPassEnd = xCap;

      if (X >= xPassEnd) {
        brakeMotors();
        waitIntegrating(150);
        stopMotors();
        lateralEndCm = Y;
        xEndCm = X;

        Serial.print("[DIAG] edge=");
        Serial.print(yEdgeUsed, 1);
        Serial.print(" lateral_target=");
        Serial.print(yGoal, 1);
        Serial.print(" Y_end_of_pass=");
        Serial.print(Y, 1);
        Serial.print(" X_end_of_pass=");
        Serial.println(X, 1);
        Serial.print("PASS FINISHED | X = ");
        Serial.print(X, 1);
        Serial.print(" cm | Y (lateral) = ");
        Serial.print(Y, 1);
        Serial.print(" cm | side sensor saw the obstacle: ");
        Serial.print(sideSeen ? "YES" : "NO");
        Serial.print(" | rear confirmed: ");
        Serial.println(rearConfirmed ? "YES" : "NO");

        waitIntegrating(250);
        return true;
      }
    }

    float error = headingTotal - target;
    int correction = (int)(KP * error);
    correction = constrain(correction, -ARC_MAX_CORRECTION, ARC_MAX_CORRECTION);

    int leftPWM = constrain(FORWARD_PWM + LEFT_TRIM - correction, 0, 255);
    int rightPWM = constrain(FORWARD_PWM + RIGHT_TRIM + correction, 0, 255);

    ledcWrite(PWMB, leftPWM);
    ledcWrite(PWMA, rightPWM);

    if (millis() - lastPrint >= 300) {
      lastPrint = millis();
      Serial.print("[OUT] phase ");
      Serial.print(phase);
      Serial.print(" | X: ");
      Serial.print(X, 1);
      Serial.print(" | Y: ");
      Serial.print(Y, 1);
      Serial.print(" | heading: ");
      Serial.print(headingTotal, 2);
      Serial.print(" | FRONT: ");
      Serial.println(frontDistance);
    }

    if (millis() - startTime >= timeoutMs) {
      brakeMotors();
      Serial.println("ERROR: ARC OUT TIMEOUT!");
      netEvent("Arc out stopped: timeout (angle never dropped below 4 degrees?)");
      delay(300);
      stopMotors();
      return false;
    }

    delay(5);
  }
}

const uint16_t AVOID_MIN_START_MM = 220;
const uint16_t AVOID_START_MM = 250;
const uint16_t AVOID_RETRY_MM = 350;
const float MAX_BACK_CM = 50.0;

const float WIDE_GOAL_CM = 25.0;
const float WIDE_FORWARD_PER_CM = 0.6;
const float WIDE_MARGIN_CM = 15.0;
uint16_t gWantStartMm = 0;

const float B_WALL_MARGIN_CM = 30.0;

bool obstacleIsAtB() {
  if (gFront < 0 || gFront >= 8000) return false;
  float th = (headingTotal - pathHeading) * PI / 180.0f;
  float obstX = gX + (SENSOR_AHEAD_CM + gFront / 10.0f) * cosf(th);
  return obstX >= ROUTE_LENGTH_CM - B_WALL_MARGIN_CM;
}

bool ensureRoomInFront() {
  uint16_t nearest = 8190;
  for (int i = 0; i < 3; i++) {
    uint16_t d = sensorFront.readRangeSingleMillimeters();
    if (!sensorFailed(sensorFront, d) && d < 8000 && d < nearest) nearest = d;
    delay(20);
  }
  uint16_t target = AVOID_START_MM, minOk = AVOID_MIN_START_MM;
  if (gWantStartMm > target) { target = gWantStartMm; minOk = target - 30; }
  gWantStartMm = 0;
  if (nearest >= minOk) return true;

  float backCm = constrain((target - nearest) / 10.0f, 0.0f, MAX_BACK_CM);
  netEvent("Obstacle at %d mm, too close to avoid: backing up %.0f cm", (int)nearest, backCm);
  return backUpCm(backCm);
}

bool prepareNewAvoidance(bool duringOut) {
  int seenMm = gFront;

  int lastSi = gLastAvoidSide > 0 ? 1 : 0;
  if (duringOut && gGoalY > WIDE_GOAL_CM && gProbeGoal[lastSi] < 999.0f) {
    float needCm = constrain(WIDE_FORWARD_PER_CM * gGoalY + WIDE_MARGIN_CM, 25.0f, 50.0f);
    gWantStartMm = (uint16_t)(needCm * 10.0f);
    gForceSide = gLastAvoidSide;
    gNestedStreak++;
    netEvent("Wide obstacle (lateral target %.0f cm): backing up to %.0f cm and retrying on the %s",
             (float)gGoalY, needCm, gLastAvoidSide < 0 ? "left" : "right");
    float backCm = 10.0f;
    if (seenMm >= 0 && seenMm < 8000) backCm = needCm - seenMm / 10.0f;
    backCm = constrain(backCm, 0.0f, MAX_BACK_CM);
    if (backCm > 1.0f && !backUpCm(backCm)) return false;
    gMode = "ALIGN";
    if (!turnToHeading(pathHeading)) return false;
    gNestedPending = true;
    return true;
  }

  netEvent("New obstacle during avoidance (%d mm): stopping and recalculating", seenMm);

  gNestedStreak++;
  uint16_t wantMm = AVOID_START_MM;
  if (gNestedStreak >= 2) {
    wantMm = AVOID_RETRY_MM;
    gForceSide = -gLastAvoidSide;
    netEvent("Blocked again: backing up further and trying the %s", gForceSide < 0 ? "left" : "right");
  }

  float backCm = 10.0f;
  if (seenMm >= 0 && seenMm < 8000) backCm = (wantMm - seenMm) / 10.0f;
  backCm = constrain(backCm, 0.0f, MAX_BACK_CM);

  if (backCm > 1.0f && !backUpCm(backCm)) return false;

  gMode = "ALIGN";
  if (!turnToHeading(pathHeading)) return false;

  gNestedPending = true;
  return true;
}

// One full avoidance: make room, choose a side, arc out and pass, return to the line, align.
bool avoidObstacleInner(float &advanceCm) {

  Serial.println();
  Serial.println("========== NAVCORE AVOIDANCE START (ALL ON ARCS) ==========");

  brakeMotors();
  waitIntegrating(300);
  stopMotors();

  if (!ensureRoomInFront()) return false;

  const float avoidanceStartHeading = pathHeading;

  printAvoidanceHeading("AVOIDANCE START", avoidanceStartHeading);

  int side = chooseAvoidanceSide();
  if (side != 0) gLastAvoidSide = side;

  if (side == 0) {
    Serial.println("NOT ENOUGH SIDE SPACE. ROBOT STOPPED.");
    return false;
  }

  VL53L0X &obstacleSideSensor = (side > 0) ? sensorLeft : sensorRight;

  Serial.println(
    side < 0 ? "SIDE CHOSEN: LEFT." : "SIDE CHOSEN: RIGHT."
  );

  float lateralEndCm = 0.0f;
  float xEndCm = 0.0f;
  float returnForwardCm = 0.0f;

  Serial.println("STEP 1: ARC OUT AND PASS THE OBSTACLE");

  if (!avoidOutArcAndPass(side, obstacleSideSensor, lateralEndCm, xEndCm)) {
    if (gProbeAbort) {

      gProbeAbort = false;
      float back = sqrtf((gX - gOx) * (gX - gOx) + (gY - gOy) * (gY - gOy)) + 3.0f;
      if (back > 1.0f && !backUpCm(constrain(back, 0.0f, MAX_BACK_CM))) return false;
      gMode = "ALIGN";
      if (!turnToHeading(pathHeading)) return false;
      gNestedPending = true;
      return true;
    }
    if (obstacleDetected && (gX >= ROUTE_LENGTH_CM - ARRIVAL_ZONE_CM || obstacleIsAtB())) {
      netEvent("Wall at B reached during avoidance: arrived");
      advanceCm = xEndCm;
      return true;
    }
    if (obstacleDetected) return prepareNewAvoidance(true);
    return false;
  }

  printAvoidanceHeading(
    "AFTER STEP 1 - OBSTACLE PASSED",
    avoidanceStartHeading
  );

  Serial.println("STEP 2: RETURN ARC TO THE ROUTE");

  float lateralToLine = side * gY;
  Serial.print("[RET] lateral from the pass: ");
  Serial.print(lateralEndCm, 1);
  Serial.print(" cm | from line A-B: ");
  Serial.print(lateralToLine, 1);
  Serial.println(" cm");

  netEvent("Return: %.1f cm to the line, angle %.1f", lateralToLine, headingTotal - pathHeading);
  if (lateralToLine > 1.0f) {
    if (!returnToLineArc(side, lateralToLine, returnForwardCm)) {

      if (obstacleDetected && (gX >= ROUTE_LENGTH_CM - ARRIVAL_ZONE_CM || obstacleIsAtB())) {
        netEvent("Wall at B reached during return: arrived");
        advanceCm = xEndCm + returnForwardCm;
        return true;
      }
      if (obstacleDetected) return prepareNewAvoidance(false);
      return false;
    }
  }

  printAvoidanceHeading(
    "AFTER STEP 2 - RETURN ARC",
    avoidanceStartHeading
  );

  gMode = "ALIGN";
  Serial.println("STEP 3: FINE ALIGNMENT WITH THE ROUTE");

  if (!turnToHeading(pathHeading)) return false;

  printAvoidanceHeading(
    "AFTER STEP 3 - FINAL ALIGNMENT",
    avoidanceStartHeading
  );

  brakeMotors();
  waitIntegrating(200);
  stopMotors();

  Serial.println();
  Serial.println("========== AVOIDANCE FINISHED ==========");

  advanceCm = xEndCm + returnForwardCm;

  Serial.print("Avoidance side: ");
  Serial.println(side < 0 ? "LEFT" : "RIGHT");
  Serial.print("Progress along the route during the avoidance: ");
  Serial.print(advanceCm, 1);
  Serial.println(" cm");

  return true;
}

bool avoidObstacle(float &advanceCm) {
  advanceCm = 0.0f;
  frontLimitMm = AVOID_SAFETY_MM;
  invalidIsNear = false;
  bool ok = avoidObstacleInner(advanceCm);
  frontLimitMm = AVOID_TRIGGER_MM;
  invalidIsNear = true;
  return ok;
}

void setup() {

  Serial.begin(115200);
  delay(1500);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  stopMotors();

  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  attachInterrupt(
    digitalPinToInterrupt(ENC_LEFT),
    isrR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_RIGHT),
    isrL,
    RISING
  );

  bool sensorsOk = false;
  for (int attempt = 0; attempt < 3 && !sensorsOk; attempt++) {
    if (attempt > 0) delay(300);
    Wire.end();
    unstickI2CBus();
    beginI2C();
    sensorsOk = initializeDistanceSensors();
  }

  if (!sensorsOk) {

    Serial.println("SENSOR INITIALISATION ERROR!");

    brakeMotors();
    delay(300);
    stopMotors();

    while (true) {
      delay(1000);
    }
  }

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() != 0) {

    Serial.println("MPU6050 NOT DETECTED!");

    brakeMotors();
    delay(300);
    stopMotors();

    while (true) {
      delay(1000);
    }
  }

  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  writeRegister(GYRO_CONFIG, 0x08);
  delay(50);

  if (!calibrateGyro()) {

    Serial.println("CALIBRATION ERROR!");

    brakeMotors();
    delay(300);
    stopMotors();

    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("====================================");
  Serial.println("NAVCORE - ROUTE WITH WI-FI");
  Serial.println("Obstacle avoidance | real-time Wi-Fi telemetry");
  Serial.println("====================================");

  frontLimitMm = AVOID_TRIGGER_MM;
  gMode = "IDLE";

  loadCalibration();
  netSetup();

  esp_reset_reason_t why = esp_reset_reason();
  if (why == ESP_RST_BROWNOUT)
    netEvent("WARNING: robot restarted after a brown-out - weak battery or motors drawing too much current");
  else if (why == ESP_RST_PANIC || why == ESP_RST_INT_WDT || why == ESP_RST_TASK_WDT || why == ESP_RST_WDT)
    netEvent("WARNING: robot restarted after a software error (code %d)", (int)why);

  Serial.println();
  Serial.println("Ready. Start from the web app or send s in the Serial Monitor; x stops.");
}

static void endRun(const char *mode, const char *msg) {
  brakeMotors();
  delay(300);
  stopMotors();
  gMode = gAbortReq ? "STOPPED" : mode;
  netEvent("%s", gAbortReq ? "Stopped remotely" : msg);
  Serial.println(gAbortReq ? "STOPPED REMOTELY." : msg);
  gRunning = false;
}

// A run: follow the line to B and avoid every obstacle on the way.
void runRoute() {

  pathHeading = headingTotal;
  gX = 0;
  gY = 0;

  noInterrupts();
  countL = 0;
  countR = 0;
  interrupts();
  odoLastAvg = 0;
  gOx = gOy = gEdgeY = gGoalY = gRearX = gFaceX = -999;
  gSide = 0;
  gNestedStreak = 0;
  gForceSide = 0;
  gLastAvoidSide = 0;
  gWantStartMm = 0;
  netPose(0, 0, true);
  gRunning = true;
  gAbortReq = false;
  gAvoid = 0;
  gMode = "DRIVE";

  Serial.println();
  Serial.print("Starting route ");
  Serial.print(ROUTE_LENGTH_CM, 0);
  Serial.println(" cm in 2 seconds...");
  netEvent("Route started: %.0f cm", ROUTE_LENGTH_CM);
  delay(500);

  gMode = "CALIB";
  if (!calibrateGyro()) {
    endRun("FAILED", "Gyro calibration failed");
    return;
  }
  pathHeading = headingTotal;
  gMode = "DRIVE";

  invalidIsNear = true;
  frontLimitMm = AVOID_TRIGGER_MM;

  float progressCm = 0.0f;
  int avoidances = 0;

  while (progressCm < ROUTE_LENGTH_CM - 2.0f) {

    float remainingCm = ROUTE_LENGTH_CM - progressCm;

    Serial.println();
    Serial.print("[ROUTE] progress ");
    Serial.print(progressCm, 1);
    Serial.print(" / ");
    Serial.print(ROUTE_LENGTH_CM, 0);
    Serial.println(" cm");

    gMode = "DRIVE";
    float segX0 = gX;
    unsigned long segT0 = millis();
    bool segmentFinished = moveForwardCm(remainingCm, pathHeading, true, true);
    progressCm = gX;

    float segS = (millis() - segT0) / 1000.0f;
    if (segS > 0.5f && (gX - segX0) / segS > 40.0f) {
      netEvent("WARNING: encoders %.0f cm/s - false pulses, distance is wrong", (gX - segX0) / segS);
    }

    if (segmentFinished) {
      Serial.println("[ROUTE] destination B reached.");
      break;
    }

    if (!obstacleDetected) {
      endRun("FAILED", "Route stopped because of an error");
      return;
    }

    if (ROUTE_LENGTH_CM - progressCm <= ARRIVAL_ZONE_CM || obstacleIsAtB()) {
      netEvent("Obstacle ahead is at B (%d mm): arrived", (int)gFront);
      Serial.println("[ROUTE] obstacle in the arrival zone (wall at B): destination reached.");
      break;
    }

    avoidances++;
    gOx = gOy = gEdgeY = gGoalY = gRearX = gFaceX = -999;
    gSide = 0;
    gAvoid = avoidances;

    if (avoidances > MAX_AVOIDANCES) {
      endRun("FAILED", "Too many avoidances on this route");
      return;
    }

    Serial.print("Obstacle detected (avoidance ");
    Serial.print(avoidances);
    Serial.println("). Starting the avoidance manoeuvre.");
    netEvent("Obstacle at %d mm: avoidance %d", (int)gFront, avoidances);

    float advanceCm = 0.0f;

    gNestedPending = false;
    gProbeGoal[0] = gProbeGoal[1] = -1.0f;
    gNoProbe = false;
    gProbeAbort = false;
    bool avoidOk = avoidObstacle(advanceCm);

    while (avoidOk && gNestedPending) {
      gNestedPending = false;
      avoidances++;
      gOx = gOy = gEdgeY = gGoalY = gRearX = gFaceX = -999;
      gSide = 0;
      gAvoid = avoidances;
      if (avoidances > MAX_AVOIDANCES) {
        endRun("FAILED", "Too many avoidances on this route");
        return;
      }
      netEvent("Avoidance %d, recalculated from where the robot stopped", avoidances);
      avoidOk = avoidObstacle(advanceCm);
    }

    if (!avoidOk) {
      endRun("FAILED", "Avoidance failed, robot stopped safely");
      return;
    }
    gNestedStreak = 0;
    gForceSide = 0;

    progressCm = gX;
    netEvent("Avoidance finished at X=%.1f cm, offset %.1f cm", (float)gX, (float)gY);
  }

  Serial.print("Avoidances done: ");
  Serial.println(avoidances);

  brakeMotors();
  waitIntegrating(300);
  stopMotors();
  gMode = "ALIGN";
  if (fabs(headingTotal - pathHeading) > HEADING_TOLERANCE && !turnToHeading(pathHeading)) {
    netEvent("Reached B, but the final alignment failed (angle %.1f)", headingTotal - pathHeading);
  }

  endRun("ARRIVED", "Destination B reached");
}

// 360 degree turn used by the app to calibrate GYRO_SCALE.
void runTurnTest() {
  gRunning = true;
  gAbortReq = false;
  gTurnTestDeg = -1;
  gMode = "CALIB";
  netEvent("Turn test: do not move the robot (gyro calibration)");
  delay(500);
  if (!calibrateGyro()) {
    endRun("FAILED", "Gyro calibration failed");
    return;
  }

  gMode = "TURN";
  netEvent("Turn test: one full turn to the right");
  TurnGuard turnGuard;
  yaw = 0;
  unsigned long previousTime = micros();
  unsigned long startTime = millis();
  float brakeAt = 360.0f - rightOvershootEstimate;

  setRightTurn(TURN_PWM);
  while (yaw < brakeAt) {
    if (!updateYaw(previousTime)) {
      endRun("FAILED", "Turn test stopped");
      return;
    }
    if (yaw >= 360.0f - SLOWDOWN_ANGLE) {
      ledcWrite(PWMA, SLOW_PWM);
      ledcWrite(PWMB, SLOW_PWM);
    }
    if (millis() - startTime >= 20000) {
      endRun("FAILED", "Turn test: timeout (is the robot turning?)");
      return;
    }
    delay(5);
  }

  brakeMotors();
  unsigned long stopTime = millis();
  while (millis() - stopTime < MEASURE_AFTER_STOP) {
    if (!updateYaw(previousTime)) break;
    delay(10);
  }
  stopMotors();

  gTurnTestDeg = yaw;
  netEvent("Turn test: gyro measured %.1f degrees", yaw);
  endRun("IDLE", "Turn test finished");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') gStartReq = true;
  }

  if (gStartReq) {
    gStartReq = false;
    runRoute();
  }

  if (gTurnTestReq) {
    gTurnTestReq = false;
    runTurnTest();
  }

  static unsigned long lastIdleRead = 0;
  static int idleFails = 0;
  if (millis() - lastIdleRead >= 150) {
    lastIdleRead = millis();
    uint16_t l, f, r;
    if (readAllDistances(l, f, r)) idleFails = 0;
    else if (++idleFails >= 3) { idleFails = 0; recoverDistanceSensors(); }
  }

  delay(10);
}
