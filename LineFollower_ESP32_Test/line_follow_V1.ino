
/*
  XE DO LINE - ESP32 DEV MODULE
  QTR-8A + L298N
  LINE DEN - NEN TRANG

  - Cap nguon: tu hieu chuan nen trang 2 giay.
  - Sau do tu dong chay.
  - Duong thang: hai banh cung PWM.
  - Vao cua: PD tu dieu chinh toc do hai banh.
  - Kalman loc vi tri line.
  - Mat line: tu tim lai, khong tim duoc thi dung.

  QTR OUT1 -> OUT8: TRAI -> PHAI
*/

// ================== CAM BIEN QTR-8A ==================

const uint8_t SENSOR_COUNT = 8;

const uint8_t SENSOR_PINS[SENSOR_COUNT] = {
  27, 32, 26, 35, 25, 34, 33, 39
};

// ================== L298N ==================

const uint8_t ENA = 5;
const uint8_t IN1 = 18;
const uint8_t IN2 = 19;

const uint8_t IN3 = 23;
const uint8_t IN4 = 22;
const uint8_t ENB = 21;

// ================== PWM ==================

const uint32_t PWM_FREQ = 18000;
const uint8_t PWM_BITS = 8;

const uint8_t LEFT_CHANNEL = 0;
const uint8_t RIGHT_CHANNEL = 1;

// ================== THAM SO XE ==================

int BASE_SPEED = 125;

const int MIN_SPEED = 50;
const int MAX_SPEED = 210;

const int MAX_CORRECTION = 100;

// PD
float KP = 110.0f;
float KD = 170.0f;

// Vung chet: giu hai banh cung toc do khi di thang
const float STRAIGHT_DEADBAND = 0.025f;

// ================== HIEU CHUAN ==================

uint16_t whiteLevel[SENSOR_COUNT];

const uint32_t CALIBRATION_TIME = 2000;

// QTR-8A: den co raw cao hon nen trang
const int BLACK_THRESHOLD = 100;
const int BLACK_SCALE = 700;

// ================== KALMAN ==================

float kalmanPosition = 0.0f;
float kalmanVariance = 1.0f;

const float KALMAN_Q = 0.035f;
const float KALMAN_R = 0.12f;

// ================== TRANG THAI ==================

float previousError = 0.0f;

int lastLineSide = 0;

uint32_t lastLineSeen = 0;
uint32_t lastControlTime = 0;

bool motorsEnabled = false;

// ================== DOC CAM BIEN ==================

uint16_t readSensor(uint8_t index) {
  uint32_t total = 0;

  for (int i = 0; i < 4; i++) {
    total += analogRead(SENSOR_PINS[index]);
    delayMicroseconds(60);
  }

  return total / 4;
}

// ================== DIEU KHIEN DONG CO ==================

void writeMotor(
  uint8_t channel,
  uint8_t pinA,
  uint8_t pinB,
  int speed
) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  }
  else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  uint8_t pwmPin = (channel == LEFT_CHANNEL) ? ENA : ENB;
  ledcWrite(pwmPin, abs(speed));
#else
  ledcWrite(channel, abs(speed));
#endif
}

void setMotors(int leftSpeed, int rightSpeed) {

  if (!motorsEnabled) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  writeMotor(
    LEFT_CHANNEL,
    IN1,
    IN2,
    leftSpeed
  );

  writeMotor(
    RIGHT_CHANNEL,
    IN3,
    IN4,
    rightSpeed
  );
}

void stopMotors() {
  motorsEnabled = false;

  writeMotor(LEFT_CHANNEL, IN1, IN2, 0);
  writeMotor(RIGHT_CHANNEL, IN3, IN4, 0);
}

// ================== HIEU CHUAN NEN TRANG ==================

void calibrateWhite() {

  stopMotors();

  uint32_t sum[SENSOR_COUNT] = {0};

  uint32_t startTime = millis();
  uint32_t sampleCount = 0;

  while (millis() - startTime < CALIBRATION_TIME) {

    for (int i = 0; i < SENSOR_COUNT; i++) {
      sum[i] += analogRead(SENSOR_PINS[i]);
    }

    sampleCount++;
    delay(4);
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    whiteLevel[i] = sum[i] / sampleCount;
  }
}

// ================== DOC VI TRI LINE ==================

bool readLinePosition(
  float &position,
  int &totalStrength
) {

  long weightedSum = 0;
  totalStrength = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {

    uint16_t raw = readSensor(i);

    // Muc den = raw - gia tri nen trang
    int strength =
      (int)raw - (int)whiteLevel[i] - BLACK_THRESHOLD;

    if (strength < 0) {
      strength = 0;
    }

    strength = (strength * 1000) / BLACK_SCALE;

    strength = constrain(strength, 0, 1000);

    if (strength < 80) {
      strength = 0;
    }

    // S1 = -3500, S8 = +3500
    int sensorPosition = i * 1000 - 3500;

    weightedSum += (long)strength * sensorPosition;

    totalStrength += strength;
  }

  if (totalStrength < 130) {
    return false;
  }

  position =
    (float)weightedSum / (float)totalStrength;

  position /= 3500.0f;

  position = constrain(position, -1.0f, 1.0f);

  return true;
}

// ================== BO LOC KALMAN ==================

float kalmanFilter(float measurement) {

  kalmanVariance += KALMAN_Q;

  float gain =
    kalmanVariance /
    (kalmanVariance + KALMAN_R);

  kalmanPosition +=
    gain * (measurement - kalmanPosition);

  kalmanVariance *= (1.0f - gain);

  return kalmanPosition;
}

// ================== TIM LAI LINE ==================

void recoverLine() {

  uint32_t lostTime = millis() - lastLineSeen;

  const int SEARCH_SPEED = 85;

  if (lostTime < 250) {

    if (lastLineSide < 0) {
      setMotors(-30, SEARCH_SPEED);
    }
    else {
      setMotors(SEARCH_SPEED, -30);
    }

  }
  else if (lostTime < 650) {

    if (lastLineSide < 0) {
      setMotors(0, SEARCH_SPEED);
    }
    else {
      setMotors(SEARCH_SPEED, 0);
    }

  }
  else {

    stopMotors();
  }
}

// ================== DIEU KHIEN PD ==================

void followLine() {

  float measuredPosition = 0.0f;
  int totalStrength = 0;

  bool found = readLinePosition(
    measuredPosition,
    totalStrength
  );

  if (!found) {
    recoverLine();
    return;
  }

  lastLineSeen = millis();

  float error = kalmanFilter(measuredPosition);

  if (error < -0.06f) {
    lastLineSide = -1;
  }
  else if (error > 0.06f) {
    lastLineSide = 1;
  }

  // Vung giu thang
  if (fabs(error) < STRAIGHT_DEADBAND) {
    error = 0.0f;
  }

  float derivative = error - previousError;

  float correction =
    KP * error + KD * derivative;

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  // Giam toc do co so khi vao cua
  float absError = fabs(error);

  int currentBase = BASE_SPEED;

  if (absError > 0.65f) {
    currentBase = BASE_SPEED - 35;
  }
  else if (absError > 0.35f) {
    currentBase = BASE_SPEED - 20;
  }

  currentBase = max(currentBase, MIN_SPEED);

  // Di thang: correction = 0
  // Hai banh cung toc do PWM

  // Line ben phai: banh trai nhanh hon
  // Line ben trai: banh phai nhanh hon

  int leftPWM =
    currentBase + (int)correction;

  int rightPWM =
    currentBase - (int)correction;

  leftPWM = constrain(
    leftPWM,
    MIN_SPEED,
    MAX_SPEED
  );

  rightPWM = constrain(
    rightPWM,
    MIN_SPEED,
    MAX_SPEED
  );

  setMotors(leftPWM, rightPWM);

  previousError = error;
}

// ================== SETUP ==================

void setup() {

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  analogReadResolution(12);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    analogSetPinAttenuation(
      SENSOR_PINS[i],
      ADC_11db
    );
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);

#else

  ledcSetup(LEFT_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_BITS);

  ledcAttachPin(ENA, LEFT_CHANNEL);
  ledcAttachPin(ENB, RIGHT_CHANNEL);

#endif

  stopMotors();

  // Xe phai dung tren nen trang luc khoi dong
  calibrateWhite();

  kalmanPosition = 0.0f;
  kalmanVariance = 1.0f;

  previousError = 0.0f;

  lastLineSeen = millis();

  delay(500);

  motorsEnabled = true;

  lastControlTime = micros();
}

// ================== LOOP ==================

void loop() {

  const uint32_t CONTROL_PERIOD = 5000;

  uint32_t now = micros();

  if ((uint32_t)(now - lastControlTime) >= CONTROL_PERIOD) {

    lastControlTime = now;

    followLine();
  }
}
