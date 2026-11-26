// File nhóm đang sử dụng để chạy
/*
 * ===================================================================
 *  XE DÒ LINE - ESP32 DEV MODULE (v4 - bản đã rà soát & sửa lỗi từ v3)
 *  QTR-8A (analog) + HC-SR04 + L298N + 2x Encoder GA25 (Pha A & B)
 *  Điều khiển: PD (đạo hàm có lọc), tự hiệu chuẩn, dừng vật cản
 *  (non-blocking), chỉnh thông số qua Serial.
 * ===================================================================
 *
 *  SƠ ĐỒ CHÂN (giữ nguyên như v3):
 *  - L298N (Bánh TRÁI - OUT3/OUT4) : ENB->21, IN3->23, IN4->22
 *  - L298N (Bánh PHẢI - OUT1/OUT2) : ENA->5,  IN1->18, IN2->19
 *  - Encoder TRÁI                  : A->17, B->16
 *  - Encoder PHẢI                  : A->4,  B->2   (GPIO2 là chân strapping!)
 *  - QTR-8A (OUT1..OUT8)           : 27, 32, 26, 35, 25, 34, 33, 39
 *  - HC-SR04                       : TRIG->12 (strapping!), ECHO->13 (chia áp 5V->3.3V)
 *
 *  LỆNH SERIAL (115200, Newline):
 *    c          : hiệu chuẩn cảm biến (4 giây)
 *    s          : bắt đầu chạy       | x   : dừng
 *    kp 0.05    : đặt Kp             | kd 1.0 : đặt Kd  | ki 0.0 : đặt Ki
 *    base 150   : tốc độ cơ bản      | max 230 : tốc độ tối đa
 *    lost 140   : tốc độ xoay khi mất line
 *    dbg        : bật/tắt debug      | p   : in thông số hiện tại
 */

#include <Arduino.h>
#include <Preferences.h>

// ============================ CẤU HÌNH ==============================
#define AUTO_START     true     // true: bật nguồn là tự chạy (không cần Serial)
const uint32_t START_DELAY_MS = 3000;              // chờ bao lâu sau khi bật nguồn rồi mới chạy
#define BLACK_LINE      true     // true: line đen nền trắng | false: line trắng nền đen
#define AUTO_CAL_SPIN   false    // true: xe tự xoay tại chỗ khi hiệu chuẩn
#define USE_ENCODER     1        // 1: bật đọc encoder | 0: tắt hẳn ngắt encoder
#define CAL_TIME_MS     4000
#define NUM_SENSORS     8

// QTR-8A: OUT1..OUT8 (OUT1 = ngoài cùng TRÁI; nếu bị ngược hãy đảo thứ tự mảng này)
const uint8_t SENSOR_PINS[NUM_SENSORS] = {39,33,34,25,35,26,32,27};

// L298N (Trái = OUT3/OUT4, Phải = OUT1/OUT2)
const uint8_t EN_R = 5, IN1_R = 18, IN2_R = 19;   // Động cơ PHẢI (ENB, IN1, IN2)
const uint8_t EN_L = 21,  IN1_L = 23, IN2_L = 22;   // Động cơ TRÁI (ENA, IN3, IN4)
const bool INVERT_LEFT  = false;                   // đổi thành true nếu bánh trái quay ngược
const bool INVERT_RIGHT = true;                   // đổi thành true nếu bánh phải quay ngược

// HC-SR04
const uint8_t TRIG_PIN = 12, ECHO_PIN = 13;
const float    OBSTACLE_CM   = 15.0f;
const uint8_t  OBSTACLE_HITS = 2;                  // số lần đo liên tiếp để xác nhận vật cản (2 x 60ms)
const uint32_t US_MIN_WIDTH  = 120;                // us; xung ngắn hơn (~2cm) coi là nhiễu

// Mất line
const uint32_t LOST_TIMEOUT_MS = 2000;             // xoay tìm line quá lâu thì dừng hẳn

// Encoder GA25 (Pha A + Pha B)
const uint8_t ENC_L_A = 17, ENC_L_B = 16;          // Trái
const uint8_t ENC_R_A = 4,  ENC_R_B = 2;           // Phải

// PWM
const uint32_t PWM_FREQ = 5000;
const uint8_t  PWM_BITS = 8;                       // 0..255

// Vòng điều khiển
const uint32_t LOOP_US = 2000;                     // 500 Hz

// Lọc đạo hàm: 1.0 = không lọc, càng nhỏ càng mượt (nhưng trễ hơn)
const float DERIV_ALPHA = 0.4f;

// ======================= THAM SỐ CÓ THỂ CHỈNH ========================
float Kp = 0.050f, Ki = 0.0f, Kd = 1.0f;
int   baseSpeed = 180;
int   maxSpeed  = 230;
int   minSpeed  = -80;                             // cho phép đảo chiều nhẹ khi vào cua gắt
int   lostSpin  = 150;                             // tốc độ xoay khi mất line

// ============================ BIẾN TOÀN CỤC ===========================
uint16_t calMin[NUM_SENSORS], calMax[NUM_SENSORS];
uint16_t sv[NUM_SENSORS];                          // giá trị chuẩn hóa 0..1000
bool     calibrated = false;

enum State { IDLE, CALIBRATING, RUNNING };
State state = IDLE;

float    lastErr = 0, integral = 0, derivFilt = 0;
bool     havePrev = false;                         // tránh derivative kick
int      lastSide = 0;                             // -1: lệch trái, +1: lệch phải
bool     debugOn = false;
bool     wasLost = false;
bool     obsPrinted = false;
uint32_t lostSinceMs = 0;

#if USE_ENCODER
volatile int32_t encL = 0, encR = 0;
int32_t  lastEncL = 0, lastEncR = 0;
int32_t  speedL = 0, speedR = 0;                   // xung có dấu / 50 ms

void IRAM_ATTR isrL() {
  if (digitalRead(ENC_L_B)) encL--;
  else                      encL++;
}
void IRAM_ATTR isrR() {
  if (digitalRead(ENC_R_B)) encR--;
  else                      encR++;
}
#endif

// ---- HC-SR04 non-blocking ----
volatile uint32_t echoRiseUs  = 0;
volatile uint32_t echoWidthUs = 0;
volatile bool     echoRising  = false;
volatile bool     echoDone    = false;
float    distCm = 999;
uint8_t  obstacleCount = 0;
bool     obstacle = false;
bool     usWaiting = false;
uint32_t usTrigMs = 0;

// =============================== PWM ==================================
void pwmBegin(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void driveOne(uint8_t en, uint8_t chn, uint8_t inA, uint8_t inB, int spd, bool inv) {
  if (inv) spd = -spd;
  if (spd > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else if (spd < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); spd = -spd; }
  else              { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }
  pwmWrite(en, chn, (uint8_t)constrain(spd, 0, 255));
}

void setMotors(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  driveOne(EN_L, 0, IN1_L, IN2_L, l, INVERT_LEFT);
  driveOne(EN_R, 1, IN1_R, IN2_R, r, INVERT_RIGHT);
}

void stopMotors() { setMotors(0, 0); }             // thả trôi (coast)

// Phanh: hai chân IN cùng mức HIGH + EN full -> L298N đoản mạch cuộn dây, dừng nhanh
void brakeMotors() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, HIGH); pwmWrite(EN_L, 0, 255);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, HIGH); pwmWrite(EN_R, 1, 255);
}

// ============================== HC-SR04 ===============================
void IRAM_ATTR isrEcho() {
  if (digitalRead(ECHO_PIN)) {
    echoRiseUs = micros();
    echoRising = true;
  } else if (echoRising) {
    echoWidthUs = micros() - echoRiseUs;
    echoRising  = false;
    echoDone    = true;
  }
}

void usReset() {
  noInterrupts();
  echoDone = false;
  echoRising = false;
  interrupts();
  usWaiting = false;
  obstacleCount = 0;
  obstacle = false;
  distCm = 999;
  usTrigMs = millis();
}

void usProcess(float cm) {
  distCm = cm;
  if (cm < OBSTACLE_CM) { if (obstacleCount < OBSTACLE_HITS + 2) obstacleCount++; }
  else                  { if (obstacleCount > 0) obstacleCount--; }
  obstacle = (obstacleCount >= OBSTACLE_HITS);
}

void updateObstacle(uint32_t ms) {
  if (usWaiting) {
    if (echoDone) {
      uint32_t w;
      noInterrupts(); w = echoWidthUs; echoDone = false; interrupts();
      usWaiting = false;
      if (w < US_MIN_WIDTH || w > 23200) usProcess(999.0f);   // nhiễu hoặc ngoài tầm
      else                               usProcess(w * 0.0343f / 2.0f);
    } else if (ms - usTrigMs > 30) {
      usWaiting = false;
      echoRising = false;
      usProcess(999.0f);
    }
  } else if (ms - usTrigMs >= 60) {
    echoDone = false;
    echoRising = false;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    usTrigMs = ms;
    usWaiting = true;
  }
}

// ============================== QTR-8A ================================
// Đọc bỏ 1 lần đầu (xả điện tích còn lại từ kênh ADC trước), rồi lấy trung bình 2 lần
uint16_t readOne(uint8_t pin) {
  analogRead(pin);
  return (analogRead(pin) + analogRead(pin)) / 2;
}

void readRaw(uint16_t *raw) {
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = readOne(SENSOR_PINS[i]);
}

// ---- Lưu/đọc hiệu chuẩn và thông số vào flash (còn nguyên khi mất điện) ----
Preferences prefs;

void saveCal() {
  prefs.begin("linebot", false);
  prefs.putBytes("calMin", calMin, sizeof(calMin));
  prefs.putBytes("calMax", calMax, sizeof(calMax));
  prefs.end();
}

bool loadCal() {
  prefs.begin("linebot", false);
  bool ok = prefs.getBytesLength("calMin") == sizeof(calMin) &&
            prefs.getBytesLength("calMax") == sizeof(calMax);
  if (ok) {
    prefs.getBytes("calMin", calMin, sizeof(calMin));
    prefs.getBytes("calMax", calMax, sizeof(calMax));
  }
  prefs.end();
  if (ok) calibrated = true;
  return ok;
}

void saveParams() {
  prefs.begin("linebot", false);
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putInt("base", baseSpeed); prefs.putInt("max", maxSpeed); prefs.putInt("lost", lostSpin);
  prefs.end();
}

void loadParams() {
  prefs.begin("linebot", false);
  Kp = prefs.getFloat("kp", Kp); Ki = prefs.getFloat("ki", Ki); Kd = prefs.getFloat("kd", Kd);
  baseSpeed = prefs.getInt("base", baseSpeed);
  maxSpeed  = prefs.getInt("max", maxSpeed);
  lostSpin  = prefs.getInt("lost", lostSpin);
  prefs.end();
}

void resetCal() {
  for (int i = 0; i < NUM_SENSORS; i++) { calMin[i] = 4095; calMax[i] = 0; }
}

void calibrateStep() {
  uint16_t raw[NUM_SENSORS];
  readRaw(raw);
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (raw[i] < calMin[i]) calMin[i] = raw[i];
    if (raw[i] > calMax[i]) calMax[i] = raw[i];
  }
}

void runCalibration(bool spin) {
  Serial.println(F("Hieu chuan: quet xe qua lai tren line..."));
  stopMotors();
  resetCal();
  uint32_t t0 = millis();
  if (spin) setMotors(90, -90);
  while (millis() - t0 < CAL_TIME_MS) { calibrateStep(); delay(2); }
  stopMotors();
  bool ok = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    bool bad = ((int)calMax[i] - (int)calMin[i]) < 200;
    if (bad) ok = false;
    Serial.printf("S%d (GPIO%u): min=%u max=%u%s\n", i + 1, SENSOR_PINS[i],
                  calMin[i], calMax[i], bad ? "  <-- DAI HEP, kiem tra day/khoang cach/quet lai" : "");
  }
  calibrated = true;
  saveCal();                                       // nhớ lại cho lần bật nguồn sau
  Serial.println(ok ? F("Xong. Gui 's' de chay.")
                    : F("Xong nhung co cam bien loi (xem tren). Nen gui 'c' lai."));
}

int readPosition(bool &onLine) {
  uint16_t raw[NUM_SENSORS];
  readRaw(raw);
  long wsum = 0, sum = 0;
  int  maxV = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int range = (int)calMax[i] - (int)calMin[i];
    int v = 0;
    if (range > 50) {
      v = ((int)raw[i] - (int)calMin[i]) * 1000 / range;
      v = constrain(v, 0, 1000);
      if (!BLACK_LINE) v = 1000 - v;               // chỉ đảo khi cảm biến có dải hợp lệ
    }
    sv[i] = v;
    if (v > maxV) maxV = v;
    if (v > 150) {
      wsum += (long)v * i * 1000;
      sum  += v;
    }
  }
  onLine = (maxV > 250 && sum > 0);
  if (!onLine) return (lastSide < 0) ? 0 : (lastSide > 0 ? 7000 : 3500);
  int pos = (int)(wsum / sum);
  lastSide = (pos < 3500) ? -1 : +1;               // luôn có hướng, để mất line còn biết xoay phía nào
  return pos;
}

// ============================ ĐIỀU KHIỂN ==============================
void resetPid() {
  integral = 0;
  derivFilt = 0;
  havePrev = false;
}

void controlStep() {
  bool onLine;
  int pos = readPosition(onLine);

  if (obstacle != obsPrinted) {                    // báo khi trạng thái vật cản đổi
    obsPrinted = obstacle;
    if (obstacle) Serial.printf("VAT CAN: %.0f cm -> phanh\n", distCm);
    else          Serial.println(F("Het vat can"));
  }

  if (obstacle) {
    brakeMotors();
    resetPid();
    return;
  }

  if (!onLine) {
    resetPid();
    if (!wasLost) { wasLost = true; lostSinceMs = millis(); }
    // Chưa biết lệch phía nào, hoặc xoay quá lâu -> dừng hẳn (an toàn)
    if (lastSide == 0 || millis() - lostSinceMs > LOST_TIMEOUT_MS) {
      brakeMotors();
      state = IDLE;
      Serial.println(F("MAT LINE -> DUNG. Dat xe len line, gui 's' de chay lai."));
      return;
    }
    if (lastSide < 0) setMotors(-lostSpin, lostSpin);
    else              setMotors(lostSpin, -lostSpin);
    return;
  }
  wasLost = false;

  float err = (float)pos - 3500.0f;
  if (!havePrev) { lastErr = err; havePrev = true; }
  integral = constrain(integral + err, -20000.0f, 20000.0f);
  float deriv = err - lastErr;
  lastErr = err;
  derivFilt += DERIV_ALPHA * (deriv - derivFilt);  // lọc thông thấp, giảm giật do lượng tử ADC

  float corr = Kp * err + Ki * integral + Kd * derivFilt;

  float slow = 1.0f - 0.35f * fabsf(err) / 3500.0f;
  int base = (int)(baseSpeed * 1.0);

  int l = constrain(base + (int)corr, minSpeed, maxSpeed);
  int r = constrain(base - (int)corr, minSpeed, maxSpeed);
  setMotors(l, r);
}

// ============================== SERIAL ================================
void printParams() {
  Serial.printf("Kp=%.4f Ki=%.5f Kd=%.3f base=%d max=%d lost=%d\n",
                Kp, Ki, Kd, baseSpeed, maxSpeed, lostSpin);
}

void processCommand(char *buf) {
  for (char *p = buf; *p; p++) *p = tolower((unsigned char)*p);

  if      (!strcmp(buf, "c"))   { state = CALIBRATING; return; }
  else if (!strcmp(buf, "s"))   {
    if (!calibrated) Serial.println(F("Chua hieu chuan! Gui 'c' truoc."));
    else {
      resetPid(); usReset(); lastSide = 0; wasLost = false;
      state = RUNNING; Serial.println(F("RUN"));
    }
    return;
  }
  else if (!strcmp(buf, "x"))   { state = IDLE; brakeMotors(); Serial.println(F("STOP")); return; }
  else if (!strcmp(buf, "dbg")) {
    debugOn = !debugOn;
    Serial.println(debugOn ? F("DEBUG ON") : F("DEBUG OFF"));
    return;
  }
  else if (!strcmp(buf, "p"))   { printParams(); return; }
  else if (!strcmp(buf, "save")) { saveParams(); Serial.println(F("Da luu thong so vao flash.")); return; }
  else if (!strcmp(buf, "clear")) {
    prefs.begin("linebot", false); prefs.clear(); prefs.end();
    calibrated = false;
    Serial.println(F("Da xoa hieu chuan va thong so da luu."));
    return;
  }
  else if (!strcmp(buf, "test")) {                 // thử từng động cơ, mỗi bánh quay tới 1 giây
    state = IDLE;
    Serial.println(F("Test: banh TRAI quay toi..."));
    setMotors(150, 0); delay(1000);
    Serial.println(F("Test: banh PHAI quay toi..."));
    setMotors(0, 150); delay(1000);
    stopMotors();
    Serial.println(F("Xong test."));
    return;
  }
  else if (!strcmp(buf, "raw")) {                  // in giá trị ADC thô của 8 cảm biến
    uint16_t raw[NUM_SENSORS];
    readRaw(raw);
    for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("%4u ", raw[i]);
    Serial.printf("| dist=%.0f cm\n", distCm);
    return;
  }

  char key[16];
  float v;
  if (sscanf(buf, "%15s %f", key, &v) == 2) {
    if      (!strcmp(key, "kp"))   Kp = max(0.0f, v);
    else if (!strcmp(key, "ki"))   Ki = max(0.0f, v);
    else if (!strcmp(key, "kd"))   Kd = max(0.0f, v);
    else if (!strcmp(key, "base")) baseSpeed = constrain((int)v, 0, 255);
    else if (!strcmp(key, "max"))  maxSpeed  = constrain((int)v, 0, 255);
    else if (!strcmp(key, "lost")) lostSpin  = constrain((int)v, 0, 255);
    else { Serial.println(F("Lenh khong hop le")); return; }
    if (baseSpeed > maxSpeed) baseSpeed = maxSpeed;
    printParams();
  } else {
    Serial.println(F("Lenh khong hop le"));
  }
}

void handleSerial() {
  static char    cmdBuf[32];
  static uint8_t cmdLen = 0;
  static bool    overflow = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflow) {                              // dòng quá dài: bỏ cả dòng, không xử lý phần đuôi
        Serial.println(F("Lenh qua dai"));
        overflow = false;
      } else if (cmdLen) {
        cmdBuf[cmdLen] = 0;
        processCommand(cmdBuf);
      }
      cmdLen = 0;
    } else if (!overflow) {
      if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
      else overflow = true;
    }
  }
}

// ============================== SETUP/LOOP ============================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pwmBegin(EN_L, 0);
  pwmBegin(EN_R, 1);
  stopMotors();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(SENSOR_PINS[i], INPUT);
  resetCal();

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), isrEcho, CHANGE);

#if USE_ENCODER
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);
#endif

  loadParams();
  bool haveCal = loadCal();

  Serial.println(F("San sang. 'c' = hieu chuan, 's' = chay, 'x' = dung, 'save' = luu thong so."));
  printParams();

#if AUTO_START
  // Bật nguồn là tự chạy: đặt xe lên line, chờ START_DELAY_MS rồi xuất phát
  Serial.printf("AUTO_START: chay sau %lu ms\n", (unsigned long)START_DELAY_MS);
  delay(START_DELAY_MS);
  if (!haveCal) {
    Serial.println(F("Chua co hieu chuan -> xe tu xoay de hieu chuan (dat xe giua line)."));
    runCalibration(true);
    delay(500);
  }
  resetPid(); usReset(); lastSide = 0; wasLost = false;
  state = RUNNING;
  Serial.println(F("RUN"));
#endif
}

void loop() {
  static uint32_t tNext = 0, tEnc = 0, tDbg = 0;
  handleSerial();

  if (state == CALIBRATING) {
    runCalibration(AUTO_CAL_SPIN);
    state = IDLE;
    return;
  }

  uint32_t now = micros();
  if ((int32_t)(now - tNext) < 0) return;
  tNext += LOOP_US;
  if ((int32_t)(now - tNext) > (int32_t)(10 * LOOP_US)) tNext = now + LOOP_US;

  uint32_t ms = millis();

#if USE_ENCODER
  if (ms - tEnc >= 50) {
    noInterrupts(); int32_t a = encL, b = encR; interrupts();
    speedL = a - lastEncL; speedR = b - lastEncR;
    lastEncL = a; lastEncR = b;
    tEnc = ms;
  }
#endif

  if (state == RUNNING) {
    updateObstacle(ms);
    controlStep();
  } else if (debugOn && calibrated) {
    bool dummy;
    readPosition(dummy);                           // cho phép xem cảm biến khi đang đứng yên
  }

  if (debugOn && ms - tDbg >= 100) {
    tDbg = ms;
    for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("%4u ", sv[i]);
#if USE_ENCODER
    Serial.printf("| err=%.0f dist=%.0f encL=%ld encR=%ld\n",
                  lastErr, distCm, (long)speedL, (long)speedR);
#else
    Serial.printf("| err=%.0f dist=%.0f\n", lastErr, distCm);
#endif
  }
}
