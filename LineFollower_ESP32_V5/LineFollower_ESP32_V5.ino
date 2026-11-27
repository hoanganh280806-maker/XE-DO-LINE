/* code đi thẳng và rẽ + Ghi log tốc độ Encoder & Gửi UDP sang Python (VS Code)
 * ===================================================================
 *  XE DÒ LINE - ESP32 DEV MODULE (v5 + UDP Telemetry Logger)
 *  QTR-8A (analog) + HC-SR04 + L298N + 2x Encoder GA25 (Pha A & B)
 *
 *  QUY TRÌNH GHI & XUẤT DỮ LIỆU:
 *   1. Cắm đủ 8 mắt QTR-8A, bật nguồn (hoặc gửi 's' / 'ramp') -> xe chạy & tự lưu tốc độ
 *      2 encoder (speedL, speedR) mỗi 50ms vào RAM (tối đa 4000 mẫu = 200 giây).
 *   2. Khi xe dừng (gửi 'x', tự dừng khi hết ramp/mất line), rút 3 dây ADC2 (25, 26, 27).
 *   3. Chạy script Python trên VS Code trước, sau đó gửi lệnh 'udp' qua Serial
 *      (hoặc nhấn giữ nút BOOT trên ESP32 khoảng 1 giây) để bắn toàn bộ log qua UDP.
 *
 *  LỆNH SERIAL (115200, Newline):
 *    c : hiệu chuẩn (4s)    s : chạy dò line    ramp : chạy tăng tốc dần tới max & ghi log
 *    x : dừng               p : in thông số
 *    kp kd ki base max      : thông số PD / tốc độ        (vd: kp 0.05)
 *    lost 140               : tốc độ xoay khi mất line
 *    tspd 170 / tadv 90     : tốc độ xoay khi cua / ms đi thẳng trước khi cua
 *    corner 0|1             : tắt/bật xử lý góc gắt
 *    tfull 1600             : thời gian (ms) xe xoay đủ MỘT VÒNG tại chỗ
 *    tcnt 0                 : số xung encoder cho một vòng (0 = dùng thời gian tfull)
 *    spin                   : xoay thử 'tfull' ms, in số xung encoder
 *    udp / log              : gửi dữ liệu log qua Wi-Fi UDP / xem số mẫu đã lưu
 *    test / raw / dbg       : thử động cơ / in ADC thô / bật tắt debug
 *    save / clear           : lưu thông số vào flash / xóa dữ liệu đã lưu
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======================= CẤU HÌNH WIFI / UDP ========================
const char*    WIFI_SSID = "iPhone";               // Wi-Fi 2.4GHz (hoặc Hotspot)
const char*    WIFI_PASS = "17072006";             // Mật khẩu Wi-Fi
const char*    PC_IP     = "172.20.10.3";          // IPv4 của máy tính chạy Python
const uint16_t PC_PORT   = 5005;                   // Cổng UDP nhận trên Python
const uint8_t  DUMP_BTN  = 0;                      // Nút BOOT trên ESP32 (nhấn giữ 1s khi xe dừng để gửi UDP)

// ============================ CẤU HÌNH ==============================
#define AUTO_START      true                       // true: bật nguồn là tự chạy (nếu đã hiệu chuẩn)
const uint32_t START_DELAY_MS = 3000;              // chờ bao lâu sau khi bật nguồn rồi mới chạy
#define BLACK_LINE      true                       // true: line đen nền trắng | false: line trắng nền đen
#define AUTO_CAL_SPIN   false                      // true: xe tự xoay tại chỗ khi hiệu chuẩn bằng lệnh 'c'
#define USE_ENCODER     1                          // 1: bật đọc encoder | 0: tắt hẳn ngắt encoder
#define CAL_TIME_MS     4000
#define NUM_SENSORS     8

// QTR-8A: OUT1..OUT8 (OUT1 = ngoài cùng TRÁI; nếu bị ngược hãy đảo thứ tự mảng này)
const uint8_t SENSOR_PINS[NUM_SENSORS] = {39, 33, 34, 25, 35, 26, 32, 27};

// L298N (Trái = OUT3/OUT4, Phải = OUT1/OUT2)
const uint8_t EN_L = 21, IN1_L = 23, IN2_L = 22;   // Động cơ TRÁI (ENB, IN3, IN4)
const uint8_t EN_R = 5,  IN1_R = 18, IN2_R = 19;   // Động cơ PHẢI (ENA, IN1, IN2)
const bool INVERT_LEFT  = false;                   // đổi thành true nếu bánh trái quay ngược
const bool INVERT_RIGHT = true;                    // đổi thành true nếu bánh phải quay ngược

// HC-SR04
const uint8_t TRIG_PIN = 12, ECHO_PIN = 13;
const float    OBSTACLE_CM   = 15.0f;
const uint8_t  OBSTACLE_HITS = 2;                  // số lần đo liên tiếp để xác nhận vật cản (2 x 60ms)
const uint32_t US_MIN_WIDTH  = 120;                // us; xung ngắn hơn (~2cm) coi là nhiễu

// Encoder GA25 (Pha A + Pha B)
const uint8_t ENC_L_A = 17, ENC_L_B = 16;          // Trái
const uint8_t ENC_R_A = 4,  ENC_R_B = 15;          // Phải

// PWM
const uint32_t PWM_FREQ = 5000;
const uint8_t  PWM_BITS = 8;                       // 0..255

// Vòng điều khiển & chu kỳ ghi log
const uint32_t LOOP_US    = 2000;                  // 500 Hz
const uint32_t SAMPLE_MS  = 50;                    // Chu kỳ lấy mẫu vận tốc & lưu log (50ms)

// Lọc đạo hàm: 1.0 = không lọc, càng nhỏ càng mượt (nhưng trễ hơn)
const float DERIV_ALPHA = 0.4f;

// Góc gắt / mất line / settle
const int      CORNER_TH      = 600;               // ngưỡng cảm biến ngoài cùng "thấy line đen"
const uint8_t  CORNER_CONFIRM = 2;                 // số vòng liên tiếp để xác nhận góc (chống nhiễu)
const int      TURN_FWD_SPEED = 110;               // tốc độ đi thẳng trong giai đoạn ADVANCE
const uint32_t TURN_MAX_MS    = 900;               // xoay cua quá lâu thì chuyển sang SEARCH
const int      CENTER_BAND    = 700;               // |err| < giá trị này coi như line đã về giữa
const uint32_t LOST_GRACE_MS  = 40;                // mất line dưới thời gian này thì chưa phản ứng
const uint32_t SETTLE_MS      = 300;               // thời gian tăng tốc dần sau khi cua/tìm lại line
const uint32_t CORNER_HOLD_MS = 250;               // sau khi đi thẳng qua ngã rẽ, bỏ qua phát hiện góc

// ======================= THAM SỐ CÓ THỂ CHỈNH ========================
float Kp = 0.050f, Ki = 0.0f, Kd = 1.0f;
int   baseSpeed = 170;
int   maxSpeed  = 230;
int   minSpeed  = -80;                             // cho phép đảo chiều nhẹ khi vào cua gắt
int   lostSpin  = 140;                             // tốc độ xoay khi mất line
bool  cornerMode = true;
int   turnSpeed  = 230;                            // tốc độ xoay tại chỗ khi cua 90 độ
int   turnAdvMs  = 40;                             // ms đi thẳng trước khi quyết định cua
int   fullTurnMs  = 1600;                          // ms để xe xoay đủ 360 độ tại chỗ (chỉnh bằng lệnh 'spin')
int   fullTurnCnt = 0;                             // xung encoder cho 360 độ (0 = dùng fullTurnMs)

// Tham số điều khiển tăng tốc dần đến max (cho chế độ 'ramp')
int      rampStepPwm    = 5;                       // Bước tăng/giảm PWM mỗi chu kỳ ramp
uint32_t rampIntervalMs = 50;                      // Chu kỳ tăng/giảm PWM (ms)
uint32_t holdMaxMs      = 1500;                    // Thời gian giữ tốc độ max trước khi giảm/dừng (ms)
bool     autoStopOnZero = true;                    // Tự dừng khi 2 encoder đã quay rồi dừng hẳn

// ============================ BIẾN TOÀN CỤC ===========================
uint16_t calMin[NUM_SENSORS], calMax[NUM_SENSORS];
uint16_t sv[NUM_SENSORS];                          // giá trị chuẩn hóa 0..1000
bool     calibrated = false;

enum State { IDLE, CALIBRATING, RUNNING, RAMPING };
State state = IDLE;

enum RunMode { RM_FOLLOW, RM_ADVANCE, RM_TURN, RM_SEARCH };
RunMode  rmode = RM_FOLLOW;
int      turnDir = 0;                              // -1 quay trái, +1 quay phải
uint32_t modeStartMs = 0, settleStartMs = 0, lostSinceMs = 0, cornerHoldUntil = 0;
uint8_t  cornerCnt = 0;
bool     wasLost = false;
int32_t  searchEncL0 = 0, searchEncR0 = 0;

enum RampPhase { RAMP_IDLE, RAMP_UP, RAMP_HOLD_MAX, RAMP_DOWN };
RampPhase rampPhase = RAMP_IDLE;
int       currentPwm   = 0;
uint32_t  lastRampMs   = 0;
uint32_t  maxReachedMs = 0;
bool      hadMotion    = false;

float    lastErr = 0, integral = 0, derivFilt = 0;
bool     havePrev = false;                         // tránh derivative kick
int      lastSide = 0;                             // -1: line lệch trái, +1: lệch phải (lần cuối thấy rõ)
bool     debugOn = false;
bool     obsPrinted = false;

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

// ---- Bộ đệm lưu Log vận tốc Encoder trong RAM (mỗi 50ms) ----
struct __attribute__((packed)) LogSample {
  uint32_t tMs;      // Thời gian kể từ lúc bắt đầu chạy (ms)
  int16_t  spdL;     // Vận tốc encoder trái (xung / 50ms)
  int16_t  spdR;     // Vận tốc encoder phải (xung / 50ms)
  int16_t  err;      // Sai số dò line (-3500..3500) hoặc PWM khi chạy ramp
  uint8_t  mode;     // Trạng thái chạy (0:FOLLOW, 1:ADVANCE, 2:TURN, 3:SEARCH)
};

const size_t MAX_LOG_SAMPLES = 4000;               // 4000 * 50ms = 200 giây chạy (~44KB RAM)
LogSample    logBuf[MAX_LOG_SAMPLES];
size_t       logCount   = 0;
uint32_t     runStartMs = 0;
WiFiUDP      udp;

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

// ========================= LOG & GỬI UDP =============================
void clearLog(uint32_t nowMs) {
  logCount   = 0;
  runStartMs = nowMs;
  hadMotion  = false;
}

void recordLogSample(uint32_t nowMs) {
  if (state != RUNNING && state != RAMPING) return;
  if (logCount >= MAX_LOG_SAMPLES) return;

  LogSample &s = logBuf[logCount++];
  s.tMs = nowMs - runStartMs;
#if USE_ENCODER
  s.spdL = (int16_t)constrain(speedL, -32768, 32767);
  s.spdR = (int16_t)constrain(speedR, -32768, 32767);
#else
  s.spdL = 0;
  s.spdR = 0;
#endif
  if (state == RAMPING) {
    s.err  = (int16_t)constrain(currentPwm, -32768, 32767);
    s.mode = 0;
  } else {
    s.err  = (int16_t)constrain((int)lastErr, -32768, 32767);
    s.mode = (uint8_t)rmode;
  }
}

void dumpLogOverUdp() {
  state = IDLE;
  rmode = RM_FOLLOW;
  rampPhase = RAMP_IDLE;
  currentPwm = 0;
  brakeMotors();

  if (logCount == 0) {
    Serial.println(F("Chua co du lieu log nao trong RAM! Hay cho xe chay truoc."));
    return;
  }

  Serial.printf("Chuan bi gui %u mau qua UDP toi %s:%u ...\n",
                (unsigned)logCount, PC_IP, PC_PORT);
  Serial.println(F("Luu y: Hay dam bao da rut 3 day ADC2 (GPIO 25, 26, 27) truoc khi bat Wi-Fi!"));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Ket noi Wi-Fi THAT BAI! Kiem tra lai WIFI_SSID / WIFI_PASS."));
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  Serial.print(F("Da ket noi Wi-Fi. IP xe: "));
  Serial.println(WiFi.localIP());

  udp.begin(PC_PORT);

  for (int k = 0; k < 3; k++) {
    udp.beginPacket(PC_IP, PC_PORT);
    udp.printf("START,%u\n", (unsigned)logCount);
    udp.endPacket();
    delay(20);
  }

  for (size_t i = 0; i < logCount; i++) {
    const LogSample &s = logBuf[i];
    udp.beginPacket(PC_IP, PC_PORT);
    udp.printf("DATA,%u,%lu,%d,%d,%d,%u\n",
               (unsigned)i,
               (unsigned long)s.tMs,
               (int)s.spdL,
               (int)s.spdR,
               (int)s.err,
               (unsigned)s.mode);
    udp.endPacket();
    delay(3);
  }

  for (int k = 0; k < 3; k++) {
    udp.beginPacket(PC_IP, PC_PORT);
    udp.printf("END,%u\n", (unsigned)logCount);
    udp.endPacket();
    delay(20);
  }

  udp.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(F("Da gui xong toan bo log qua UDP va tat Wi-Fi!"));
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
      if (w < US_MIN_WIDTH || w > 23200) usProcess(999.0f);
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
uint16_t readOne(uint8_t pin) {
  analogRead(pin);
  return (analogRead(pin) + analogRead(pin)) / 2;
}

void readRaw(uint16_t *raw) {
  for (int i = 0; i < NUM_SENSORS; i++) raw[i] = readOne(SENSOR_PINS[i]);
}

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
  prefs.putInt("tspd", turnSpeed); prefs.putInt("tadv", turnAdvMs); prefs.putBool("corner", cornerMode);
  prefs.putInt("tfull", fullTurnMs); prefs.putInt("tcnt", fullTurnCnt);
  prefs.end();
}

void loadParams() {
  prefs.begin("linebot", false);
  Kp = prefs.getFloat("kp", Kp); Ki = prefs.getFloat("ki", Ki); Kd = prefs.getFloat("kd", Kd);
  baseSpeed = prefs.getInt("base", baseSpeed);
  maxSpeed  = prefs.getInt("max", maxSpeed);
  lostSpin  = prefs.getInt("lost", lostSpin);
  turnSpeed = prefs.getInt("tspd", turnSpeed);
  turnAdvMs = prefs.getInt("tadv", turnAdvMs);
  cornerMode = prefs.getBool("corner", cornerMode);
  fullTurnMs  = prefs.getInt("tfull", fullTurnMs);
  fullTurnCnt = prefs.getInt("tcnt", fullTurnCnt);
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
  saveCal();
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
      if (!BLACK_LINE) v = 1000 - v;
    }
    sv[i] = v;
    if (v > maxV) maxV = v;
    if (v > 150) {
      wsum += (long)v * i * 1000;
      sum  += v;
    }
  }
  onLine = (maxV > 250 && sum > 0);
  if (!onLine) return 3500;
  int pos = (int)(wsum / sum);
  int e = pos - 3500;
  if (e < -400)      lastSide = -1;
  else if (e > 400)  lastSide = +1;
  return pos;
}

// ============================ ĐIỀU KHIỂN ==============================
void dbgEvent(const __FlashStringHelper *s) { if (debugOn) Serial.println(s); }

void resetPid() {
  integral = 0;
  derivFilt = 0;
  havePrev = false;
}

int32_t spinProgress() {
#if USE_ENCODER
  noInterrupts(); int32_t a = encL, b = encR; interrupts();
  return (abs(a - searchEncL0) + abs(b - searchEncR0)) / 2;
#else
  return 0;
#endif
}

bool searchFullTurnDone(uint32_t nowMs) {
#if USE_ENCODER
  if (fullTurnCnt > 0) return spinProgress() >= fullTurnCnt;
#endif
  return nowMs - modeStartMs >= (uint32_t)fullTurnMs;
}

void startSettle(uint32_t nowMs) {
  rmode = RM_FOLLOW;
  settleStartMs = nowMs;
  wasLost = false;
  cornerCnt = 0;
  resetPid();
}

void startSearch(uint32_t nowMs) {
  rmode = RM_SEARCH;
  modeStartMs = nowMs;
  turnDir = (lastSide >= 0) ? +1 : -1;
  wasLost = false;
#if USE_ENCODER
  noInterrupts(); searchEncL0 = encL; searchEncR0 = encR; interrupts();
#endif
  resetPid();
  dbgEvent(F("MAT LINE -> SEARCH"));
}

void startRun() {
  uint32_t nowMs = millis();
  resetPid();
  usReset();
  lastSide = 0;
  rmode = RM_FOLLOW;
  rampPhase = RAMP_IDLE;
  currentPwm = 0;
  wasLost = false;
  cornerCnt = 0;
  cornerHoldUntil = 0;
  settleStartMs = nowMs;
  obsPrinted = false;

#if USE_ENCODER
  noInterrupts();
  lastEncL = encL;
  lastEncR = encR;
  interrupts();
  speedL = 0;
  speedR = 0;
#endif
  clearLog(nowMs);
  state = RUNNING;
}

void startEncoderRampTest() {
  uint32_t nowMs = millis();
#if USE_ENCODER
  noInterrupts();
  lastEncL = encL;
  lastEncR = encR;
  interrupts();
  speedL = 0;
  speedR = 0;
#endif
  currentPwm   = 0;
  lastRampMs   = nowMs;
  rampPhase    = RAMP_UP;
  rmode        = RM_FOLLOW;
  clearLog(nowMs);
  setMotors(0, 0);
  state = RAMPING;
  Serial.printf("Bat dau tang toc den max=%d | Luu log moi %lu ms\n",
                maxSpeed, (unsigned long)SAMPLE_MS);
}

void stopMotionAndLog(const char* reason) {
  currentPwm = 0;
  rampPhase  = RAMP_IDLE;
  rmode      = RM_FOLLOW;
  state      = IDLE;
  brakeMotors();
  Serial.printf("DUNG (%s). Da luu %u mau. Rut 3 day ADC2 (25,26,27) roi gui 'udp'.\n",
                reason, (unsigned)logCount);
}

void updateRampStep(uint32_t nowMs) {
  if (state != RAMPING) return;

#if USE_ENCODER
  if (autoStopOnZero && hadMotion && speedL == 0 && speedR == 0 && rampPhase != RAMP_UP) {
    stopMotionAndLog("2 encoder da dung");
    return;
  }
#endif

  if (rampPhase == RAMP_UP) {
    if (nowMs - lastRampMs >= rampIntervalMs) {
      lastRampMs = nowMs;
      currentPwm += rampStepPwm;
      if (currentPwm >= maxSpeed) {
        currentPwm = maxSpeed;
        rampPhase = RAMP_HOLD_MAX;
        maxReachedMs = nowMs;
      }
      setMotors(currentPwm, currentPwm);
    }
  } else if (rampPhase == RAMP_HOLD_MAX) {
    setMotors(currentPwm, currentPwm);
    if (nowMs - maxReachedMs >= holdMaxMs) {
      rampPhase = RAMP_DOWN;
      lastRampMs = nowMs;
    }
  } else if (rampPhase == RAMP_DOWN) {
    if (nowMs - lastRampMs >= rampIntervalMs) {
      lastRampMs = nowMs;
      currentPwm -= rampStepPwm;
      if (currentPwm <= 0) {
        stopMotionAndLog("hoan tat chu trinh ramp");
        return;
      }
      setMotors(currentPwm, currentPwm);
    }
  }
}

void controlStep() {
  bool onLine;
  int pos = readPosition(onLine);
  uint32_t nowMs = millis();
  float err = (float)pos - 3500.0f;

  if (obstacle != obsPrinted) {
    obsPrinted = obstacle;
    if (obstacle) Serial.printf("VAT CAN: %.0f cm -> phanh\n", distCm);
    else          Serial.println(F("Het vat can"));
  }
  if (obstacle) {
    brakeMotors();
    resetPid();
    rmode = RM_FOLLOW;
    wasLost = false;
    return;
  }

  if (rmode == RM_ADVANCE) {
    setMotors(TURN_FWD_SPEED, TURN_FWD_SPEED);
    if (nowMs - modeStartMs >= (uint32_t)turnAdvMs) {
      bool frontOn = sv[3] > 400 || sv[4] > 400;
      if (frontOn) {
        cornerHoldUntil = nowMs + CORNER_HOLD_MS;
        startSettle(nowMs);
        dbgEvent(F("NGA RE/CUA THOAI -> di thang"));
      } else {
        rmode = RM_TURN;
        modeStartMs = nowMs;
        dbgEvent(F("GOC 90 -> TURN"));
      }
    }
    return;
  }

  if (rmode == RM_TURN) {
    uint32_t t = nowMs - modeStartMs;
    if (onLine && fabsf(err) < CENTER_BAND && t > 40) {
      startSettle(nowMs);
      dbgEvent(F("TURN xong"));
      return;
    }
    if (t > TURN_MAX_MS) { startSearch(nowMs); return; }
    int spd = (onLine && fabsf(err) < 1800.0f) ? (turnSpeed * 6 / 10) : turnSpeed;
    setMotors(turnDir * spd, -turnDir * spd);
    return;
  }

  if (rmode == RM_SEARCH) {
    if (onLine && fabsf(err) < CENTER_BAND) {
      startSettle(nowMs);
      dbgEvent(F("SEARCH: tim thay line"));
      return;
    }
    if (searchFullTurnDone(nowMs)) {
      stopMotionAndLog("KHONG TIM THAY LINE sau 1 vong");
      return;
    }
    int spd = onLine ? (lostSpin * 6 / 10) : lostSpin;
    setMotors(turnDir * spd, -turnDir * spd);
    return;
  }

  if (!onLine) {
    if (!wasLost) { wasLost = true; lostSinceMs = nowMs; }
    if (nowMs - lostSinceMs < LOST_GRACE_MS) return;
    startSearch(nowMs);
    return;
  }
  wasLost = false;

  if (cornerMode && nowMs >= cornerHoldUntil) {
    bool leftHit  = sv[0] > CORNER_TH && sv[1] > CORNER_TH;
    bool rightHit = sv[7] > CORNER_TH && sv[6] > CORNER_TH;
    bool leftAny  = sv[0] > CORNER_TH || sv[1] > CORNER_TH;
    bool rightAny = sv[7] > CORNER_TH || sv[6] > CORNER_TH;
    if ((leftHit && !rightAny) || (rightHit && !leftAny)) {
      if (++cornerCnt >= CORNER_CONFIRM) {
        cornerCnt = 0;
        turnDir = leftHit ? -1 : +1;
        lastSide = turnDir;
        rmode = RM_ADVANCE;
        modeStartMs = nowMs;
        resetPid();
        setMotors(TURN_FWD_SPEED, TURN_FWD_SPEED);
        return;
      }
    } else {
      cornerCnt = 0;
    }
  }

  if (!havePrev) { lastErr = err; havePrev = true; }
  integral = constrain(integral + err, -20000.0f, 20000.0f);
  float deriv = err - lastErr;
  lastErr = err;
  derivFilt += DERIV_ALPHA * (deriv - derivFilt);

  float corr = Kp * err + Ki * integral + Kd * derivFilt;

  float slow = 1.0f - 0.50f * fabsf(err) / 3500.0f;
  float ramp = 1.0f;
  uint32_t sinceSettle = nowMs - settleStartMs;
  if (sinceSettle < SETTLE_MS) ramp = 0.6f + 0.4f * (float)sinceSettle / (float)SETTLE_MS;
  int base = (int)(baseSpeed * ramp * slow);

  int l = constrain(base + (int)corr, minSpeed, maxSpeed);
  int r = constrain(base - (int)corr, minSpeed, maxSpeed);
  setMotors(l, r);
}

// ============================== SERIAL ================================
void printParams() {
  Serial.printf("Kp=%.4f Ki=%.5f Kd=%.3f base=%d max=%d lost=%d | corner=%d tspd=%d tadv=%d | tfull=%d tcnt=%d\n",
                Kp, Ki, Kd, baseSpeed, maxSpeed, lostSpin, (int)cornerMode, turnSpeed, turnAdvMs,
                fullTurnMs, fullTurnCnt);
}

void processCommand(char *buf) {
  for (char *p = buf; *p; p++) *p = tolower((unsigned char)*p);

  if      (!strcmp(buf, "c"))   { state = CALIBRATING; return; }
  else if (!strcmp(buf, "s"))   {
    if (!calibrated) Serial.println(F("Chua hieu chuan! Gui 'c' truoc (hoac gui 'ramp' de test tang toc encoder)."));
    else { startRun(); Serial.println(F("RUN")); }
    return;
  }
  else if (!strcmp(buf, "ramp")) {
    startEncoderRampTest();
    return;
  }
  else if (!strcmp(buf, "x"))   {
    stopMotionAndLog("lenh dung tu Serial");
    return;
  }
  else if (!strcmp(buf, "udp")) { dumpLogOverUdp(); return; }
  else if (!strcmp(buf, "log")) {
    Serial.printf("So mau log hien tai: %u / %u\n", (unsigned)logCount, (unsigned)MAX_LOG_SAMPLES);
    return;
  }
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
  else if (!strcmp(buf, "test")) {
    state = IDLE;
    Serial.println(F("Test: banh TRAI quay toi..."));
    setMotors(150, 0); delay(1000);
    Serial.println(F("Test: banh PHAI quay toi..."));
    setMotors(0, 150); delay(1000);
    stopMotors();
    Serial.println(F("Xong test."));
    return;
  }
  else if (!strcmp(buf, "spin")) {
    state = IDLE;
    Serial.printf("Spin %d ms o toc do lost=%d ...\n", fullTurnMs, lostSpin);
#if USE_ENCODER
    noInterrupts(); int32_t a0 = encL, b0 = encR; interrupts();
#endif
    setMotors(lostSpin, -lostSpin);
    delay(fullTurnMs);
    brakeMotors();
#if USE_ENCODER
    noInterrupts(); int32_t a1 = encL, b1 = encR; interrupts();
    Serial.printf("Xung encoder: trai=%ld phai=%ld (trung binh %ld) -> neu dung 1 vong: 'tcnt %ld'\n",
                  (long)(a1 - a0), (long)(b1 - b0),
                  (long)((abs(a1 - a0) + abs(b1 - b0)) / 2), (long)((abs(a1 - a0) + abs(b1 - b0)) / 2));
#endif
    Serial.println(F("Xe xoay thieu -> tang tfull, xoay thua -> giam tfull."));
    return;
  }
  else if (!strcmp(buf, "raw")) {
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
    else if (!strcmp(key, "tspd")) turnSpeed = constrain((int)v, 0, 255);
    else if (!strcmp(key, "tadv")) turnAdvMs = constrain((int)v, 0, 500);
    else if (!strcmp(key, "corner")) cornerMode = (v != 0);
    else if (!strcmp(key, "tfull")) fullTurnMs  = constrain((int)v, 200, 10000);
    else if (!strcmp(key, "tcnt"))  fullTurnCnt = max(0, (int)v);
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
      if (overflow) {
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

  WiFi.mode(WIFI_OFF);                             // Tắt hẳn Wi-Fi khi chạy để không xung đột ADC2

  pinMode(DUMP_BTN, INPUT_PULLUP);
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

  Serial.println(F("San sang. 'c' = hieu chuan, 's' = chay line, 'ramp' = tang toc encoder, 'x' = dung, 'udp' = gui log."));
  printParams();

#if AUTO_START
  if (!haveCal) {
    Serial.println(F("Chua co du lieu hieu chuan trong flash -> KHONG tu chay. Gui 'c' de hieu chuan."));
  } else {
    Serial.println(F("Da nap hieu chuan tu flash."));
    Serial.printf("AUTO_START: chay sau %lu ms\n", (unsigned long)START_DELAY_MS);
    delay(START_DELAY_MS);
    startRun();
    Serial.println(F("RUN"));
  }
#else
  if (haveCal) Serial.println(F("Da nap hieu chuan tu flash."));
  else         Serial.println(F("Chua co du lieu hieu chuan trong flash. Gui 'c' de hieu chuan."));
#endif
}

void loop() {
  static uint32_t tNext = 0, tEnc = 0, tDbg = 0;
  static uint32_t btnHoldMs = 0;
  handleSerial();

  if (state == IDLE) {
    if (digitalRead(DUMP_BTN) == LOW) {
      if (btnHoldMs == 0) btnHoldMs = millis();
      else if (millis() - btnHoldMs >= 1000) {
        btnHoldMs = 0;
        dumpLogOverUdp();
        return;
      }
    } else {
      btnHoldMs = 0;
    }
  } else {
    btnHoldMs = 0;
  }

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
  if (ms - tEnc >= SAMPLE_MS) {
    noInterrupts(); int32_t a = encL, b = encR; interrupts();
    speedL = a - lastEncL;
    speedR = b - lastEncR;
    lastEncL = a;
    lastEncR = b;
    tEnc = ms;

    if (labs(speedL) > 0 || labs(speedR) > 0) {
      hadMotion = true;
    }

    recordLogSample(ms);
  }
#endif

  if (state == RUNNING) {
    updateObstacle(ms);
    controlStep();
  } else if (state == RAMPING) {
    updateRampStep(ms);
  } else if (debugOn && calibrated) {
    bool dummy;
    readPosition(dummy);
  }

  if (debugOn && ms - tDbg >= 100) {
    tDbg = ms;
    for (int i = 0; i < NUM_SENSORS; i++) Serial.printf("%4u ", sv[i]);
#if USE_ENCODER
    Serial.printf("| m=%d err=%.0f dist=%.0f encL=%ld encR=%ld\n",
                  (int)rmode, lastErr, distCm, (long)speedL, (long)speedR);
#else
    Serial.printf("| m=%d err=%.0f dist=%.0f\n", (int)rmode, lastErr, distCm);
#endif
  }
}