//Đây là file code của nhóm Tú Anh , vừa chạy chiều thứ 7 , khá ổn rồi 
/*
=====================================================================
  ROBOT DÒ LINE TỐC ĐỘ CAO  -  ESP32-S3 + L298N + mảng 8 cảm biến
=====================================================================
  Port từ code gốc chạy trên Arduino Nano (ATmega328P).
  Bản này đã chỉnh cho XE NẶNG / MA SÁT LỚN. Mọi chỗ sửa có đánh dấu [SỬA]/[MỚI].

  NỐI DÂY (tóm tắt)
  -----------------------------------------------------------------
  Cảm biến : S1..S8 -> GPIO 17,2,4,5,6,7,8,16   (S1 = ngoài cùng BÊN TRÁI)
             VCC cảm biến -> 3V3 (KHÔNG cấp 5V nếu ngõ ra analog)
             G -> GND
  L298N    : ENA -> GPIO10   IN1 -> GPIO11   IN2 -> GPIO12  (PHẢI: OUT1/OUT2)
             ENB -> GPIO13   IN3 -> GPIO14   IN4 -> GPIO15  (TRÁI: OUT3/OUT4)
             (tháo jumper ENA, ENB trên L298N)

  Nút      : nút BOOT (GPIO0) có sẵn trên board ESP32-S3
  Nguồn    : Pin -> L298N (12V, GND) ; 5V L298N -> chân 5V ESP32 ;
             GND của pin, L298N, ESP32, cảm biến phải NỐI CHUNG

  CÁCH DÙNG (không cần nút ngoài)
  -----------------------------------------------------------------
  Nút BOOT : nhấn giữ > 1 giây  -> vào chế độ HỌC MÀU (đưa cảm biến qua lại
                                    trên vạch và nền, rồi nhấn BOOT lần nữa để lưu)
             nhấn ngắn          -> chạy (tốc độ nut1)
             nhấn khi đang chạy -> DỪNG
  Serial 115200 (từ máy tính):
             1/2/3 = chạy tốc độ nut1/nut2/nut3     c = học màu (s = lưu)
             t = xem giá trị cảm biến               m = thử động cơ
             p = DÒ PWM TỐI THIỂU để xe lăn         h = trợ giúp
=====================================================================
*/

#include <Arduino.h>
#include <Preferences.h>

//=====================================================================
//  1. KHAI BÁO CHÂN
//=====================================================================
// 8 cảm biến, dùng chân ADC1 (GPIO1-10) để đọc analog ổn định.
// Bỏ GPIO3 vì là chân strapping. Nếu xe chạy ngược trái/phải, đảo thứ tự mảng này.
const uint8_t SENSOR_PINS[8] = {17, 2, 4, 5, 6, 7, 8, 16};

// L298N - động cơ PHẢI (OUT1/OUT2)
#define R_EN    10   // ENA: PWM tốc độ
#define R_IN1   11
#define R_IN2   12
// L298N - động cơ TRÁI (OUT3/OUT4)
#define L_EN    13   // ENB: PWM tốc độ
#define L_IN3   14
#define L_IN4   15

#define BUZZER_PIN  -1   // còi chủ động. Đặt -1 nếu không có còi
#define BUTTON1      0   // nút BOOT trên board (nhấn = LOW)
#define BUTTON2     -1   // nếu mua thêm nút: nối GPIO bất kỳ -> GND rồi điền số chân
#define BUTTON3     -1

//=====================================================================
//  2. CẤU HÌNH (chỉnh ở đây)
//=====================================================================
#define LINE_READS_HIGH   0     // 1: mắt nằm trên VẠCH thì số ADC CAO (như code gốc "line trắng")
                                // 0: mắt nằm trên vạch thì số ADC THẤP (vạch đen trên nền sáng)
#define ADC_BITS          12
#define ADC_MAX           4095
#define ADC_SAMPLES       1     // số lần đọc trung bình mỗi mắt (tăng lên 2-4 nếu nhiễu)
#define MIN_CONTRAST      300   // chênh lệch nền-vạch tối thiểu (trên thang 0-4095), nhỏ hơn thì coi là học màu lỗi

#define PWM_FREQ          5000  // Hz, PWM vào chân ENA/ENB của L298N
#define PWM_BITS          8     // 0-255, giữ nguyên thang tốc độ như code gốc

#define PID_DIV           27    // hệ số chia đầu ra PD (gốc: 27 cho bánh 68 mm)

// ---------- [SỬA] các thông số cho xe nặng ----------
// Dùng lệnh 'p' để đo PWM tối thiểu làm xe lăn, rồi đặt SPEED_RUN_MIN >= mức đó + ~30.
#define SPEED_RUN_MIN     160   // [SỬA] gốc 130. Tốc độ cơ sở THẤP NHẤT khi đang chạy (đoạn cua gắt)
#define SPEED_SEARCH      200   // [SỬA] gốc 160. PWM bánh quay tiến khi mất line
#define SEARCH_BACK       130   // [SỬA] PWM bánh còn lại quay LÙI -> xe xoay tại chỗ.
                                //        Xe xoay quá gắt/văng quá line thì giảm xuống 80, hoặc 0.
#define RAMP_START_PWM    110   // [SỬA] Các đoạn tăng tốc mềm sau cua/chuyển làn bắt đầu từ mức này.
#define KICK_PWM          100   // [SỬA] "Đạp ga" khi bánh vừa chuyển từ đứng yên sang quay
#define KICK_MS           0     //        thời gian đạp ga (ms). Nếu ESP32 bị reset khi xuất phát thì giảm.
#define SPEED_DROP_AT     1200  // [SỬA] gốc 1800. Độ lệch line (0..3500) mà tại đó xe giảm hẳn về SPEED_RUN_MIN.
                                //        Nhỏ = giảm tốc sớm hơn (an toàn hơn).
#define ACCEL_STEP        1     // [SỬA] Tăng tốc: cộng ACCEL_STEP mỗi 2 ms. Tăng lên 2 nếu muốn bốc hơn.
#define LOST_GAP_MS       300   // [SỬA] Mất line khi đang ở giữa (đứt nét, bụi): chạy tiếp chừng này ms

#define GAP_STEER_PCT     100   // [MỚI] % lực lái trung bình giữ lại khi vượt khoảng đứt (0 = đi thẳng)
#define GAP_SPEED_PCT     85    // [SỬA] % của baseSpeed khi vượt khoảng đứt (gốc 100)
#define STEER_AVG_N       16    // [MỚI] độ dài bộ lọc trung bình lực lái (mỗi lần tính PD là 4 ms)
#define LOST_GIVEUP_MS    2500  // [MỚI] tổng thời gian mất line tối đa trước khi dừng
#define GAP_DEBUG         1     // [MỚI] 1: in thời gian mỗi khoảng đứt ra Serial

// ---------- [MỚI] cua góc vuông: chạy thẳng -> DỪNG -> xoay tại chỗ -> dừng -> đi tiếp ----------
#define TURN_FWD_MS       40    // chạy thẳng chừng này ms rồi mới dừng (mắt nằm trước trục bánh)
                                // Xe cua sớm/cắt góc: tăng. Xe đi quá góc: giảm về 0
#define TURN_STOP_MS      120   // đứng yên chừng này ms trước khi xoay (cho xe hết trớn)
#define TURN_OUT_PWM      190   // PWM bánh ngoài (tiến) khi xoay
#define TURN_BACK_PWM     190   // PWM bánh trong (lùi) khi xoay. Bằng nhau = xoay đúng tại chỗ
#define TURN_LOSE_MS      250   // giai đoạn 1: xoay tối đa chừng này ms để mắt giữa rời khỏi line cũ
#define TURN_MIN_MS       450   // TỔNG thời gian xoay tối thiểu. *** CHỈNH CÁI NÀY cho xe quay đủ 90 độ ***
                                // Xe quay thiếu: tăng (vd 550, 650). Xe quay lố: giảm
#define TURN_TIMEOUT_MS   1500  // xoay tối đa, quá thì thoát ra để chuyển sang tìm line
#define TURN_SETTLE_MS    100   // đứng yên sau khi xoay xong rồi mới bám line tiếp

#define START_DELAY_MS    1000  // chờ sau khi nhấn nút để buông tay khỏi xe
#define LONG_PRESS_MS     1000  // nhấn giữ bao lâu thì tính là "nhấn giữ"
#define MAX_RUN_MS        30000UL  // AN TOÀN: tự dừng sau chừng này ms. Đặt 0 để tắt giới hạn
#define MANEUVER_TIMEOUT_MS 8000UL // AN TOÀN: cua vuông/chuyển làn kéo dài quá mức này thì dừng

#define ENABLE_LEFT_LANE  0     // code gốc đã tắt chuyển làn trái (left_lance bị comment)

#define AUTO_START        0     // 1: không cần bấm gì, bật nguồn -> học màu -> đếm ngược -> chạy
#define AUTO_CALIB_MS     5000  // thời gian quét học màu khi AUTO_START = 1
#define AUTO_SPEED        180   // tốc độ khi AUTO_START = 1

//=====================================================================
//  3. BIẾN TOÀN CỤC
//=====================================================================
Preferences prefs;

volatile int      lastPos   = 0;
volatile int      servoPwm  = 0;
volatile int      steerAvg  = 0;     // lực lái trung bình khi còn thấy line
volatile int      posPID    = 0;
volatile uint8_t  sensor    = 0;      // bit7 = S1 (trái nhất) ... bit0 = S8
volatile uint8_t  isCalib   = 0;
volatile bool     calibOK   = false;
volatile bool     estop     = false;  // yêu cầu dừng khẩn cấp
volatile bool     running   = false;
volatile int      PIDfre    = 0;
uint32_t          runStartMs = 0;

uint16_t sensorValue[8];   // giá trị ADC (đã quy ước: vạch = số cao)
uint16_t sensorPID[8];     // 0..1000
uint16_t v_min[8];         // nhỏ nhất (nền)
uint16_t v_max[8];         // lớn nhất (vạch)
uint16_t v_th[8];          // ngưỡng = (min+max)/2

int speed_run_forward = 0;
int baseSpeed = 0;         // tốc độ cơ sở hiện tại (có giới hạn gia tốc), xem smoothSpeed()
volatile int isHighSpeed  = 0;
volatile int isLowSpeed   = 0;
volatile int HaveToBrake  = 0;
int timeBrakeCurve = 20;
int speedBrake90   = 140;   // [SỬA] gốc 100 (xe nặng không lăn nổi ở PWM 100)
int timeToHighSpeed = 150;
int vitri = 0;
volatile int isInturn90 = 0;
int timeBrake90 = 30;
volatile int isLeft  = 0;
volatile int isRight = 0;
int speed90    = 150;       // [SỬA] gốc 90  (cua vuông: xe nặng đứng ì ở PWM 90)
int speedChLan = 150;       // [SỬA] gốc 100 (chuyển làn)

volatile int cnt  = 0;   // đồng hồ đa năng, +1 mỗi 2 ms
volatile int cnt3 = 0;
volatile int cnt4 = 0;

int pattern = 9;
float kp = 1.0;          // hệ số P
int   kd = 6;            // hệ số D

int nut1 = 200;          // [SỬA] gốc 180
int nut2 = 225;          // [SỬA] gốc 200
int nut3 = 245;          // [SỬA] gốc 220

enum { CMD_NONE = 0, CMD_RUN1, CMD_RUN2, CMD_RUN3, CMD_CALIB, CMD_SENSOR, CMD_MOTOR, CMD_PWMTEST };

//=====================================================================
//  4. KHAI BÁO TRƯỚC HÀM
//=====================================================================
void controlTask(void *pv);
void controlTick();
void read_sensor();
void speed_run(int speedDC_left, int speedDC_right);
void handleAndSpeed(int angle, int speed1);
void runforwardline(int tocdo);
void straightSpeed(int tocdo);
void curveSpeed();
int  smoothSpeed(int tocdo);
void searchLeft();
void searchRight();
unsigned char sensorMask(unsigned char mask);
char check_crossline();
char check_crosshalfline();
int gocturn90(int tocdo);
int left_lance(int tocdo);
int right_lance(int tocdo);
void setLed(int color);
void beep(int ms);
void printHelp();
void updateLine();
bool calibValid();
void calibrate(uint32_t durationMs);
void saveCalib();
bool loadCalib();
void startRun(int spd);
void idleTask();
int  readCommand();
void sensorTest();
void motorTest();
void pwmTest();

//=====================================================================
//  5. SETUP
//=====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // --- L298N ---
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN3, OUTPUT);
  pinMode(L_IN4, OUTPUT);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  digitalWrite(L_IN3, LOW);
  digitalWrite(L_IN4, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(R_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(L_EN, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(R_EN, 0);
  ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcAttachPin(L_EN, 1);
#endif
  speed_run(0, 0);

  // --- nút, còi ---
  if (BUTTON1 >= 0) pinMode(BUTTON1, INPUT_PULLUP);
  if (BUTTON2 >= 0) pinMode(BUTTON2, INPUT_PULLUP);
  if (BUTTON3 >= 0) pinMode(BUTTON3, INPUT_PULLUP);
  if (BUZZER_PIN >= 0) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // --- ADC ---
  analogReadResolution(ADC_BITS);
  analogSetAttenuation(ADC_11db);   // đo được 0 ~ 3.1 V
  for (int i = 0; i < 8; i++) pinMode(SENSOR_PINS[i], INPUT);

  // --- nạp dữ liệu học màu đã lưu ---
  calibOK = loadCalib();

  // --- tác vụ đọc cảm biến + PD, thay cho ngắt Timer2 của code gốc ---
  // Chạy cùng core với loop() và ưu tiên cao hơn: hoạt động giống ngắt (ISR),
  // nên các biến dùng chung không bị tranh chấp giữa 2 core.
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, NULL, 3, NULL, xPortGetCoreID());

  beep(50);
  Serial.println();
  Serial.println("=== ROBOT DO LINE ESP32-S3 + L298N ===");
  Serial.println(calibOK ? "Da co du lieu hoc mau." : "CHUA hoc mau! Giu nut BOOT > 1 giay (hoac gui 'c').");
  printHelp();

#if AUTO_START
  Serial.println("AUTO_START: quet cam bien qua lai tren vach va nen...");
  calibrate(AUTO_CALIB_MS);
  if (calibOK) {
    for (int s = 0; s < 3; s++) { beep(100); delay(900); }   // đặt xe lên line
    startRun(AUTO_SPEED);
  }
#endif
}

void printHelp() {
  Serial.println("Nut BOOT: nhan ngan = chay | giu >1s = hoc mau | nhan luc dang chay = dung");
  Serial.println("Serial: 1/2/3 = chay | c = hoc mau (s = luu) | t = xem cam bien | m = thu dong co | p = do PWM toi thieu");
}

//=====================================================================
//  6. TÁC VỤ ĐIỀU KHIỂN (thay cho ISR Timer2, chu kỳ 2 ms)
//=====================================================================
void controlTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    controlTick();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
  }
}

void controlTick() {
  static uint8_t stopCnt = 0;

  read_sensor();
  cnt++;

  // --- an toàn: nhấn BOOT hoặc hết giờ thì dừng ---
  if (running) {
    uint32_t now = millis();
    if (MAX_RUN_MS > 0 && (now - runStartMs) > MAX_RUN_MS) estop = true;
    if ((now - runStartMs) > 500 && BUTTON1 >= 0 && digitalRead(BUTTON1) == LOW) {
      if (++stopCnt >= 10) estop = true;   // giữ ~20 ms để lọc nhiễu
    } else {
      stopCnt = 0;
    }
  } else {
    stopCnt = 0;
  }

  // --- logic tăng/giảm tốc (giữ nguyên từ ISR gốc) ---
  if (isHighSpeed == 1) {
    if (cnt3 < timeToHighSpeed)
      cnt3++;
    else {
      cnt3++;
      HaveToBrake = 1;
      cnt4 = 0;
    }
  } else if (isLowSpeed == 1) {
    if (HaveToBrake == 1) cnt4++;
    if (cnt4 >= timeBrakeCurve) {
      HaveToBrake = 0;
      cnt3 = 0;
    } else {
      cnt3 = 0;
    }
  } else if ((isHighSpeed == 0) && (isLowSpeed == 0)) {
    if (HaveToBrake == 1) cnt4++;
    if (cnt4 >= timeBrakeCurve) {
      HaveToBrake = 0;
      cnt3 = 0;
    }
  }
}

//=====================================================================
//  7. LOOP: MÁY TRẠNG THÁI
//=====================================================================
void loop() {
  if (running && estop) pattern = 100;

  switch (pattern) {
    case 9:   // chờ lệnh
      idleTask();
      break;

    case 10:  // khởi động: smoothSpeed() đã tự tăng tốc mềm
      pattern = 11;
      break;

    case 11:  // chạy bám line bình thường
      if (check_crossline()) {            // vạch ngang toàn phần
        pattern = 21;
        break;
      }
      runforwardline(speed_run_forward);
      break;

    case 12:  // nửa vạch TRÁI
      if (check_crossline()) {
        pattern = 21;
        break;
      }
#if ENABLE_LEFT_LANE
      pattern = left_lance(speedChLan);
#else
      pattern = 11;
#endif
      break;

    case 13:  // nửa vạch PHẢI -> chuyển làn phải
      if (check_crossline()) {
        pattern = 21;
        break;
      }
      pattern = right_lance(speedChLan);
      break;

    case 21:  // vạch ngang toàn phần -> cua vuông
      pattern = gocturn90(speed90);
      break;

    case 100: // dừng, quay lại chờ lệnh
      speed_run(0, 0);
      running = false;
      isInturn90 = 0;
      isLeft = 0;
      isRight = 0;
      while (BUTTON1 >= 0 && digitalRead(BUTTON1) == LOW) delay(10);   // đợi nhả nút
      estop = false;
      speed_run(0, 0);
      beep(200);
      Serial.println("DA DUNG. Nhan BOOT hoac gui 1/2/3 de chay lai.");
      pattern = 9;
      break;

    default:
      pattern = 11;
      break;
  }
}

//=====================================================================
//  8. CHẠY BÁM LINE
//=====================================================================
// Tốc độ cơ sở theo ĐỘ LỆCH LINE:
//   - Line ở giữa (posPID ~ 0)            -> chạy tốc độ đích (tocdo)
//   - Lệch >= SPEED_DROP_AT               -> hạ về SPEED_RUN_MIN
//   - Giữa hai mức: giảm tuyến tính
//   - Tăng tốc từng chút (ACCEL_STEP mỗi 2 ms), GIẢM TỐC NGAY.
int smoothSpeed(int tocdo) {
  static uint32_t tAcc = 0;
  int e = abs(posPID);                      // 0..3500, 0 = line ngay giữa mảng cảm biến
  if (e > SPEED_DROP_AT) e = SPEED_DROP_AT;
  int target = tocdo - (int)((long)(tocdo - SPEED_RUN_MIN) * e / SPEED_DROP_AT);
  if (target > tocdo) target = tocdo;       // phòng khi tocdo < SPEED_RUN_MIN

  uint32_t now = millis();
  if (now - tAcc >= 2) {                    // mỗi 2 ms
    tAcc = now;
    if (baseSpeed < target) baseSpeed += ACCEL_STEP;
  }
  if (baseSpeed > target) baseSpeed = target;   // giảm tốc tức thì
  return baseSpeed;
}

// Xoay tại chỗ tìm line: một bánh tiến, một bánh lùi.
void searchLeft() {     // line mất ở bên TRÁI -> xoay sang trái
  baseSpeed = SPEED_RUN_MIN;
  speed_run(-SEARCH_BACK, SPEED_SEARCH);
}

void searchRight() {    // line mất ở bên PHẢI -> xoay sang phải
  baseSpeed = SPEED_RUN_MIN;
  speed_run(SPEED_SEARCH, -SEARCH_BACK);
}

// Đoạn thẳng
void straightSpeed(int tocdo) {
  handleAndSpeed(servoPwm, smoothSpeed(tocdo));
  isLowSpeed = 0;
  isHighSpeed = 1;
}

// Đoạn cua (tốc độ do smoothSpeed quyết định theo độ lệch)
void curveSpeed() {
  handleAndSpeed(servoPwm, smoothSpeed(speed_run_forward));
  isLowSpeed = 1;
  isHighSpeed = 0;
}

void runforwardline(int tocdo) {
  static uint32_t lostSince = 0;
  if (sensor != 0) {
#if GAP_DEBUG
    if (lostSince != 0) Serial.printf("GAP %lu ms\n", (unsigned long)(millis() - lostSince));
#endif
    lostSince = 0;
  }

  // [MỚI] Cua góc vuông: chạy thẳng -> dừng -> xoay tại chỗ -> dừng -> bám line tiếp
  static int8_t   cornerDir = 0;      // +1 phải, -1 trái, 0 không cua
  static uint8_t  cornerSt  = 0;      // 1 chạy thẳng, 2 dừng, 3 xoay rời line, 4 xoay tìm line, 5 dừng chốt
  static uint32_t cornerT   = 0;
  if (cornerDir != 0) {
    uint32_t el = millis() - cornerT;
    baseSpeed = SPEED_RUN_MIN;
    int rotL = (cornerDir > 0) ?  TURN_OUT_PWM : -TURN_BACK_PWM;
    int rotR = (cornerDir > 0) ? -TURN_BACK_PWM :  TURN_OUT_PWM;
    switch (cornerSt) {
      case 1:   // chạy thẳng một đoạn ngắn cho trục bánh tới góc
        if (el < TURN_FWD_MS) {
          speed_run(SPEED_RUN_MIN, SPEED_RUN_MIN);
        } else {
          speed_run(0, 0);
          cornerSt = 2;
          cornerT = millis();
        }
        return;

      case 2:   // đứng yên cho xe hết trớn
        speed_run(0, 0);
        if (el >= TURN_STOP_MS) {
          cornerSt = 3;
          cornerT = millis();   // mốc thời gian bắt đầu xoay
        }
        return;

      case 3:   // xoay cho tới khi 2 mắt giữa rời khỏi line cũ
        speed_run(rotL, rotR);
        if (((sensor & 0x18) == 0) || el > TURN_LOSE_MS) cornerSt = 4;
        return;

      case 4:   // tiếp tục xoay cho tới khi 2 mắt giữa gặp line mới (và đã xoay đủ thời gian tối thiểu)
        speed_run(rotL, rotR);
        if ((el > TURN_MIN_MS && (sensor & 0x18) && __builtin_popcount(sensor) <= 4) ||
            el > TURN_TIMEOUT_MS) {
          speed_run(0, 0);
          cornerSt = 5;
          cornerT = millis();
        }
        return;

      case 5:   // dừng chốt rồi bám line tiếp
        speed_run(0, 0);
        if (el < TURN_SETTLE_MS) return;
        cornerDir = 0;
        cornerSt = 0;
        vitri = 0;
        lastPos = posPID;       // tránh xung D lớn khi thoát cua
        baseSpeed = SPEED_RUN_MIN;
        break;                  // rơi xuống switch (sensor) bên dưới

      default:
        cornerDir = 0;
        cornerSt = 0;
        break;
    }
  }

  switch (sensor) {
    case 0b00011000:
    case 0b00111100:
      straightSpeed(tocdo);
      vitri = 0;
      break;

    case 0b00011100:
    case 0b00001000:
      straightSpeed(tocdo);
      vitri = 1;
      break;

    case 0b00001100:
    case 0b00011110:
      if (vitri < -7) {
        searchLeft();
        break;
      }
      curveSpeed();
      vitri = 2;
      break;

    case 0b00000100:
    case 0b00001110:
      if (vitri < -7) {
        searchLeft();
        break;
      }
      curveSpeed();
      vitri = 3;
      break;

    case 0b00000110:
    case 0b00001111:
    case 0b00000010:
    case 0b00000111:
    case 0b00000011:
    case 0b00000001:
      if (vitri < -7) {
        searchLeft();
        break;
      }
      curveSpeed();
      vitri = 7;
      break;

    case 0b00000000: {   // mất line
      if (lostSince == 0) lostSince = millis();
      uint32_t lost = millis() - lostSince;
      bool atEdge = (vitri <= -3) || (vitri >= 3);   // [SỬA] gốc: ±7

      if (!atEdge && lost < LOST_GAP_MS) {
        // Pha 1: khoảng đứt -> vượt "mù", lái theo lực lái trung bình
        handleAndSpeed(steerAvg * GAP_STEER_PCT / 100, baseSpeed * GAP_SPEED_PCT / 100);
      } else if (lost < LOST_GIVEUP_MS) {
        // Pha 2: tìm line, xoay về phía line biến mất
        bool left = atEdge ? (vitri < 0) : (lastPos < 0);
        if (left) { searchLeft();  vitri = -8; }
        else      { searchRight(); vitri = 8;  }
      } else {
        speed_run(0, 0);   // Pha 3: dừng
      }
      break;
    }

    case 0b00010000:
    case 0b00111000:
      if (vitri >= 7) {
        searchRight();
        break;
      }
      straightSpeed(tocdo);
      vitri = -1;
      break;

    case 0b00110000:
    case 0b01111000:
      if (vitri >= 7) {
        searchRight();
        break;
      }
      curveSpeed();
      vitri = -2;
      break;

    case 0b00100000:
    case 0b01110000:
      if (vitri >= 7) {
        searchRight();
        break;
      }
      curveSpeed();
      vitri = -3;
      break;

    case 0b01100000:
    case 0b11110000:
    case 0b01000000:
    case 0b11100000:
    case 0b11000000:
    case 0b10000000:
      if (vitri >= 7) {
        searchRight();
        break;
      }
      curveSpeed();
      vitri = -7;
      break;

    // ===== góc vuông: [SỬA] chuyển sang chế độ xoay tại chỗ =====
    case 0b00011111:   // góc vuông phải
    case 0b00111111:
    case 0b01111111:
      cornerDir = 1;
      cornerSt = 1;
      cornerT = millis();
      baseSpeed = SPEED_RUN_MIN;
      break;

    case 0b11111000:   // góc vuông trái
    case 0b11111100:
    case 0b11111110:
      cornerDir = -1;
      cornerSt = 1;
      cornerT = millis();
      baseSpeed = SPEED_RUN_MIN;
      break;

    default:           // mẫu lạ (vd 10011000, 11011000: nhánh xiên của góc nhọn lọt vào mắt ngoài cùng)
      baseSpeed = SPEED_RUN_MIN;
      if (sensor & 0xC0)      vitri = -7;   // S1 hoặc S2 sáng
      else if (sensor & 0x03) vitri = 7;    // S7 hoặc S8 sáng
      curveSpeed();
      break;
  }
}

//=====================================================================
//  9. ĐỌC CẢM BIẾN + TÍNH PD  (chạy mỗi 2 ms trong controlTask)
//=====================================================================
void read_sensor() {
  uint8_t temp = 0;
  static bool lineLost = false;
  PIDfre++;

  // --- đọc 8 mắt ---
  for (int j = 0; j < 8; j++) {
    uint32_t s = 0;
    for (int k = 0; k < ADC_SAMPLES; k++) s += analogRead(SENSOR_PINS[j]);
    int v = (int)(s / ADC_SAMPLES);
#if !LINE_READS_HIGH
    v = ADC_MAX - v;   // đảo để "vạch = số cao"
#endif
    sensorValue[j] = (uint16_t)v;
  }

  // --- chuẩn hóa 0..1000 và tạo byte nhị phân ---
  for (int j = 0; j < 8; j++) {
    if (isCalib == 0 && calibOK) {
      uint16_t v = sensorValue[j];
      if (v < v_min[j]) v = v_min[j];
      if (v > v_max[j]) v = v_max[j];
      sensorValue[j] = v;
      // calibOK đảm bảo v_max - v_min >= MIN_CONTRAST nên không chia cho 0
      sensorPID[j] = (uint16_t)map(v, v_min[j], v_max[j], 0, 1000);
    }
    temp = temp << 1;
    if (sensorValue[j] > v_th[j]) temp |= 0x01;
  }
  sensor = temp;   // chỉ gán MỘT LẦN sau khi xong cả 8 bit

  // --- PD: cứ 2 lần đọc (4 ms) tính 1 lần ---
  if (PIDfre >= 2) {
    PIDfre = 0;
    if (isCalib == 0 && calibOK) {
      long avg = 0;
      long sum = 0;
      for (int j = 0; j < 8; j++) {
        avg += (long)sensorPID[j] * (j * 1000L);
        sum += sensorPID[j];
      }
      if (sum > 50) {
        int center = 3500;
        if (isLeft == 1) center = 4000;
        else if (isRight == 1) center = 3000;
        int i = (int)(avg / sum) - center;
        posPID = i;
        if (lineLost) { lastPos = i; lineLost = false; }   // vừa tìm lại line: bỏ xung D
        int iP = (int)(kp * i);
        int iD = kd * (lastPos - i);
        int iRet = iP - iD;
        iRet = constrain(iRet, -4000, 4000);
        servoPwm = iRet / PID_DIV;
        steerAvg = (steerAvg * (STEER_AVG_N - 1) + servoPwm) / STEER_AVG_N;
        lastPos = i;
      } else {
        lineLost = true;
      }
      // sum quá nhỏ (mất line): giữ nguyên servoPwm cũ
    }
  }
}

unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}

char check_crossline() {   // vạch ngang toàn phần
  return (sensorMask(0xff) == 0xff) ? 1 : 0;
}

char check_crosshalfline() {   // nửa vạch ngang: 1 = trái, 2 = phải
  if (check_crossline() == 0) {
    if (isInturn90 == 1) {
      if (sensorMask(0xe0) == 0xe0) return 1;
      else if (sensorMask(0x07) == 0x07) return 2;
      else return 0;
    } else {
      if (cnt3 > 30) {
        if (sensorMask(0xf0) == 0xf0) return 1;
        else if (sensorMask(0x0f) == 0x0f) return 2;
        else return 0;
      }
    }
  }
  return 0;
}

//=====================================================================
//  10. ĐIỀU KHIỂN ĐỘNG CƠ QUA L298N
//=====================================================================
// angle > 0: rẽ phải (bánh trái nhanh hơn). speed1: tốc độ cơ sở.
void handleAndSpeed(int angle, int speed1) {
  if ((speed1 + angle) > 255) speed1 = 255 - angle;
  if ((speed1 - angle) > 255) speed1 = 255 + angle;
  speed_run(speed1 + angle, speed1 - angle);
}

static void pwmDuty(int side, uint32_t duty) {   // side 0 = phải, 1 = trái
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(side == 0 ? R_EN : L_EN, duty);
#else
  ledcWrite(side, duty);
#endif
}

// speed > 0: tiến, < 0: lùi, = 0: thả trôi
static void driveMotor(int side, int speed) {
  static int      prevSpd[2] = {0, 0};   // lệnh tốc độ lần trước của từng bánh
  static uint32_t kickEnd[2] = {0, 0};   // thời điểm hết "đạp ga"
  int pinF, pinB;
  if (side == 0) { pinF = R_IN2; pinB = R_IN1; }   // PHẢI tiến: IN1 = LOW,  IN2 = HIGH
  else           { pinF = L_IN3; pinB = L_IN4; }   // TRÁI tiến: IN3 = HIGH, IN4 = LOW
  if (speed > 0) {
    digitalWrite(pinF, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinF, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinF, LOW);
    digitalWrite(pinB, LOW);
  }

  // KICK-START: bánh đang đứng yên mà được ra lệnh chạy thật sự thì cấp KICK_PWM trong KICK_MS.
  uint32_t duty = (uint32_t)abs(speed);
  if (speed != 0) {
    uint32_t now = millis();
    if (prevSpd[side] == 0 && abs(speed) >= RAMP_START_PWM) kickEnd[side] = now + KICK_MS;
    if ((int32_t)(kickEnd[side] - now) > 0 && duty < KICK_PWM) duty = KICK_PWM;
  }
  prevSpd[side] = speed;
  pwmDuty(side, duty);
}

// speedDC_left / right: -255..255 (âm = quay ngược)
void speed_run(int speedDC_left, int speedDC_right) {
  if (estop) {                      // dừng khẩn cấp: ép về 0 dù code gọi gì
    speedDC_left = 0;
    speedDC_right = 0;
  }
  speedDC_left  = constrain(speedDC_left, -255, 255);
  speedDC_right = constrain(speedDC_right, -255, 255);
  driveMotor(1, speedDC_left);
  driveMotor(0, speedDC_right);
}

//=====================================================================
//  11. CUA VUÔNG (vạch ngang toàn phần)
//=====================================================================
int gocturn90(int tocdo) {
  isInturn90 = 1;
  int pattern2 = 19;
  cnt = 0;
  uint32_t t0 = millis();
  while (1) {
    if (estop || (millis() - t0) > MANEUVER_TIMEOUT_MS) {
      isInturn90 = 0;
      return 100;
    }
    switch (pattern2) {
      case 19:   // giảm tốc trước khi cua
        if (cnt < timeBrake90) {
          handleAndSpeed(servoPwm, speedBrake90);
        } else {
          pattern2 = 23;
        }
        break;

      case 23:   // chờ nửa vạch để biết hướng cua
        if (cnt > 1500) {
          isInturn90 = 0;
          return 11;
        }
        if (sensorMask(0xff) == 0xff) {   // lại thấy vạch ngang toàn phần: vạch đích
          speed_run(0, 0);
          isInturn90 = 0;
          return 100;
        }
        handleAndSpeed(servoPwm, tocdo);
        switch (check_crosshalfline()) {
          case 2:   // phải
            handleAndSpeed(servoPwm, tocdo);
            pattern2 = 40;
            setLed(2);
            break;
          case 1:   // trái
            handleAndSpeed(servoPwm, tocdo);
            pattern2 = 30;
            break;
          default:
            break;
        }
        break;

      // ---------------- cua TRÁI ----------------
      case 30:
        if (sensorMask(0xff) == 0x00) {
          pattern2 = 31;
          break;
        }
        handleAndSpeed(servoPwm, tocdo);
        break;

      case 31:
        speed_run(-20, tocdo);
        if (sensorMask(0x3c) != 0x00) {
          cnt = 0;
          pattern2 = 35;
          break;
        }
        if (sensorMask(0x0f) == 0x03) {
          cnt = 0;
          pattern2 = 32;
          break;
        }
        break;

      case 32:
        speed_run(-20, tocdo);
        if (sensorMask(0x0f) == 0x01) pattern2 = 33;
        break;

      case 33:
        speed_run(tocdo * 50 / 100, tocdo);
        if (sensorMask(0xe0) == 0xc0) pattern2 = 34;
        break;

      case 34:
        speed_run(0, 0);
        if (sensorMask(0x18) != 0x00) {
          pattern2 = 35;
          cnt = 0;
        }
        break;

      case 35:   // tăng tốc mềm sau cua: bắt đầu từ RAMP_START_PWM thay vì 0
        while ((cnt + RAMP_START_PWM) < tocdo && !estop) {
          handleAndSpeed(servoPwm, cnt + RAMP_START_PWM);
          delay(1);
        }
        cnt = 0;
        pattern2 = 36;
        vitri = 3;
        break;

      case 36:   // lấy lại tốc độ nhanh gấp đôi (cnt*2)
        while ((cnt * 2 + tocdo) < speed_run_forward && !estop) {
          handleAndSpeed(servoPwm, (cnt * 2 + tocdo));
          delay(1);
        }
        isInturn90 = 0;
        HaveToBrake = 0;
        cnt3 = 100;
        baseSpeed = speed_run_forward;
        return 11;

      // ---------------- cua PHẢI ----------------
      case 40:
        if (sensorMask(0xff) == 0x00) {
          pattern2 = 41;
          break;
        }
        handleAndSpeed(servoPwm, tocdo);
        break;

      case 41:
        speed_run(tocdo, -20);
        if (sensorMask(0x3c) != 0x00) {
          cnt = 0;
          setLed(0);
          pattern2 = 45;
          break;
        }
        if (sensorMask(0xf0) == 0xc0) {
          cnt = 0;
          setLed(1);
          pattern2 = 42;
          break;
        }
        break;

      case 42:
        speed_run(tocdo, -20);
        if (sensorMask(0xf0) == 0x80) {
          pattern2 = 43;
          setLed(2);
        }
        break;

      case 43:
        speed_run(tocdo, tocdo * 50 / 100);
        if (sensorMask(0x07) == 0x03) {
          pattern2 = 44;
          setLed(4);
        }
        break;

      case 44:
        speed_run(0, 0);
        if (sensorMask(0x18) != 0x00) {
          pattern2 = 45;
          cnt = 0;
          setLed(4);
        }
        break;

      case 45:   // như case 35
        while ((cnt + RAMP_START_PWM) < tocdo && !estop) {
          handleAndSpeed(servoPwm, cnt + RAMP_START_PWM);
          delay(1);
        }
        cnt = 0;
        pattern2 = 46;
        vitri = -3;
        setLed(2);
        break;

      case 46:   // như case 36
        while ((cnt * 2 + tocdo) < speed_run_forward && !estop) {
          handleAndSpeed(servoPwm, (cnt * 2 + tocdo));
          delay(1);
        }
        isInturn90 = 0;
        HaveToBrake = 0;
        cnt3 = 100;
        baseSpeed = speed_run_forward;
        setLed(4);
        return 11;

      default:
        isInturn90 = 0;
        return 11;
    }
  }
}

//=====================================================================
//  12. CHUYỂN LÀN
//=====================================================================
int left_lance(int tocdo) {
  int pattern3 = 51;
  isLeft = 1;
  uint32_t t0 = millis();
  while (1) {
    if (estop || (millis() - t0) > MANEUVER_TIMEOUT_MS) {
      isLeft = 0;
      return 100;
    }
    if (check_crossline()) {
      cnt = 0;
      isLeft = 0;
      return 21;
    }

    switch (pattern3) {
      case 51:
        if (sensorMask(0xff) == 0x00) {
          speed_run(tocdo * 80 / 100, tocdo);
          pattern3 = 56;
          cnt = 0;
          isLeft = 0;
          break;
        } else {
          handleAndSpeed(servoPwm, tocdo);
        }
        if (cnt > 500) {
          cnt = 0;
          isLeft = 0;
          return 11;
        }
        break;

      case 56:
        if ((sensorMask(0x80) != 0) && (cnt > 50)) {
          cnt = 0;
          speed_run(tocdo * 105 / 100, tocdo);
          pattern3 = 57;
        } else {
          speed_run(tocdo * 80 / 100, tocdo);
        }
        break;

      case 57:
        if (sensorMask(0x30) == 0x30) {
          cnt = 0;
          pattern3 = 58;
          vitri = 0;
          HaveToBrake = 0;
          cnt3 = 0;
          isHighSpeed = 0;
          break;
        } else {
          speed_run(tocdo * 105 / 100, tocdo);
        }
        break;

      case 58:
        if ((cnt / 4 + tocdo) < speed_run_forward) {
          handleAndSpeed(servoPwm, (cnt / 4 + tocdo));
        } else {
          cnt3 = 400;
          cnt4 = 50;
          HaveToBrake = 1;
          isHighSpeed = 1;
          baseSpeed = speed_run_forward;
          return 11;
        }
        break;

      default:
        isLeft = 0;
        return 11;
    }
  }
}

int right_lance(int tocdo) {
  int pattern4 = 61;
  isRight = 1;
  uint32_t t0 = millis();
  while (1) {
    if (estop || (millis() - t0) > MANEUVER_TIMEOUT_MS) {
      isRight = 0;
      return 100;
    }
    if (check_crossline()) {
      cnt = 0;
      isRight = 0;
      return 21;
    }

    switch (pattern4) {
      case 61:
        if ((sensorMask(0xff) == 0x00) && (cnt > 20)) {
          speed_run(tocdo, tocdo * 70 / 100);
          pattern4 = 66;
          cnt = 0;
          isRight = 0;
          break;
        } else {
          if ((speed_run_forward - cnt) > tocdo) {
            handleAndSpeed(servoPwm, speed_run_forward - cnt);
          } else {
            handleAndSpeed(servoPwm, tocdo);
          }
        }
        if (cnt > 500) {
          cnt = 0;
          isRight = 0;
          return 11;
        }
        break;

      case 66:
        if (sensorMask(0x03) != 0) {
          cnt = 0;
          pattern4 = 67;
        } else {
          speed_run(tocdo, tocdo * 70 / 100);
        }
        break;

      case 67:
        if (sensorMask(0x0c) != 0x00) {
          handleAndSpeed(servoPwm, 0);
          cnt = 0;
          pattern4 = 68;
          vitri = 0;
          HaveToBrake = 0;
          cnt3 = 0;
          isHighSpeed = 0;
          break;
        } else {
          speed_run(tocdo, tocdo * 180 / 100);
        }
        break;

      case 68:
        if (cnt < 20) {
          handleAndSpeed(servoPwm, 0);
        } else {
          cnt = 0;
          pattern4 = 69;
        }
        break;

      case 69:   // tăng tốc sau chuyển làn: bắt đầu từ RAMP_START_PWM, nhanh gấp đôi
        if ((cnt * 2 + RAMP_START_PWM) < speed_run_forward) {
          handleAndSpeed(servoPwm, cnt * 2 + RAMP_START_PWM);
        } else {
          handleAndSpeed(servoPwm, speed_run_forward);
          cnt3 = 0;
          HaveToBrake = 0;
          isHighSpeed = 0;
          baseSpeed = speed_run_forward;
          return 11;
        }
        break;

      default:
        isRight = 0;
        return 11;
    }
  }
}

//=====================================================================
//  13. HỌC MÀU (thay cho SW1 của code gốc) + LƯU FLASH
//=====================================================================
void updateLine() {
  static uint32_t tPrint = 0, tBeep = 0;

  for (int i = 0; i < 8; i++) {
    uint16_t v = sensorValue[i];
    if (v < v_min[i]) v_min[i] = v;
    if (v > v_max[i]) v_max[i] = v;
    v_th[i] = (v_min[i] + v_max[i]) / 2;
  }

  if (millis() - tPrint > 100) {    // in giá trị ra Serial mỗi 100 ms
    tPrint = millis();
    for (int i = 0; i < 8; i++) Serial.printf("%4u ", sensorValue[i]);
    Serial.println();
  }
  if (sensor == 0xff && millis() - tBeep > 1000) {   // cả 8 mắt cùng thấy "vạch"
    tBeep = millis();
    beep(50);
  }
  setLed((millis() / 100) % 3);
}

bool calibValid() {
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    if (v_max[i] < v_min[i] + MIN_CONTRAST) {
      Serial.printf("  Mat S%d: chenh lech qua nho (min=%u max=%u)\n", i + 1, v_min[i], v_max[i]);
      ok = false;
    }
  }
  return ok;
}

void saveCalib() {
  prefs.begin("linebot", false);
  prefs.putBytes("vmin", v_min, sizeof(v_min));
  prefs.putBytes("vmax", v_max, sizeof(v_max));
  prefs.putBool("ok", true);
  prefs.end();
}

bool loadCalib() {
  for (int i = 0; i < 8; i++) {
    v_min[i] = 0;
    v_max[i] = 0;
    v_th[i] = ADC_MAX / 2;
  }
  prefs.begin("linebot", false);
  bool saved = prefs.getBool("ok", false);
  if (saved) {
    prefs.getBytes("vmin", v_min, sizeof(v_min));
    prefs.getBytes("vmax", v_max, sizeof(v_max));
  }
  prefs.end();
  if (!saved) return false;
  for (int i = 0; i < 8; i++) v_th[i] = (v_min[i] + v_max[i]) / 2;
  return calibValid();
}

// durationMs = 0: quét đến khi nhấn BOOT (hoặc gửi 's') thì lưu.
// durationMs > 0: tự quét trong khoảng thời gian đó rồi lưu.
void calibrate(uint32_t durationMs) {
  calibOK = false;
  isCalib = 1;      // controlTick ngừng kẹp/chuẩn hóa, chỉ đưa giá trị thô
  for (int i = 0; i < 8; i++) {
    v_min[i] = ADC_MAX + 100;
    v_max[i] = 0;
    v_th[i] = ADC_MAX / 2;
  }
  Serial.println(">>> HOC MAU: dua mang cam bien qua lai tren VACH va NEN.");
  if (durationMs == 0) Serial.println(">>> Xong thi nhan BOOT (hoac gui 's') de luu.");

  uint32_t t0 = millis();
  bool done = false;
  while (!done) {
    updateLine();
    if (durationMs > 0) {
      if (millis() - t0 >= durationMs) done = true;
    } else {
      if (BUTTON1 >= 0 && digitalRead(BUTTON1) == LOW) {
        delay(30);
        if (digitalRead(BUTTON1) == LOW) {
          while (digitalRead(BUTTON1) == LOW) delay(10);
          done = true;
        }
      }
      while (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'c') done = true;
      }
    }
    delay(2);
  }

  isCalib = 0;
  setLed(4);
  if (calibValid()) {
    saveCalib();
    calibOK = true;
    Serial.println(">>> DA LUU hoc mau.");
    beep(300);
  } else {
    calibOK = false;
    Serial.println(">>> HOC MAU LOI. Kiem tra: nguon cam bien, LINE_READS_HIGH, do cao cam bien; roi hoc lai.");
    for (int i = 0; i < 5; i++) { beep(80); delay(80); }
  }
}

//=====================================================================
//  14. CHỜ LỆNH, KHỞI ĐỘNG, THỬ NGHIỆM
//=====================================================================
static bool btnDown(int pin) {
  return pin >= 0 && digitalRead(pin) == LOW;
}

static bool btnPressed(int pin) {     // nhấn có lọc nhiễu
  if (!btnDown(pin)) return false;
  delay(25);
  return btnDown(pin);
}

static void waitRelease(int pin) {
  while (btnDown(pin)) delay(10);
  delay(30);
}

int readCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': return CMD_RUN1;
      case '2': return CMD_RUN2;
      case '3': return CMD_RUN3;
      case 'c': return CMD_CALIB;
      case 't': return CMD_SENSOR;
      case 'm': return CMD_MOTOR;
      case 'p': return CMD_PWMTEST;
      case 'h': printHelp(); break;
      default: break;
    }
  }
  if (btnPressed(BUTTON1)) {
    uint32_t t0 = millis();
    while (btnDown(BUTTON1) && (millis() - t0) < LONG_PRESS_MS) delay(10);
    bool isLong = btnDown(BUTTON1);
    waitRelease(BUTTON1);
    return isLong ? CMD_CALIB : CMD_RUN1;
  }
  if (btnPressed(BUTTON2)) { waitRelease(BUTTON2); return CMD_RUN2; }
  if (btnPressed(BUTTON3)) { waitRelease(BUTTON3); return CMD_RUN3; }
  return CMD_NONE;
}

void idleTask() {
  // LED: xanh lá nhấp nháy = sẵn sàng, đỏ nhấp nháy = chưa học màu
  if (calibOK) setLed(((millis() / 500) % 2) ? 1 : 4);
  else         setLed(((millis() / 250) % 2) ? 0 : 4);

  switch (readCommand()) {
    case CMD_RUN1:    startRun(nut1); break;
    case CMD_RUN2:    startRun(nut2); break;
    case CMD_RUN3:    startRun(nut3); break;
    case CMD_CALIB:   calibrate(0);   break;
    case CMD_SENSOR:  sensorTest();   break;
    case CMD_MOTOR:   motorTest();    break;
    case CMD_PWMTEST: pwmTest();      break;
    default: break;
  }
}

void startRun(int spd) {
  if (!calibOK) {
    Serial.println("CHUA HOC MAU, khong chay. Giu BOOT > 1s (hoac gui 'c') de hoc mau.");
    for (int i = 0; i < 3; i++) { beep(80); delay(80); }
    return;
  }
  Serial.printf("Chay toc do %d sau %d ms. (BOOT = dung, tu dung sau %lu ms)\n",
                spd, START_DELAY_MS, (unsigned long)MAX_RUN_MS);
  beep(100);
  delay(START_DELAY_MS);

  estop = false;
  isInturn90 = 0;
  isLeft = 0;
  isRight = 0;
  isHighSpeed = 0;
  isLowSpeed = 0;
  HaveToBrake = 0;
  vitri = 0;
  lastPos = 0;
  servoPwm = 0;
  steerAvg = 0;          // reset lực lái trung bình mỗi lần xuất phát
  cnt3 = 0;
  cnt4 = 0;
  speed_run_forward = spd;
  baseSpeed = SPEED_RUN_MIN;   // xuất phát ngay từ tốc độ đủ lăn (kết hợp kick-start)
  cnt = 0;
  runStartMs = millis();
  running = true;
  pattern = 10;
}

void sensorTest() {
  Serial.println("XEM CAM BIEN (gui ky tu bat ky hoac nhan BOOT de thoat).");
  Serial.println("Mat nam tren VACH phai ra so CAO / bit 1. Neu nguoc: doi LINE_READS_HIGH.");
  while (Serial.available()) Serial.read();
  isCalib = 1;   // hiển thị giá trị thô (không kẹp)
  while (!Serial.available() && !btnDown(BUTTON1)) {
    for (int i = 0; i < 8; i++) Serial.printf("%4u ", sensorValue[i]);
    Serial.print(" | ");
    for (int b = 7; b >= 0; b--) Serial.print((sensor >> b) & 1);
    Serial.println();
    delay(100);
  }
  while (Serial.available()) Serial.read();
  waitRelease(BUTTON1);
  isCalib = 0;
}

void motorTest() {
  Serial.println("THU DONG CO. NHAC BANH XE KHOI MAT DAT! Bat dau sau 3 giay...");
  beep(100);
  delay(3000);
  Serial.println("Banh PHAI tien");
  speed_run(0, 150);
  delay(1500);
  speed_run(0, 0);
  delay(700);
  Serial.println("Banh TRAI tien");
  speed_run(150, 0);
  delay(1500);
  speed_run(0, 0);
  delay(700);
  Serial.println("Hai banh tien");
  speed_run(150, 150);
  delay(1500);
  speed_run(0, 0);
  Serial.println("Xong. Banh nao quay nguoc thi doi 2 day OUT cua banh do (hoac IN tuong ung).");
}

// Dò mức PWM tối thiểu để XE (đang có tải, trên đúng mặt đường đua) bắt đầu lăn.
// Gửi 'p', đặt xe xuống mặt đường đua với khoảng trống phía trước. Sau 3 giây PWM của
// cả 2 bánh tăng dần (mỗi 0.4 s thêm 5). Thấy xe BẮT ĐẦU lăn thì gửi ký tự bất kỳ (hoặc nhấn BOOT).
void pwmTest() {
  Serial.println("DO PWM TOI THIEU. Dat xe xuong mat duong dua, chua trong phia truoc. Bat dau sau 3 giay...");
  Serial.println("Thay xe BAT DAU LAN thi gui ky tu bat ky (hoac nhan BOOT).");
  beep(100);
  delay(3000);
  while (Serial.available()) Serial.read();
  for (int pwm = 40; pwm <= 255; pwm += 5) {
    Serial.printf("PWM = %d\n", pwm);
    speed_run(pwm, pwm);
    uint32_t t0 = millis();
    while (millis() - t0 < 400) {
      if (Serial.available() || btnDown(BUTTON1)) {
        speed_run(0, 0);
        while (Serial.available()) Serial.read();
        waitRelease(BUTTON1);
        Serial.printf(">>> Xe bat dau lan o PWM ~ %d. Dat SPEED_RUN_MIN >= %d va RAMP_START_PWM ~ %d.\n",
                      pwm, pwm + 30, pwm + 10);
        return;
      }
      delay(5);
    }
  }
  speed_run(0, 0);
  Serial.println("Xe KHONG lan ngay ca o PWM 255: kiem tra pin, nguon L298N, day dong co, co khi (ket banh).");
}

//=====================================================================
//  15. LED RGB ONBOARD VÀ CÒI
//=====================================================================
// color: 0 = đỏ, 1 = xanh lá, 2 = xanh dương, 3 = trắng, khác = tắt
void setLed(int color) {
#ifdef RGB_BUILTIN
  static int last = -2;
  if (color == last) return;
  last = color;
  const uint8_t B = 20;
  switch (color) {
    case 0:  neopixelWrite(RGB_BUILTIN, B, 0, 0); break;
    case 1:  neopixelWrite(RGB_BUILTIN, 0, B, 0); break;
    case 2:  neopixelWrite(RGB_BUILTIN, 0, 0, B); break;
    case 3:  neopixelWrite(RGB_BUILTIN, B, B, B); break;
    default: neopixelWrite(RGB_BUILTIN, 0, 0, 0); break;
  }
#else
  (void)color;
#endif
}

void beep(int ms) {
  if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, HIGH);
  setLed(3);
  delay(ms);
  if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);
  setLed(4);
}
