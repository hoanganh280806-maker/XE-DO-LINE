
#include <avr/interrupt.h>
#include <EEPROM.h>
#define RDIR 11  // chiều quay động cơ phải
#define LDIR 9   // chiều quay động cơ trái
#define LPWM 5   // tốc độ động cơ trái
#define RPWM 6   // tốc độ động cơ phải
#define BUTTON1 7
#define BUZZER 4
#define RED 3
#define GREEN 2
#define BLUE 13
//---------------------------------------------------

volatile unsigned char isCalib = 0;  // biến trạng thái học màu
volatile int servoPwm;               // biến chứa thông số PID
volatile int lastPos;                // biến vị trí cảm biến lần lấy mẫu trước để tính PID
volatile unsigned char sensor;       // biến chứa 8 cảm biến đã xử lý, cho biết cảm biến nào đang trên line
unsigned int sensorValue[5];         // mảng chứa mẫu thô 8 cảm biến
unsigned int sensorPID[5];           // mảng chứa mẫu 8 cảm biến đã xử lý cân bằng cho PID
unsigned int black_value[5];         // mảng chứa 8 giá trị cảm biến thấp khi học màu
unsigned int white_value[5];         // mảng chứa 8 giá trị cảm biến cao khi học màu
unsigned int compare_value[5];       // mảng chứa 8 giá trị so sánh để biết cảm biến có mằn trên line hay không
int speed_run_forward;               // biến tốc độ chạy bám line
int cnt = 0;                         // biến thời gian để giải map
int sensorCount = 0;                 // biến chứa số lượng sensor trên line
unsigned char pattern, start;        // biến trạng thái
int RememberLine = 0;
int kp;
int kd;
void setup() {


  //---------------PWM------------------------//
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RDIR, LOW);
  digitalWrite(LDIR, LOW);
  digitalWrite(RED, 1);
  digitalWrite(GREEN, 1);
  digitalWrite(BLUE, 1);
  //-------------------------------------------------//
  speed_run(0, 0);
  pattern = 1;
  start = 0;
  readEeprom();
  Serial.begin(9600);
  //-------------------------------------------------//
  for (int i = 2000; i < 3500; i += 500) {
    tone(BUZZER, i, 100);
    delay(100);
  }
  timer_init();
  isCalib = 1;
  pinMode(BUZZER, OUTPUT);
  //-------------------------------------------------//
}
void timer_init() {  // cài đặt timer 2 để lấy mẫu cảm biến 5ms
  // Timer/Counter 2 initialization
  ASSR = (0 << EXCLK) | (0 << AS2);
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TCNT2 = 0xB2;
  OCR2A = 0x00;
  OCR2B = 0x00;
  // Timer/Counter 2 Interrupt(s) initialization
  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);
}
ISR(TIMER2_OVF_vect) {  // ngắt timer 2
  TCNT2 = 0xB2;
  read_sensor();
  cnt++;  // Biến xử lý thời gian của robot, dùng để giải map
}
//-------------------------------------------------//
void loop() {
  while (start == 0)  // nhấn nút để chạy
  {
    updateLine();
    if (digitalRead(BUTTON1) == 0) {
      while (digitalRead(BUTTON1) == 0) {
        delay(1000);
      }
      start = 1;
      isCalib = 0;
      cnt = 0;
      pattern = 1;
    }
  }

  switch (pattern) {
    case 1:
      if (digitalRead(BUTTON1) == 0) {
        start = 1;
        isCalib = 0;
        cnt = 0;
        pattern = 10;
        speed_run_forward = 150;
      }
      break;
    case 10:
      if (cnt >= 50) {  // nếu quá thời gian thì chuyển trạng thái
        pattern = 11;
        cnt = 0;
        break;
      }
      runforwardline(speed_run_forward);
      break;
    case 11:                                       // trạng thái vừa dò line vừa bắt line
      if (sensorMask(0b00000001) == 0b00000001) {  // bắt được đoạn cua gấp bên phải thì cài biến nhớ bằng 1
        RememberLine = 1;
      } else if (sensorMask(0b00010000) == 0b00010000) {  // bắt được đoạn cua gấp bên trái thì cài biến nhớ bằng -1
        RememberLine = -1;
      }
      if (sensor == 0b00000000) {  // nếu bị mất line do cua gấp và có biến nhớ thì nhảy vô trường hợp giải quyết cua gấp
        if (RememberLine != 0) {
          if (RememberLine == 1) {
            speed_run(speed_run_forward, 0);
            pattern = 12;
          } else if (RememberLine == -1) {
            speed_run(0, speed_run_forward);
            pattern = 12;
          } else pattern = 100;                             // chuyển trạng thái sang 12 để xử lý cua gấp
        } else {                                            // Nếu mất line mà không có biến nhớ thì chạy thẳng, đoạn line đứt khúc
          speed_run(speed_run_forward, speed_run_forward);  // chạy thẳng
        }
        break;
      } else runforwardline(speed_run_forward);  // Nếu không có gì xảy ra thì bám line bằng PID
      if (sensorMask(0b00001110) != 0b00000000) {
        RememberLine = 0;
      }
      break;
    case 12:  // xử lý cua gấp
      if (RememberLine == 1) {
        speed_run(speed_run_forward, -3);  // bánh phải phanh gấp, bánh trái chạy tới để bẻ cua sang phải
        pattern = 21;                      // chuyển tiếp trạng thái để xử lý cua phải
        RememberLine = 0;
        break;
      } else if (RememberLine == -1) {
        speed_run(-3, speed_run_forward);  // bánh trái phanh gấp, bánh phải chạy tới để bẻ cua sang trái
        pattern = 31;                      // chuyển tiếp trạng thái để xử lý cua trái
        RememberLine = 0;
        break;
      } else {  // nếu biến nhớ không phải trái hay phải thì trở về trạng thái bám line bằng PID
        pattern = 11;
        break;
      }
    case 21:  // xử lý cua phải
      speed_run(speed_run_forward, -3);
      RGB(1);
      if (sensorMask(0b00000111) != 0) {  // bắt được line thì giảm lực xoay để xe ổn định
        speed_run(speed_run_forward, speed_run_forward / 2);
        //   speed_run(30, speed_run_forward / 4);
        pattern = 22;  // chuyển tiếp trạng thái để xử lý
      }
      break;
    case 22:  // xử lý cua phải
      speed_run(speed_run_forward, speed_run_forward / 2);
      //speed_run(30, speed_run_forward / 4);
      RGB(1);
      if (sensorMask(0b00011100) != 0) {  // xe về vị trí ổn định thì chuyển trạng thái
        pattern = 23;
        RememberLine = 0;
        cnt = 0;
      }
      break;
    case 23:              // ổn định xe sau khi hoàn thành cua phải
      runforwardline(0);  // dừng hẳn xe để ổn định trong 100ms
      if (cnt > 20) {
        pattern = 11;  // khi đã ổn định thì chuyển về trạng thái bám line
        RememberLine = 0;
        cnt = 0;
      }
      break;

    case 31:  // 31 32 33 tương tự cua phải nhưng là xử lý cua bên trái
      speed_run(-3, speed_run_forward);
      RGB(2);
      if (sensorMask(0b00011100) != 0) {
        speed_run(speed_run_forward / 2, speed_run_forward);
        // speed_run(speed_run_forward / 4, 30);
        pattern = 32;
      }
      break;
    case 32:
      speed_run(speed_run_forward / 2, speed_run_forward);
      //  speed_run(speed_run_forward / 4, 30);
      RGB(2);
      if (sensorMask(0b00000111) != 0) {
        pattern = 33;
        RememberLine = 0;
        cnt = 0;
      }
      break;
    case 33:
      runforwardline(0);
      if (cnt > 20) {
        pattern = 11;
        RememberLine = 0;
        cnt = 0;
      }
      break;
    case 100:  // case kết thúc xử lý, không cần quan tâm, không ảnh hưởng đến map này
      speed_run(0, 0);
      RGB(millis() / 100 % 3);
      break;
    default:
      pattern = 11;
      break;
  }
}
///////////////////////////////////////////////////////////////////////
void runforwardline(int tocdo)  // hàm chạy bám line
{
  switch (sensor) {
    case 0b00000000:  // không có cảm biến trên line
    case 0b00000110:  // không có cảm biến trên line
    case 0b00000100:  // không có cảm biến trên line  // code
    case 0b00001100:  // không có cảm biến trên line
    case 0b00001110:  // không có cảm biến trên line
    case 0b00011100:  // không có cảm biến trên line
    case 0b00000111:  // không có cảm biến trên line
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 0b00000001:  // không có cảm biến trên line
    case 0b00010000:  // không có cảm biến trên line
      handleAndSpeed(servoPwm, tocdo-40);
      break;
    /////////////////////////////////////////////////////////////////
    default:
      handleAndSpeed(servoPwm, tocdo);
      break;
  }
}
//--------------------------------------------------------------------------//
void updateLine() {  // hàm học màu
  // Serial.println(sensor, BIN);
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
    if (black_value[i] == 0) black_value[i] = 1100;
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
  Serial.println();
  /*  if (digitalRead(BUTTON1) == 0) {
    for (int i = 0; i < 8; i++) {
      // compare_value[i] = (black_value[i] + white_value[i]) / 2;
      EEPROM.write(addr + i, compare_value[i] / 4);
    }
    digitalWrite(BUZZER, 1);
    delay(100);
    digitalWrite(BUZZER, 0);
    }*/
  RGB(millis() / 100 % 3);
}
void RGB(int color) {  // hàm điều khiển led 7 màu
  switch (color) {
    case 0:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
    case 1:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      break;
    case 2:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      break;
    default:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
  }
}
void readEeprom() {
  for (int i = 0; i < 5; i++) {
    compare_value[i] = EEPROM.read(i) * 4;
  }
}
//-------------------------------------------------------------------//

//---------------------------------------------------------------------------//
void handleAndSpeed(int angle, int speed1) {  // hàm bẻ lái xe, khi cho thông số tính toán PID vào vị trí angle thì sẽ thành bám line PID
  int speedLeft;
  int speedRight;
  if ((speed1 + angle) > 255) {
    speed1 = 255 - angle;
  }
  if ((speed1 - angle) > 255) {
    speed1 = 255 + angle;
  }
  speedLeft = speed1 + angle;
  speedRight = speed1 - angle;
  speed_run(speedLeft, speedRight);
}
//---------------------------------------------------------------------------//
void speed_run(int speedDC_left, int speedDC_right)  // hàm truyền vào tốc độ động cơ trái + phải thủ công không PID
{

  if (speedDC_left < 0) {
    analogWrite(LPWM, 255 + speedDC_left);
    digitalWrite(LDIR, HIGH);
  } else if (speedDC_left >= 0) {
    speedDC_left = speedDC_left;
    analogWrite(LPWM, speedDC_left);
    digitalWrite(LDIR, LOW);
  }
  if (speedDC_right < 0) {
    analogWrite(RPWM, 255 + speedDC_right);
    digitalWrite(RDIR, HIGH);
  } else if (speedDC_right >= 0) {
    speedDC_right = speedDC_right;
    analogWrite(RPWM, speedDC_right);
    digitalWrite(RDIR, LOW);
  }
}
//-----------------------------------------------------------//
void read_sensor()  // hàm đọc cảm biến
{
  unsigned char temp = 0;
  unsigned int sum = 0;
  unsigned long avg = 0;
  int i, iP, iD;
  int iRet;
  sensorCount = 0;
  sensorValue[0] = 1023 - analogRead(A0);  // ĐEN
  sensorValue[1] = 1023 - analogRead(A1);
  sensorValue[2] = 1023 - analogRead(A2);
  sensorValue[3] = 1023 - analogRead(A3);
  sensorValue[4] = 1023 - analogRead(A4);
  //SensorValue[5] = 1023 - analogRead(A5);
  // sensorValue[0] = analogRead(A0);  // TRẮNG
  // sensorValue[1] = analogRead(A1);
  // sensorValue[2] = analogRead(A2);
  // sensorValue[3] = analogRead(A3);
  // sensorValue[4] = analogRead(A4);
  for (int j = 0; j < 5; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])  //cảm biến không được nhỏ hơn giá trị min và lớn hơn giá trị max khi học màu
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);  // cân bằng cảm biến để tính toán PID
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {  // so sánh cảm biến để biết cảm biến đang trên line hay không
      temp |= 0x01;
      sensorCount++;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  }
  for (int j = 0; j < 5; j++) {  // đoạn tính toán PID, tuyến tính và đạo hàm
    avg += (long)(sensorPID[j]) * ((j)*1000);
    sum += sensorPID[j];
  }
  i = (int)((avg / sum) - 2000);  //  tính toán error trên cảm biến, con số 3500 này là vị trí đặt = 2000
  kp = 1;
  kd = 12;
  iP = kp * i;
  iD = kd * (lastPos - i);
  iRet = (iP - iD);
  if ((iRet < -2000)) {
    iRet = 0;
  }
  servoPwm = iRet / 20;  // chia ra để phù hợp với 8 bit trên arduino
  lastPos = i;
}
unsigned char sensorMask(unsigned char mask) {  // hàm hỗ trợ kiểm tra nhanh cảm biến trên line
  return (sensor & mask);
}
