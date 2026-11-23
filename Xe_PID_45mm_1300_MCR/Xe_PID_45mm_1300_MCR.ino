//Đây là code của dò line cho 
// hoang anh chó 
#include <avr/interrupt.h>
#include <EEPROM.h>
#define RDIR 11     // chiều quay động cơ phải
#define RPWM 6      // tốc độ động cơ phải
#define LDIR 9      // chiều quay động cơ trái
#define LPWM 5      // tốc độ động cơ trái
#define BUTTON1 7   // nút nhấn 1
#define BUTTON2 8   // nút nhấn 2
#define BUTTON3 12  // nút nhấn 3
#define BUZZER 4    // chân điều khiển loa
#define RED 3       // Chân RED trên mạch
#define GREEN 2     // Chân GREEN trên mạch
#define BLUE 13     // Chân BLUE trên mạch
//---------------------------------------------------
int addr = 0;
volatile int lastPos;
volatile unsigned char isCalib = 0;
volatile int lastServoPwm = 0;
volatile int servoPwm;
volatile unsigned char sensor;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
int speed_run_forward;
volatile int isHighSpeed = 0;
volatile int isLowSpeed = 0;
volatile int HaveToBrake = 0;
int timeBrakeCurve = 20;
int speedBrackCurve = 90;
int speedBrake90 = 100;
int timeToHighSpeed = 150;
int vitri = 0;
int isInturn90 = 0;
int timeBrake90 = 30;
volatile int isLeft = 0;
volatile int isRight = 0;
int speed90 = 90;
int speedChLan = 100;
volatile int posPID;
volatile int cnt = 0;
volatile int cnt1 = 0;
volatile int cnt2 = 0;
volatile int cnt3 = 0;
volatile int cnt4 = 0;
unsigned char pattern, start;
int line = 0;
int RememberLine = 0;
volatile float kp;
volatile int kd;
volatile int PIDfre = 0;
int nut1 = 180;
int nut2 = 200;
int nut3 = 220;
void setup() {


  //---------------PWM------------------------//
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RDIR, LOW);
  digitalWrite(LDIR, LOW);
  digitalWrite(RED, 1);
  digitalWrite(GREEN, 1);
  digitalWrite(BLUE, 1);
  //-------------------------------------------------//
  //CÀI ĐẶT CÁC THÔNG SỐ BAN ĐẦU
  speed_run(0, 0);  // dừng 2 bánh
  pattern = 9;      // set trạng thái chạy
  start = 0;        // Trước khi chạy học màu lại, đưa cảm biến qua lại trên đường line rồi bấm SW1 để lưu
  readEeprom();
  Serial.begin(9600);
  //-------------------------------------------------//
  // for (int i = 2000; i < 3500; i += 500) {
  //   tone(BUZZER, i, 100);
  //   delay(100);
  // }
  timer_init();
  isCalib = 0;
  pinMode(BUZZER, OUTPUT);
  beep(50);
  //-------------------------------------------------//
}
void timer_init() {
  // Timer/Counter 2 initialization
  ASSR = (0 << EXCLK) | (0 << AS2);
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  //TCNT2 = 0xB2;
  TCNT2 = 0xE0;
  //TCNT2 = 0xF0;
  OCR2A = 0x00;
  OCR2B = 0x00;
  // Timer/Counter 2 Interrupt(s) initialization
  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);
}
ISR(TIMER2_OVF_vect) {
  //TCNT2 = 0xB2;
  TCNT2 = 0xE0;
  //TCNT2 = 0xF0;
  read_sensor();
  cnt++;
  //cnt2++;
  if (isHighSpeed == 1) {

    if (cnt3 < timeToHighSpeed)
      cnt3++;
    else {
      cnt3++;
      HaveToBrake = 1;
      cnt4 = 0;
    }
  } else if (isLowSpeed == 1) {
    if (HaveToBrake == 1)
      cnt4++;
    if (cnt4 >= timeBrakeCurve) {
      HaveToBrake = 0;
      cnt3 = 0;
    } else {
      cnt3 = 0;
    }
  } else if ((isHighSpeed == 0) && (isLowSpeed == 0)) {
    if (HaveToBrake == 1)
      cnt4++;
    if (cnt4 >= timeBrakeCurve) {
      HaveToBrake = 0;
      cnt3 = 0;
    }
  }
}
void loop()  // vòng lặp, không dùng delay() trong này
{
  while (start == 0)  // nhấn nút để chạy
  {
    if (isCalib == 0) {
      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để học màu
        for (int i = 0; i < 8; i++) {
          black_value[i] = 1100;
        }
        for (int i = 0; i < 8; i++) {
          white_value[i] = 0;
        }
        for (int i = 0; i < 8; i++) {
          compare_value[i] = 1000;
        }
        beep(100);
        while (digitalRead(BUTTON1) == 0)
          ;
        isCalib = 1;
      }
      if (digitalRead(BUTTON2) == 0) {  // Nhấn SW2 để chạy
        beep(100);
        start = 1;  // học màu xong va bắt đầu chạy
        isCalib = 0;
        pattern = 10;
        cnt = 0;
        speed_run_forward = nut2;  // chậm
      }
      if (digitalRead(BUTTON3) == 0) {  // Nhấn SW3 để chạy
        beep(100);
        start = 1;  // học màu xong va bắt đầu chạy
        isCalib = 0;
        pattern = 10;
        cnt = 0;
        speed_run_forward = nut3;  // nhanh
      }
    } else {

      updateLine();
      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để chạy
        for (int i = 0; i < 8; i++) {
          // compare_value[i] = (black_value[i] + white_value[i]) / 2;
          EEPROM.write(addr + i, black_value[i] / 4);
          delay(1);
        }
        for (int i = 0; i < 8; i++) {
          // compare_value[i] = (black_value[i] + white_value[i]) / 2;
          EEPROM.write(8 + i, white_value[i] / 4);
          delay(1);
        }
        beep(300);

        while (digitalRead(BUTTON1) == 0)
          ;
        start = 1;  // học màu xong va bắt đầu chạy
        isCalib = 0;
        speed_run_forward = 0;  //test PID
      }
    }
  }

  switch (pattern) {  // đoạn mã trạng thái máy
    case 9:
      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để chạy
        beep(100);
        pattern = 10;
        cnt = 0;
        speed_run_forward = nut1;  //test PID
      }
      if (digitalRead(BUTTON2) == 0) {  // Nhấn SW2 để chạy
        beep(100);
        pattern = 10;
        cnt = 0;
        speed_run_forward = nut2;  // chậm
      }
      if (digitalRead(BUTTON3) == 0) {  // Nhấn SW3 để chạy
        beep(100);
        pattern = 10;
        cnt = 0;
        speed_run_forward = nut3;  // nhanh
      }
      break;
    case 10:
      if ((cnt*2) < speed_run_forward) {
        runforwardline(cnt*2);
      } else {
        //handleAndSpeed(servoPwm, 0);
        pattern = 11;
        cnt2 = 0;
        break;
      }
      break;
    case 11:
      // if (cnt2 > 2500) pattern = 100;
      if (check_crossline()) {  // Kiem tra line ngang cua vuong
        pattern = 21;
        break;
      } else {  // kiem tra nua line ngang qua chuyen lan
        switch (check_crosshalfline()) {
          case 1:
            pattern = 12;
            cnt = 0;
            break;
          case 2:
            pattern = 13;
            cnt = 0;
            break;
            // case 3:
            //   pattern = 21;
            //   cnt = 0;
            //   break;
          default:
            break;
        }
      }
      runforwardline(speed_run_forward);
      break;
    case 12:
      /* Check of large turning to the right completion */
      if (check_crossline()) { /* Crossline check even during turning */
        pattern = 21;
        break;
      }
      //pattern = left_lance(speedChLan);
      pattern = 11;
      break;
    case 13:
      /* Check of large turning completion to the left */
      if (check_crossline()) { /* Crossline check even during large turn */
        pattern = 21;
        break;
      }
      pattern = right_lance(speedChLan);
      break;

    case 21:
      /* Process when first crossline is detected */
      pattern = gocturn90(speed90);
      break;

    case 100:
      speed_run(0, 0);
      break;
    default:
      pattern = 11;
      break;
  }
}
///////////////////////////////////////////////////////////////////////
void runforwardline(int tocdo)  // hàm chạy bám line
{
  // if (HaveToBrake == 1)
  //   //RGB(1);
  //   else
  //     // RGB(4);
  switch (sensor) {
    case 0b00011000:
    case 0b00111100:
      if ((cnt3 / 3 + 130) > tocdo) {
        handleAndSpeed(servoPwm, tocdo);
      } else handleAndSpeed(servoPwm, cnt3 / 3 + 130);
      isLowSpeed = 0;
      isHighSpeed = 1;
      vitri = 0;
      break;

    case 0b00011100:
    case 0b00001000:
      if ((cnt3 / 3 + 130) > tocdo) {
        handleAndSpeed(servoPwm, tocdo);
      } else handleAndSpeed(servoPwm, cnt3 / 3 + 130);
      isLowSpeed = 0;
      isHighSpeed = 1;
      vitri = 1;
      break;

    case 0b00001100:
    case 0b00011110:
      if (vitri < -7) {
        speed_run(0, 70);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          // handleAndSpeed(servoPwm, speedBrackCurve);
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);
        } else {
          handleAndSpeed(servoPwm, 130);
        }
        vitri = 2;
      }
      break;

    case 0b00000100:
    case 0b00001110:
      if (vitri < -7) {
        speed_run(0, 70);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);

        } else {
          handleAndSpeed(servoPwm, 130);
        }
        vitri = 3;
      }
      break;
    case 0b00000110:
    case 0b00001111:
    case 0b00000010:
    case 0b00000111:
    case 0b00000011:
    case 0b00000001:
      if (vitri < -7) {
        speed_run(0, 70);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          //  handleAndSpeed(servoPwm, speedBrackCurve);
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);
        } else {
          handleAndSpeed(servoPwm, 130);
        }
        // handleAndSpeed(servoPwm, 130);
        vitri = 7;
      }
      break;

      ///////////////////////////////////////////////////////////////////////
    case 0b00000000:
      if (vitri <= -7) {
        speed_run(0, 70);
        vitri = -8;
        break;
      } else if (vitri >= 7) {
        speed_run(70, 0);
        vitri = 8;
        break;
      } else
        speed_run(0, 0);
      break;
    //////////////////////////////////////////////////////////////////////////////////
    case 0b00010000:
    case 0b00111000:
      if (vitri >= 7) {
        speed_run(70, 0);
        break;
      } else if ((cnt3 / 3 + 130) > tocdo) {
        handleAndSpeed(servoPwm, tocdo);
      } else handleAndSpeed(servoPwm, cnt3 / 3 + 130);
      isLowSpeed = 0;
      isHighSpeed = 1;
      vitri = -1;
      break;

    case 0b00110000:
    case 0b01111000:
      if (vitri >= 7) {
        speed_run(70, 0);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          //  handleAndSpeed(servoPwm, speedBrackCurve);
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);
        } else {
          handleAndSpeed(servoPwm, 130);
        }
        vitri = 2;
      }
      break;

    case 0b00100000:
    case 0b01110000:
      if (vitri >= 7) {
        speed_run(70, 0);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          //handleAndSpeed(servoPwm, speedBrackCurve);
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);
        } else {
          handleAndSpeed(servoPwm, 130);
        }
        vitri = -3;
      }
      break;
    case 0b01100000:
    case 0b11110000:
    case 0b01000000:
    case 0b11100000:
    case 0b11000000:
    case 0b10000000:
      if (vitri >= 7) {
        speed_run(70, 0);
        break;
      } else {
        isLowSpeed = 1;
        isHighSpeed = 0;
        if (HaveToBrake == 1) {
          //  handleAndSpeed(servoPwm, speedBrackCurve);
          if ((speed_run_forward - (cnt4 * 2)) > 130) {
            handleAndSpeed(servoPwm, speed_run_forward - (cnt4 * 2));
          } else handleAndSpeed(servoPwm, 130);
        } else {
          handleAndSpeed(servoPwm, 130);
        }
        //handleAndSpeed(servoPwm, 130);
        vitri = -7;
      }
      break;

      /////////////////////////////////////////////////////////////////
    default:
      handleAndSpeed(0, 0);
      break;
  }
}
//--------------------------------------------------------------------------//
void updateLine() {
  // Serial.println(sensor, BIN);
  if (sensor == 0xff) beep(100);
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
    if (black_value[i] == 0) black_value[i] = 1100;
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
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
void RGB(int color) {
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
  for (int i = 0; i < 8; i++) {
    black_value[i] = EEPROM.read(i) * 4;
  }
  for (int i = 0; i < 8; i++) {
    white_value[i] = EEPROM.read(8 + i) * 4;
  }
  for (int i = 0; i < 8; i++) {
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
  }
}
//-------------------------------------------------------------------//

//---------------------------------------------------------------------------//
void handleAndSpeed(int angle, int speed1) {
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
void speed_run(int speedDC_left, int speedDC_right)  // hàm truyền vào tốc độ động cơ trái + phải
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
  PIDfre++;
  //line đen
  // sensorValue[0] = 1023 - analogRead(A0);
  // sensorValue[1] = 1023 - analogRead(A1);
  // sensorValue[2] = 1023 - analogRead(A2);
  // sensorValue[3] = 1023 - analogRead(A3);
  // sensorValue[4] = 1023 - analogRead(A4);
  // sensorValue[5] = 1023 - analogRead(A5);
  // sensorValue[6] = 1023 - analogRead(A6);
  // sensorValue[7] = 1023 - analogRead(A7);
  // line trắng
  sensorValue[0] = analogRead(A0);
  sensorValue[1] = analogRead(A1);
  sensorValue[2] = analogRead(A2);
  sensorValue[3] = analogRead(A3);
  sensorValue[4] = analogRead(A4);
  sensorValue[5] = analogRead(A5);
  sensorValue[6] = analogRead(A6);
  sensorValue[7] = analogRead(A7);
  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  }
  if (PIDfre == 2) {
    PIDfre = 0;
    for (int j = 0; j < 8; j++) {
      avg += (long)(sensorPID[j]) * ((j)*1000);
      sum += sensorPID[j];
    }
    if (isLeft == 1) i = (int)((avg / sum) - 4000);
    else if (isRight == 1) i = (int)((avg / sum) - 3000);
    else i = (int)((avg / sum) - 3500);
    posPID = i;
    kp = 1;
    kd = 6;
    iP = kp * i;
    iD = kd * (lastPos - i);
    iRet = (iP - iD);
    if ((iRet < -4000)) {
      iRet = 0;
    }
    servoPwm = iRet / 27;  //30 Banh 68mm
                           // servoPwm = iRet / 60;//Banh 48mm
    lastPos = i;
  }
}
void beep(int timer) {
  digitalWrite(BUZZER, 1);
  delay(timer);
  digitalWrite(BUZZER, 0);
}
unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}
char check_crossline() {  // hàm kiểm tra line ngang
  if (sensorMask(0xff) == 0xff)
    return 1;
  else
    return 0;
}
char check_crosshalfline()  // hàm kiểm tra nữa line ngang
{
  if ((check_crossline() == 0)) {
    if (isInturn90 == 1) {
      if ((sensorMask(0xe0) == 0xe0))
        return 1;  // left
      else if ((sensorMask(0x07) == 0x07))
        return 2;  // right
      else
        return 0;
    } else {
      if (cnt3 > 30) {
        if ((sensorMask(0xf0) == 0xf0))
          return 1;
        else if ((sensorMask(0x0f) == 0x0f))
          return 2;
        else
          return 0;
      }
    }
  }
  return 0;
}
int gocturn90(int tocdo) {
  isInturn90 = 1;
  int pattern2 = 19;
  cnt = 0;
  while (1) {
    switch (pattern2) {
      case 19:
        if (cnt < timeBrake90) {
          handleAndSpeed(servoPwm, speedBrake90);
          break;
        } else {
          pattern2 = 23;
          break;
        }
        break;
      case 23:
        if (cnt > 1500) {
          isInturn90 = 0;
          return 11;
          break;
        }
        if (sensorMask(0xff) == 0xff) {
          speed_run(0, 0);
          return 100;
          break;
        }
        handleAndSpeed(servoPwm, tocdo);
        switch (check_crosshalfline()) {
          case 2:  // right
            handleAndSpeed(servoPwm, tocdo);
            pattern2 = 40;
            RGB(2);
            break;
          case 1:  // left
            handleAndSpeed(servoPwm, tocdo);
            pattern2 = 30;
            // RGB(2);
            // break;
            break;
          default:
            break;
        }
        break;

      case 30:
        if (sensorMask(0xff) == 0x00) {
          pattern2 = 31;
          break;
        }
        // if ((sensorMask(0x03) != 0x00)) {
        //   cnt = 0;
        //   pattern2 = 35;
        //   break;
        // }
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
        if ((sensorMask(0x0f) == 0x01)) {
          pattern2 = 33;
          break;
        }
        break;

      case 33:
        speed_run(tocdo * 50 / 100, tocdo);
        if ((sensorMask(0xe0) == 0xc0)) {
          pattern2 = 34;
          break;
        }
        break;

      case 34:
        speed_run(0, 0);
        if ((sensorMask(0x18) != 0x00)) {
          pattern2 = 35;
          cnt = 0;
          break;
        }
        break;

      case 35:
        while ((cnt) < tocdo) {
          handleAndSpeed(servoPwm, cnt);
        }
        cnt = 0;
        pattern2 = 36;
        vitri = 3;
        break;

      case 36:
        while ((cnt + tocdo) < speed_run_forward) {
          handleAndSpeed(servoPwm, (cnt + tocdo));
        }
        isInturn90 = 0;
        HaveToBrake = 0;
        cnt3 = 100;
        return 11;
        break;

      case 40:
        if (sensorMask(0xff) == 0x00) {
          pattern2 = 41;
          break;
        }
        // if ((sensorMask(0x70) == 0x60)) {
        //   cnt = 0;
        //   pattern2 = 45;
        //   break;
        // }
        handleAndSpeed(servoPwm, tocdo);
        break;

      case 41:
        speed_run(tocdo, -20);
        if (sensorMask(0x3c) != 0x00) {
          cnt = 0;
          RGB(0);
          pattern2 = 45;
          break;
        }
        if (sensorMask(0xf0) == 0xc0) {
          cnt = 0;
          RGB(1);
          pattern2 = 42;
          break;
        }
        break;

      case 42:
        speed_run(tocdo, -20);
        //speed_run(0, 0);
        if ((sensorMask(0xf0) == 0x80)) {
          pattern2 = 43;
          RGB(2);
          break;
        }
        break;

      case 43:
        speed_run(tocdo, tocdo * 50 / 100);
        if ((sensorMask(0x07) == 0x03)) {
          pattern2 = 44;
          RGB(4);
          break;
        }
        break;

      case 44:
        speed_run(0, 0);
        if ((sensorMask(0x18) != 0x00)) {
          pattern2 = 45;
          cnt = 0;
          RGB(4);
          break;
        }
        break;

      case 45:
        while ((cnt) < tocdo) {
          handleAndSpeed(servoPwm, cnt);
        }
        cnt = 0;
        pattern2 = 46;
        vitri = -3;
        RGB(2);
        break;

      case 46:
        while ((cnt + tocdo) < speed_run_forward) {
          handleAndSpeed(servoPwm, (cnt + tocdo));
        }
        isInturn90 = 0;
        HaveToBrake = 0;
        cnt3 = 100;
        RGB(4);
        return 11;
        break;
      default:
        return 11;
        break;
    }
  }
}
int left_lance(int tocdo) {
  int pattern3 = 51;
  isLeft = 1;
  while (1) {
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
        } else
          speed_run(tocdo * 80 / 100, tocdo);
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
        } else
          speed_run(tocdo * 105 / 100, tocdo);
        break;
      case 58:
        if ((cnt / 4 + tocdo) < speed_run_forward) {
          handleAndSpeed(servoPwm, (cnt / 4 + tocdo));
        } else {
          cnt3 = 400;
          cnt4 = 50;
          HaveToBrake = 1;
          isHighSpeed = 1;
          return 11;
        }
        break;
      default:
        return 11;
    }
  }
}
int right_lance(int tocdo) {
  int pattern4 = 61;
  isRight = 1;
  while (1) {
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
          } else handleAndSpeed(servoPwm, tocdo);
        }
        if (cnt > 500) {
          cnt = 0;
          isRight = 0;
          return 11;
        }
        break;
      case 66:
        if ((sensorMask(0x03) != 0)) {  //&& (cnt > 20)
          cnt = 0;
          speed_run(tocdo, tocdo * 180 / 100);
          speed_run(0, 0);
          pattern4 = 67;
        } else
          speed_run(tocdo, tocdo * 70 / 100);
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
        } else
          speed_run(tocdo, tocdo * 180 / 100);
        break;
      case 68:
        if (cnt < 20) {
          handleAndSpeed(servoPwm, 0);
        } else {
          cnt = 0;
          pattern4 = 69;
          break;
        }
        //handleAndSpeed(servoPwm, 0);
        break;
      case 69:
        // if ((cnt / 4 + tocdo) < speed_run_forward) {
        //   handleAndSpeed(servoPwm, (cnt / 4 + tocdo));
        // } else {
        //   handleAndSpeed(servoPwm, speed_run_forward);
        if (cnt < speed_run_forward) {
          handleAndSpeed(servoPwm, cnt);
        } else {
          handleAndSpeed(servoPwm, speed_run_forward);
          cnt3 = 0;
          HaveToBrake = 0;
          isHighSpeed = 0;
          return 11;
        }
        //handleAndSpeed(servoPwm, 0);
        break;
      default:
        return 11;
    }
  }
}
