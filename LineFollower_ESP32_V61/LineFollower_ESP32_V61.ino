/* V6.1 experimental control candidate, based on V6 commit 13cac4d.
 * Ideas independently implemented: Pololu 3pi+ LineFollowerPID, QTR Sensors,
 * Zumo MazeSolver (slow approach, overshoot/exit check); no copied source.
 * See README_V61.md for references and bench/track tuning procedure.
 * ESP32 Dev Module. Pin map unchanged from V5 executable code.
 * SAFETY: manual start, fresh calibration, sonar fail-stop, bounded search.
 * Bench commands test/spin: wheels lifted ONLY, x cancels. No autonomous tuning yet.
 * Wi-Fi disabled during ADC sampling; UDP only while idle. Configure WiFiConfig.h locally.
 */
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#if __has_include("WiFiConfig.h")
#include "WiFiConfig.h"
#else
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
const char* PC_IP = "192.168.1.104";
#endif
const uint16_t PC_PORT = 5005;
const uint8_t DUMP_BTN = 0;
const bool BLACK_LINE = true;
const uint8_t NUM_SENSORS = 8;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {39,33,34,25,35,26,32,27};
const uint8_t EN_L=21, IN1_L=23, IN2_L=22, EN_R=5, IN1_R=18, IN2_R=19;
const bool INVERT_LEFT=false, INVERT_RIGHT=true;
const uint8_t TRIG_PIN=12, ECHO_PIN=13, ENC_L_A=17, ENC_L_B=16, ENC_R_A=4, ENC_R_B=15;
const uint32_t PWM_FREQ=5000, LOOP_US=2000, CAL_TIME_MS=4000;
const uint8_t PWM_BITS=8;
const uint32_t LOST_GRACE_MS=40, SETTLE_MS=300, TURN_MAX_MS=1800, CORNER_HOLD_MS=80;
const uint32_t SEARCH_HARD_MAX_MS=5000, ARM_TIMEOUT_MS=2500, SONAR_STALE_MS=250;
const int CENTER_BAND=550, CORNER_TH=600;
const uint8_t CORNER_CONFIRM=3, CENTER_CONFIRM=4;
const uint32_t BRAKE_MS=90, ALIGN_BRAKE_MS=65, ADV_MAX_MS=700;
const uint32_t FINE_LOST_MS=120;
const int EARLY_EDGE=350, FOLLOW_DEADBAND=65;
const float OBSTACLE_CM=15.0f, CLEAR_CM=20.0f;
const float DERIV_ALPHA=0.4f;
float Kp=0.04f, Ki=0.0f, Kd=0.20f;
int baseSpeed=100, maxSpeed=140, lostSpin=90, turnSpeed=110;
int approachPWM=85, advancePWM=85, finePWM=75, trimPWM=0, lineZero=3500;
int turnAdvMs=80, fullTurnMs=2200, fullTurnCnt=0, advanceCnt=0, turn90Cnt=0;
// Optional straight-only encoder synchronization; OFF until encoder checks.
float syncKp=0.0f, syncKi=0.0f, syncI=0, syncCorrection=0;
uint32_t syncLastMs=0; bool syncActive=false;
bool cornerMode=true;
enum State { IDLE, CALIBRATING, ARMING, RUNNING, OBSTACLE_STOP, FAULT, BENCH_TEST, BENCH_SPIN, UDP_DUMP };
// Preserve first four numeric modes for existing UDP receiver.
enum RunMode { RM_FOLLOW, RM_ADVANCE, RM_TURN, RM_SEARCH, RM_BRAKE, RM_FINE, RM_ALIGN_BRAKE };
void stopState(State s,const char* reason);
void startBench(State s);
State state=IDLE;
RunMode rmode=RM_FOLLOW;
uint16_t calMin[8], calMax[8], sv[8];
bool calibrated=false, pwmOK=false, debugOn=false;
uint32_t modeStartMs=0, settleStartMs=0, lostSinceMs=0, holdStartMs=0, armStartMs=0, calStartMs=0;
bool holdActive=false, wasLost=false, leftOldLine=false;
int lastSide=0, turnDir=0, cornerCandidateDir=0;
uint8_t cornerCnt=0, centerCnt=0, leaveCnt=0;
float lastErr=0, integral=0, derivFilt=0;
bool havePrev=false;
int currentErr=0;
bool currentLine=false;
uint32_t lastPidUs=0, lastTickUs=0, tickDtUs=LOOP_US, overrunCount=0;
int outL=0,outR=0;
bool motorBraked=false;
volatile int32_t encL=0,encR=0;
int32_t searchEncL0=0,searchEncR0=0,lastEncL=0,lastEncR=0, speedL=0,speedR=0;
int32_t maneuverL0=0,maneuverR0=0;
uint32_t turnStartMs=0, fineLostSince=0;
bool fineMissing=false; uint8_t alignAttempts=0;
int targetL=0,targetR=0;
uint32_t lastEncMs=0;
portMUX_TYPE encMux=portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE echoMux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t echoRiseUs=0, echoWidthUs=0;
volatile bool echoRising=false,echoDone=false,echoArmed=false;
float distCm=-1;
bool usWaiting=false, obstacle=false;
uint8_t sonarBad=0, sonarClear=0;
uint32_t usTrigMs=0,lastValidMs=0;
bool sonarValid=false;
Preferences prefs;
const char* NVS_NS="linebot61"; // independent tuning and calibration
struct CalRecord { uint32_t magic; uint8_t pins[8]; uint8_t black; uint8_t reserved[3]; uint16_t lo[8],hi[8]; };
const uint32_t CAL_MAGIC=0x4C423630;
struct __attribute__((packed)) LogSample { uint32_t tMs; int16_t spdL,spdR,err; uint8_t mode; };
const size_t MAX_LOG_SAMPLES=4000;
LogSample logBuf[MAX_LOG_SAMPLES];
size_t logCount=0;
uint32_t runStartMs=0;
WiFiUDP udp;
uint32_t benchStartMs=0;
int32_t benchEncL=0,benchEncR=0;


void IRAM_ATTR isrL(){ portENTER_CRITICAL_ISR(&encMux); encL += digitalRead(ENC_L_B)?-1:1; portEXIT_CRITICAL_ISR(&encMux); }
void IRAM_ATTR isrR(){ portENTER_CRITICAL_ISR(&encMux); encR += digitalRead(ENC_R_B)?-1:1; portEXIT_CRITICAL_ISR(&encMux); }
void readEnc(int32_t &a,int32_t &b){portENTER_CRITICAL(&encMux);a=encL;b=encR;portEXIT_CRITICAL(&encMux);}
void IRAM_ATTR isrEcho(){
  portENTER_CRITICAL_ISR(&echoMux);
  if(echoArmed){
    if(digitalRead(ECHO_PIN)){echoRiseUs=micros();echoRising=true;}
    else if(echoRising){echoWidthUs=micros()-echoRiseUs;echoRising=false;echoDone=true;echoArmed=false;}
  }
  portEXIT_CRITICAL_ISR(&echoMux);
}
bool pwmBegin(uint8_t pin,uint8_t ch){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;return ledcAttach(pin,PWM_FREQ,PWM_BITS);
#else
  if(ledcSetup(ch,PWM_FREQ,PWM_BITS)<=0) return false;
  ledcAttachPin(pin,ch);return true;
#endif
}
void pwmWrite(uint8_t pin,uint8_t ch,uint8_t duty){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;ledcWrite(pin,duty);
#else
  (void)pin;ledcWrite(ch,duty);
#endif
}
void driveOne(uint8_t en,uint8_t ch,uint8_t a,uint8_t b,int cmd,bool inv,int previous){
  // Disable bridge before changing direction; limit changes in setMotors.
  if(motorBraked || (cmd>0)!=(previous>0) || (cmd<0)!=(previous<0)) pwmWrite(en,ch,0);
  int u=inv?-cmd:cmd;
  digitalWrite(a,u>0?HIGH:LOW); digitalWrite(b,u<0?HIGH:LOW);
  pwmWrite(en,ch,(uint8_t)abs(u));
}
int limitStep(int old,int target,int step){
  if((old>0&&target<0)||(old<0&&target>0)) target=0;
  return old+constrain(target-old,-step,step);
}
void stopMotors(){
  pwmWrite(EN_L,0,0);pwmWrite(EN_R,1,0);
  if(!pwmOK){digitalWrite(EN_L,LOW);digitalWrite(EN_R,LOW);}
  digitalWrite(IN1_L,LOW);digitalWrite(IN2_L,LOW);digitalWrite(IN1_R,LOW);digitalWrite(IN2_R,LOW);
  outL=outR=targetL=targetR=0;motorBraked=false;
}
void brakeMotors(){
  if(!pwmOK){stopMotors();return;}
  if(!motorBraked){
    pwmWrite(EN_L,0,0);pwmWrite(EN_R,1,0);
    digitalWrite(IN1_L,HIGH);digitalWrite(IN2_L,HIGH);digitalWrite(IN1_R,HIGH);digitalWrite(IN2_R,HIGH);
    pwmWrite(EN_L,0,255);pwmWrite(EN_R,1,255);
  }
  outL=outR=targetL=targetR=0;motorBraked=true;
}
void setMotors(int l,int r){
  if(!pwmOK){stopMotors();return;}
  l=constrain(l,-maxSpeed,maxSpeed);r=constrain(r,-maxSpeed,maxSpeed);
  targetL=l;targetR=r;
  // Faster deceleration than acceleration; no direct direction reversal.
  int up=constrain((int)(3UL*min(tickDtUs,(uint32_t)10000)/LOOP_US),1,15);
  int down=constrain((int)(10UL*min(tickDtUs,(uint32_t)10000)/LOOP_US),1,50);
  bool decL=abs(l)<abs(outL)||(l>0&&outL<0)||(l<0&&outL>0);
  bool decR=abs(r)<abs(outR)||(r>0&&outR<0)||(r<0&&outR>0);
  int nl=limitStep(outL,l,decL?down:up),nr=limitStep(outR,r,decR?down:up);
  driveOne(EN_L,0,IN1_L,IN2_L,nl,INVERT_LEFT,outL);
  driveOne(EN_R,1,IN1_R,IN2_R,nr,INVERT_RIGHT,outR);
  outL=nl;outR=nr;motorBraked=false;
}
void resetSync(){syncI=0;syncCorrection=0;syncActive=false;syncLastMs=0;}
void resetPid(){ integral=0;derivFilt=0;havePrev=false;lastPidUs=0;resetSync(); }
void resetCorner(){cornerCnt=0;cornerCandidateDir=0;}
void stopState(State s,const char* reason){brakeMotors();state=s;rmode=RM_FOLLOW;resetPid();resetCorner();wasLost=false;currentLine=false;Serial.println(reason);}

bool calValid(){for(int i=0;i<8;i++) if(calMax[i]>4095 || calMin[i]>4095 || (int)calMax[i]-(int)calMin[i]<200) return false;return true;}
bool saveCal(){
  CalRecord rec={};rec.magic=CAL_MAGIC;rec.black=BLACK_LINE;
  memcpy(rec.pins,SENSOR_PINS,8);memcpy(rec.lo,calMin,sizeof(calMin));memcpy(rec.hi,calMax,sizeof(calMax));
  if(!prefs.begin(NVS_NS,false))return false;
  bool ok=prefs.putBytes("cal",&rec,sizeof(rec))==sizeof(rec);prefs.end();return ok;
}
bool loadCal(){
  calibrated=false;CalRecord rec={};
  if(!prefs.begin(NVS_NS,true))return false;
  bool ok=prefs.getBytesLength("cal")==sizeof(rec) && prefs.getBytes("cal",&rec,sizeof(rec))==sizeof(rec);
  prefs.end();
  if(!ok||rec.magic!=CAL_MAGIC||rec.black!=BLACK_LINE||memcmp(rec.pins,SENSOR_PINS,8))return false;
  memcpy(calMin,rec.lo,sizeof(calMin));memcpy(calMax,rec.hi,sizeof(calMax));return calibrated=calValid();
}
float validGain(float v,float fallback,float hi){return isfinite(v)&&v>=0&&v<=hi?v:fallback;}
void loadParams(){
  if(!prefs.begin(NVS_NS,true))return;
  Kp=validGain(prefs.getFloat("kp",Kp),Kp,1);Ki=validGain(prefs.getFloat("ki",Ki),Ki,0.1f);Kd=validGain(prefs.getFloat("kd",Kd),Kd,10);
  maxSpeed=constrain(prefs.getInt("max",maxSpeed),0,255);baseSpeed=constrain(prefs.getInt("base",baseSpeed),0,maxSpeed);
  lostSpin=constrain(prefs.getInt("lost",lostSpin),0,255);turnSpeed=constrain(prefs.getInt("tspd",turnSpeed),0,255);
  turnAdvMs=constrain(prefs.getInt("tadv",turnAdvMs),0,500);fullTurnMs=constrain(prefs.getInt("tfull",fullTurnMs),200,(int)SEARCH_HARD_MAX_MS);
  fullTurnCnt=constrain(prefs.getInt("tcnt",fullTurnCnt),0,1000000);cornerMode=prefs.getBool("corner",cornerMode);
  approachPWM=constrain(prefs.getInt("approach",approachPWM),0,255);advancePWM=constrain(prefs.getInt("advpwm",advancePWM),0,255);
  finePWM=constrain(prefs.getInt("fine",finePWM),0,255);trimPWM=constrain(prefs.getInt("trim",trimPWM),-25,25);
  lineZero=constrain(prefs.getInt("zero",lineZero),3200,3800);
  advanceCnt=constrain(prefs.getInt("advcnt",advanceCnt),0,1000000);turn90Cnt=constrain(prefs.getInt("turncnt",turn90Cnt),0,1000000);
  syncKp=validGain(prefs.getFloat("synckp",syncKp),0,2);syncKi=validGain(prefs.getFloat("syncki",syncKi),0,2);prefs.end();
}
void saveParams(){
  if(!prefs.begin(NVS_NS,false)){Serial.println("NVS ERROR");return;}
  prefs.putFloat("kp",Kp);prefs.putFloat("ki",Ki);prefs.putFloat("kd",Kd);prefs.putInt("base",baseSpeed);prefs.putInt("max",maxSpeed);
  prefs.putInt("lost",lostSpin);prefs.putInt("tspd",turnSpeed);prefs.putInt("tadv",turnAdvMs);prefs.putInt("tfull",fullTurnMs);prefs.putInt("tcnt",fullTurnCnt);prefs.putBool("corner",cornerMode);
  prefs.putInt("approach",approachPWM);prefs.putInt("advpwm",advancePWM);prefs.putInt("fine",finePWM);prefs.putInt("trim",trimPWM);prefs.putInt("zero",lineZero);
  prefs.putInt("advcnt",advanceCnt);prefs.putInt("turncnt",turn90Cnt);prefs.putFloat("synckp",syncKp);prefs.putFloat("syncki",syncKi);prefs.end();
  Serial.println("SAVE requested");
}
uint16_t readOne(uint8_t pin){analogRead(pin);return (analogRead(pin)+analogRead(pin))/2;}
void readRaw(uint16_t* a){for(int i=0;i<8;i++)a[i]=readOne(SENSOR_PINS[i]);}
void startCalibration(){
  brakeMotors();calibrated=false;
  // Invalidate old record before beginning so failed/cancelled calibration cannot auto-reuse it.
  if(!prefs.begin(NVS_NS,false)){stopState(FAULT,"NVS ERROR");return;}
  bool ok=!prefs.isKey("cal")||prefs.remove("cal");prefs.end();if(!ok){stopState(FAULT,"NVS REMOVE ERROR");return;}
  for(int i=0;i<8;i++){calMin[i]=4095;calMax[i]=0;}
  calStartMs=millis();state=CALIBRATING;Serial.println("CAL: move sensor across black and white for 4s; x cancels");
}
void calibrationStep(uint32_t now){
  uint16_t a[8];readRaw(a);for(int i=0;i<8;i++){calMin[i]=min(calMin[i],a[i]);calMax[i]=max(calMax[i],a[i]);}
  if(now-calStartMs<CAL_TIME_MS)return;
  for(int i=0;i<8;i++)Serial.printf("S%d min=%u max=%u\n",i+1,calMin[i],calMax[i]);
  calibrated=calValid()&&saveCal();stopState(calibrated?IDLE:FAULT,calibrated?"CAL OK; s to arm":"CAL FAILED; retry c");
}
int readPosition(bool &onLine){
  uint16_t raw[8];readRaw(raw);int32_t sum=0,wsum=0;int maxV=0;
  for(int i=0;i<8;i++){
    int range=(int)calMax[i]-calMin[i];int v=0;
    if(range>=200){v=constrain(((int)raw[i]-calMin[i])*1000/range,0,1000);if(!BLACK_LINE)v=1000-v;}
    sv[i]=v;maxV=max(maxV,v);if(v>150){sum+=v;wsum+=(int32_t)v*i*1000;}
  }
  onLine=maxV>250 && sum>0;return onLine?wsum/sum:3500;
}
// A single contiguous group including a central sensor; not all-black or split branches.
bool centeredLine(bool onLine,int err){
  if(!onLine||abs(err)>=CENTER_BAND||!(sv[3]>400||sv[4]>400))return false;
  int active=0,groups=0;bool prev=false;
  for(int i=0;i<8;i++){bool hit=sv[i]>400;if(hit){active++;if(!prev)groups++;}prev=hit;}
  return active<=4&&groups==1;
}

// Single connected line, rather than a broad junction or separated branches.
bool singleLine(){
  int active=0,groups=0;bool prev=false;
  for(int i=0;i<8;i++){bool hit=sv[i]>400;if(hit){active++;if(!prev)groups++;}prev=hit;}
  return active>0&&active<=4&&groups==1;
}
int64_t meanTravel(int32_t l0,int32_t r0){
  int32_t l,r;readEnc(l,r);int64_t a=(int64_t)l-l0,b=(int64_t)r-r0;
  return ((a<0?-a:a)+(b<0?-b:b))/2;
}
// Bounded straight-only PI correction in PWM units, based on |counts/50ms|.
// It is not a full wheel velocity cascade; only enabled explicitly after testing.
void updateSync(uint32_t now,bool eligible){
  if(!eligible||(syncKp==0&&syncKi==0)){resetSync();return;}
  if(!syncActive){syncActive=true;syncLastMs=lastEncMs;return;}
  if(lastEncMs==syncLastMs)return;
  uint32_t dt=lastEncMs-syncLastMs;syncLastMs=lastEncMs;
  if(now-lastEncMs>100||dt>150||speedL==0||speedR==0){resetSync();return;}
  float e=(float)abs(speedR)-(float)abs(speedL);
  float next=constrain(syncI+e*(dt/1000.0f),-100.0f,100.0f);
  float raw=syncKp*e+syncKi*next;
  if(fabsf(raw)<=15||raw*e<0)syncI=next;
  syncCorrection=constrain(syncKp*e+syncKi*syncI,-15.0f,15.0f);
}
void usReset(){
  portENTER_CRITICAL(&echoMux);echoDone=false;echoRising=false;echoArmed=false;portEXIT_CRITICAL(&echoMux);
  usWaiting=false;sonarBad=sonarClear=0;sonarValid=false;obstacle=false;distCm=-1;lastValidMs=0;usTrigMs=millis()-60;
}
void usProcess(bool valid,float cm,uint32_t now){
  if(!valid){sonarValid=false;distCm=-1;sonarClear=0;if(sonarBad<255)sonarBad++;return;}
  sonarValid=true;lastValidMs=now;distCm=cm;sonarBad=0;
  // Stop on a single near echo. Resume only on three consecutive clear echoes.
  if(cm<=OBSTACLE_CM){obstacle=true;sonarClear=0;}
  else if(cm>=CLEAR_CM){if(sonarClear<3)sonarClear++;if(sonarClear>=3)obstacle=false;}
  else sonarClear=0;
}
void updateObstacle(uint32_t now){
  if(usWaiting){
    uint32_t w=0;bool done=false;
    portENTER_CRITICAL(&echoMux);
    if(echoDone){done=true;w=echoWidthUs;echoDone=false;}
    else if(now-usTrigMs>30){echoArmed=false;echoRising=false;}
    portEXIT_CRITICAL(&echoMux);
    if(done){usWaiting=false;usProcess(w>=120&&w<=23200,w*0.0343f/2.0f,now);}
    else if(now-usTrigMs>30){usWaiting=false;usProcess(false,0,now);}
  }else if(now-usTrigMs>=60){
    portENTER_CRITICAL(&echoMux);echoDone=false;echoRising=false;echoArmed=true;portEXIT_CRITICAL(&echoMux);
    usWaiting=true;usTrigMs=now;digitalWrite(TRIG_PIN,HIGH);delayMicroseconds(10);digitalWrite(TRIG_PIN,LOW);
  }
}
void startSettle(uint32_t now){
  // Brake old turning command immediately; resume with the ramp on next tick.
  brakeMotors();rmode=RM_FOLLOW;settleStartMs=now;holdStartMs=now;holdActive=true;
  wasLost=false;resetCorner();centerCnt=0;leaveCnt=0;resetPid();
}
void startSearch(uint32_t now,bool keepDir){
  brakeMotors();rmode=RM_SEARCH;modeStartMs=now;if(!keepDir)turnDir=lastSide<0?-1:1;
  wasLost=false;centerCnt=0;resetCorner();resetPid();readEnc(searchEncL0,searchEncR0);
}
bool searchFullTurnDone(uint32_t now){
  uint32_t elapsed=now-modeStartMs;
  if(elapsed>=SEARCH_HARD_MAX_MS)return true;
  if(fullTurnCnt<=0)return elapsed>=(uint32_t)fullTurnMs;
  // Independent watchdog even if encoder target is never reached.
  uint32_t timeout=min((uint32_t)fullTurnMs*2U,SEARCH_HARD_MAX_MS);
  if(elapsed>=timeout)return true;
  int32_t a,b;readEnc(a,b);
  int64_t da=(int64_t)a-searchEncL0,db=(int64_t)b-searchEncR0;
  if(da<0)da=-da;
  if(db<0)db=-db;
  return (da+db)/2>=fullTurnCnt;
}
void startRun(){
  if(!pwmOK||!calibrated||!calValid()){stopState(FAULT,"RUN denied: PWM/calibration");return;}
  if(maxSpeed<=0||baseSpeed<=0){stopState(IDLE,"RUN denied: speed zero");return;}
  brakeMotors();resetPid();resetCorner();usReset();lastSide=0;holdActive=false;wasLost=false;rmode=RM_FOLLOW;
  armStartMs=millis();state=ARMING;Serial.println("ARMING: waiting for line and valid clear sonar");
}
void beginMotion(uint32_t now){
  readEnc(lastEncL,lastEncR);lastEncMs=now;speedL=speedR=0;logCount=0;runStartMs=now;
  startSettle(now);holdActive=false;state=RUNNING;Serial.println("RUN");
}
void controlStep(uint32_t now,uint32_t nowUs){
  int pos=readPosition(currentLine);currentErr=currentLine?pos-lineZero:0;
  bool center=centeredLine(currentLine,currentErr);
  if(state==ARMING){
    if(now-armStartMs>=ARM_TIMEOUT_MS){stopState(FAULT,"ARM timeout: check line/sonar");return;}
    if(sonarValid&&!obstacle&&sonarClear>=3&&center)beginMotion(now);
    return;
  }
  if(sonarBad>=3 || now-lastValidMs>SONAR_STALE_MS){stopState(FAULT,"SONAR invalid/stale; x then s to retry");return;}
  if(obstacle||!sonarValid){
    if(state!=OBSTACLE_STOP){brakeMotors();resetPid();resetCorner();state=OBSTACLE_STOP;Serial.println("OBSTACLE/invalid echo: STOP; x then s to resume");}
    return;
  }
  // Deliberate manual resume: no unexpected restart when obstacle disappears.
  if(state==OBSTACLE_STOP)return;
  if(rmode==RM_ADVANCE){
    uint32_t elapsed=now-modeStartMs;
    bool done=advanceCnt>0?meanTravel(maneuverL0,maneuverR0)>=advanceCnt:elapsed>=(uint32_t)turnAdvMs;
    if(!done&&elapsed>=ADV_MAX_MS){stopState(FAULT,"ADVANCE timeout; check encoder/geometry");return;}
    if(done){
      // Always brake before deciding whether the straight exit is present.
      brakeMotors();rmode=RM_BRAKE;modeStartMs=now;centerCnt=0;return;
    }
    setMotors(advancePWM,advancePWM);return;
  }
  if(rmode==RM_BRAKE){
    brakeMotors();
    if(center){if(centerCnt<CENTER_CONFIRM)centerCnt++;}else centerCnt=0;
    if(now-modeStartMs<BRAKE_MS)return;
    if(centerCnt>=CENTER_CONFIRM){startSettle(now);return;} // straight priority at junction
    rmode=RM_TURN;modeStartMs=turnStartMs=now;centerCnt=leaveCnt=0;leftOldLine=false;
    alignAttempts=0;fineMissing=false;readEnc(maneuverL0,maneuverR0);return;
  }
  if(rmode==RM_TURN||rmode==RM_FINE||rmode==RM_ALIGN_BRAKE){
    uint32_t elapsed=now-turnStartMs;
    int64_t travel=meanTravel(maneuverL0,maneuverR0);
    // A failed known corner stops, rather than starting an uncontrolled full revolution.
    if(elapsed>=TURN_MAX_MS||(turn90Cnt>0&&travel*10>(int64_t)turn90Cnt*14)){
      stopState(FAULT,"TURN timeout/angle limit; inspect corner");return;
    }
    bool nearNew=currentLine&&singleLine()&&abs(currentErr)<1600;
    bool plausibleAngle=turn90Cnt<=0||travel*10>=(int64_t)turn90Cnt*5;
    if(rmode==RM_ALIGN_BRAKE){
      brakeMotors();if(center){if(centerCnt<CENTER_CONFIRM)centerCnt++;}else centerCnt=0;
      if(now-modeStartMs<ALIGN_BRAKE_MS)return;
      if(centerCnt>=CENTER_CONFIRM){startSettle(now);return;}
      if(!nearNew||++alignAttempts>2){stopState(FAULT,"TURN alignment lost; reposition");return;}
      rmode=RM_FINE;fineMissing=false;return;
    }
    if(!leftOldLine){if(!center){if(++leaveCnt>=CENTER_CONFIRM)leftOldLine=true;}else leaveCnt=0;}
    if(rmode==RM_TURN){
      if(leftOldLine&&elapsed>=60&&plausibleAngle&&nearNew){
        rmode=RM_FINE;fineMissing=false;centerCnt=0;
      }else{
        int coarse=turnSpeed;
        if(turn90Cnt>0&&travel*10>=(int64_t)turn90Cnt*7)coarse=min(coarse,finePWM);
        setMotors(turnDir*coarse,-turnDir*coarse);return;
      }
    }
    // Fine phase: brake as soon as the center is seen; confirm while stopped.
    if(center&&plausibleAngle){brakeMotors();rmode=RM_ALIGN_BRAKE;modeStartMs=now;centerCnt=0;return;}
    if(!nearNew){
      if(!fineMissing){fineMissing=true;fineLostSince=now;}
      if(now-fineLostSince>=FINE_LOST_MS){stopState(FAULT,"Fine turn lost line");return;}
      int slow=min(finePWM,turnSpeed);setMotors(turnDir*slow,-turnDir*slow);return;
    }
    fineMissing=false;
    // Correct small overshoot based on observed line side, with reversal slew limit.
    int d=currentErr<0?-1:1;int slow=min(finePWM,turnSpeed);setMotors(d*slow,-d*slow);return;
  }
  if(rmode==RM_SEARCH){
    if(searchFullTurnDone(now)){stopState(IDLE,"SEARCH limit; STOP");return;}
    if(center){brakeMotors();if(++centerCnt>=CENTER_CONFIRM)startSettle(now);return;}else centerCnt=0;
    int spd=currentLine?min(finePWM,lostSpin):lostSpin;setMotors(turnDir*spd,-turnDir*spd);return;
  }
  // FOLLOW: remember direction only on valid FOLLOW data, not during rotation.
  if(!currentLine){
    resetCorner();resetPid();if(!wasLost){wasLost=true;lostSinceMs=now;}
    if(now-lostSinceMs>=LOST_GRACE_MS){startSearch(now,false);return;}
    int creep=min(baseSpeed/3,55);int bias=lastSide*min(creep/2,20);setMotors(creep+bias,creep-bias);return;
  }
  wasLost=false;
  if(currentErr<-400)lastSide=-1;else if(currentErr>400)lastSide=1;
  if(holdActive&&now-holdStartMs>=CORNER_HOLD_MS)holdActive=false;
  bool edgeWarning=sv[0]>EARLY_EDGE||sv[7]>EARLY_EDGE||abs(currentErr)>1400||!singleLine();
  if(cornerMode&&!holdActive){
    bool leftHit=sv[0]>CORNER_TH&&sv[1]>CORNER_TH,rightHit=sv[6]>CORNER_TH&&sv[7]>CORNER_TH;
    bool leftAny=sv[0]>CORNER_TH||sv[1]>CORNER_TH,rightAny=sv[6]>CORNER_TH||sv[7]>CORNER_TH;
    // Require extension toward inner sensors, not only a displaced narrow line.
    int candidate=(leftHit&&!rightAny&&sv[2]>400)?-1:((rightHit&&!leftAny&&sv[5]>400)?1:0);
    if(!candidate)resetCorner();else{
      if(candidate!=cornerCandidateDir){cornerCandidateDir=candidate;cornerCnt=1;}else cornerCnt++;
      if(cornerCnt>=CORNER_CONFIRM){
        turnDir=candidate;lastSide=candidate;resetCorner();rmode=RM_ADVANCE;modeStartMs=now;
        readEnc(maneuverL0,maneuverR0);resetPid();setMotors(advancePWM,advancePWM);return;
      }
    }
  }else resetCorner();
  // PD on nominal-sample units; smooth measurement and reduce speed before saturation.
  float err=abs(currentErr)<=FOLLOW_DEADBAND?0.0f:(float)currentErr;
  float ratio=1.0f,deriv=0;
  if(havePrev){ratio=constrain((nowUs-lastPidUs)/(float)LOOP_US,0.5f,5.0f);deriv=constrain((err-lastErr)/ratio,-220.0f,220.0f);}
  else{havePrev=true;derivFilt=0;}
  lastPidUs=nowUs;lastErr=err;derivFilt+=DERIV_ALPHA*(deriv-derivFilt);
  float risk=max(fabsf(err)/3500.0f,min(fabsf(derivFilt)/180.0f,1.0f));
  float slow=1.0f-0.65f*constrain(risk,0.0f,1.0f);
  float ramp=0.55f+0.45f*min((now-settleStartMs)/(float)SETTLE_MS,1.0f);
  int base=constrain((int)(baseSpeed*slow*ramp),0,maxSpeed);
  if(edgeWarning)base=min(base,approachPWM);
  // Straight-only synchronization uses the most recent encoder sample; no gain guessed.
  bool straight=singleLine()&&!edgeWarning&&abs(currentErr)<200&&fabsf(derivFilt)<35&&outL>=40&&outR>=40;
  updateSync(now,straight);
  float bias=trimPWM+syncCorrection;
  // No wheel reversal in FOLLOW; never raise base to accommodate a large correction.
  float limit=(float)min(base,maxSpeed/2);
  float proposed=constrain(integral+err*ratio,-12000.0f,12000.0f);
  float raw=Kp*err+Ki*proposed+Kd*derivFilt+bias;
  if(Ki==0)integral=0;
  else if(!edgeWarning&&(fabsf(raw)<=limit||raw*err<0))integral=proposed;
  float corr=constrain(Kp*err+Ki*integral+Kd*derivFilt+bias,-limit,limit);
  int mag=abs((int)corr);base=min(base,maxSpeed-mag);
  setMotors(constrain(base+(int)corr,0,maxSpeed),constrain(base-(int)corr,0,maxSpeed));
}

void recordEncoder(uint32_t now){
  uint32_t dt=now-lastEncMs;if(dt<50)return;
  int32_t a,b;readEnc(a,b);
  // Report equivalent counts per 50 ms, preserving the existing Python receiver's units.
  speedL=(int32_t)(((int64_t)a-lastEncL)*50/dt);speedR=(int32_t)(((int64_t)b-lastEncR)*50/dt);
  lastEncL=a;lastEncR=b;lastEncMs=now;
  if(state==RUNNING&&logCount<MAX_LOG_SAMPLES){
    LogSample &s=logBuf[logCount++];s.tMs=now-runStartMs;
    s.spdL=(int16_t)constrain((int)speedL,-32768,32767);s.spdR=(int16_t)constrain((int)speedR,-32768,32767);
    s.err=(int16_t)(currentLine?currentErr:0);s.mode=(uint8_t)rmode;
  }
}
void dumpLogOverUdp(){
  if(state!=IDLE){Serial.println("STOP with x first");return;}
  if(!logCount||!strlen(WIFI_SSID)){Serial.println("No log or WiFiConfig.h not configured");return;}
  brakeMotors();state=UDP_DUMP;
  // Blocking network operations are allowed ONLY with the bridge in its stopped state.
  WiFi.mode(WIFI_STA);WiFi.begin(WIFI_SSID,WIFI_PASS);
  uint32_t start=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-start<15000)delay(100);
  if(WiFi.status()==WL_CONNECTED){
    udp.begin(PC_PORT);
    for(int k=0;k<3;k++){udp.beginPacket(PC_IP,PC_PORT);udp.printf("START,%u\n",(unsigned)logCount);udp.endPacket();delay(20);}
    size_t failures=0;
    for(size_t i=0;i<logCount;i++){
      const LogSample &s=logBuf[i];
      if(!udp.beginPacket(PC_IP,PC_PORT)){failures++;continue;}
      udp.printf("DATA,%u,%lu,%d,%d,%d,%u\n",(unsigned)i,(unsigned long)s.tMs,(int)s.spdL,(int)s.spdR,(int)s.err,(unsigned)s.mode);
      if(!udp.endPacket())failures++;
      delay(3);
    }
    for(int k=0;k<3;k++){udp.beginPacket(PC_IP,PC_PORT);udp.printf("END,%u\n",(unsigned)logCount);udp.endPacket();delay(20);}
    Serial.printf("UDP local send failures: %u; delivery not guaranteed\n",(unsigned)failures);
    udp.stop();
  }else Serial.println("WiFi connection failed; log retained");
  WiFi.disconnect(true);WiFi.mode(WIFI_OFF);state=IDLE;
  // Drop buffered commands accumulated during export. Require fresh operator action.
  while(Serial.available())Serial.read();
}
void printParams(){
  Serial.printf("Kp=%.4f Ki=%.5f Kd=%.3f base=%d max=%d lost=%d corner=%d tspd=%d tadv=%d tfull=%d tcnt=%d\n",Kp,Ki,Kd,baseSpeed,maxSpeed,lostSpin,(int)cornerMode,turnSpeed,turnAdvMs,fullTurnMs,fullTurnCnt);
  Serial.printf("approach=%d advpwm=%d fine=%d trim=%d zero=%d advcnt=%d turncnt=%d synckp=%.3f syncki=%.3f\n",approachPWM,advancePWM,finePWM,trimPWM,lineZero,advanceCnt,turn90Cnt,syncKp,syncKi);
}
void startBench(State s){
  if(state!=IDLE||!pwmOK){Serial.println("Bench denied; x first");return;}
  brakeMotors();benchStartMs=millis();readEnc(benchEncL,benchEncR);state=s;
  Serial.println("BENCH: wheels lifted only; x cancels; sonar not used");
}
void benchStep(uint32_t now){
  uint32_t elapsed=now-benchStartMs;
  if(state==BENCH_TEST){
    if(elapsed>=2000){stopState(IDLE,"TEST DONE");return;}
    if(elapsed<1000)setMotors(150,0);else setMotors(0,150);
  }else if(state==BENCH_SPIN){
    if(elapsed>=(uint32_t)fullTurnMs){
      int32_t a,b;readEnc(a,b);stopState(IDLE,"SPIN DONE");
      long dl=(long)((int64_t)a-benchEncL),dr=(long)((int64_t)b-benchEncR);
      Serial.printf("Encoder L=%ld R=%ld; mean abs=%ld (calibrate on actual floor separately)\n",dl,dr,(labs(dl)+labs(dr))/2);return;
    }
    setMotors(lostSpin,-lostSpin);
  }
}
void processCommand(char* buf){
  for(char *p=buf;*p;p++)*p=tolower((unsigned char)*p);
  if(!strcmp(buf,"x")){stopState(IDLE,"STOP");return;}
  if(!strcmp(buf,"p")){printParams();return;}
  if(!strcmp(buf,"log")){Serial.printf("Log samples %u/%u\n",(unsigned)logCount,(unsigned)MAX_LOG_SAMPLES);return;}
  if(!strcmp(buf,"dbg")){debugOn=!debugOn;return;}
  if(!strcmp(buf,"clear")&&state!=IDLE)stopState(IDLE,"STOP before clear");
  // Configuration and maintenance only while stopped. Never reset a live safety supervisor with 's'.
  if(state!=IDLE){Serial.println("Command denied; x first");return;}
  if(!strcmp(buf,"c")){startCalibration();return;}
  if(!strcmp(buf,"s")){startRun();return;}
  if(!strcmp(buf,"save")){saveParams();return;}
  if(!strcmp(buf,"clear")){
    brakeMotors();calibrated=false;bool ok=prefs.begin(NVS_NS,false);
    if(ok){ok=prefs.clear();prefs.end();}
    stopState(ok?IDLE:FAULT,ok?"CLEARED; reboot for default gains; c required":"NVS CLEAR ERROR");return;
  }
  if(!strcmp(buf,"test")){startBench(BENCH_TEST);return;}
  if(!strcmp(buf,"spin")){startBench(BENCH_SPIN);return;}
  if(!strcmp(buf,"udp")){dumpLogOverUdp();return;}
  if(!strcmp(buf,"raw")){
    uint16_t raw[8];readRaw(raw);for(int i=0;i<8;i++)Serial.printf("%4u ",raw[i]);Serial.println();return;
  }
  char key[16],extra;float v;
  if(sscanf(buf,"%15s %f %c",key,&v,&extra)!=2||!isfinite(v)||v< -25||v>1000000){Serial.println("Invalid command/value");return;}
  if(!strcmp(key,"trim")&&v>=-25&&v<=25)trimPWM=(int)v;
  else if(v<0){Serial.println("Negative value not allowed except trim");return;}
  else if(!strcmp(key,"kp")&&v<=1)Kp=v;
  else if(!strcmp(key,"ki")&&v<=0.1f)Ki=v;
  else if(!strcmp(key,"kd")&&v<=10)Kd=v;
  else if(!strcmp(key,"base")&&v<=255)baseSpeed=(int)v;
  else if(!strcmp(key,"max")&&v<=255)maxSpeed=(int)v;
  else if(!strcmp(key,"lost")&&v<=255)lostSpin=(int)v;
  else if(!strcmp(key,"tspd")&&v<=255)turnSpeed=(int)v;
  else if(!strcmp(key,"tadv")&&v<=500)turnAdvMs=(int)v;
  else if(!strcmp(key,"corner")&&(v==0||v==1))cornerMode=v==1;
  else if(!strcmp(key,"tfull")&&v>=200&&v<=SEARCH_HARD_MAX_MS)fullTurnMs=(int)v;
  else if(!strcmp(key,"tcnt"))fullTurnCnt=(int)v;
  else if(!strcmp(key,"approach")&&v>=30&&v<=255)approachPWM=(int)v;
  else if(!strcmp(key,"advpwm")&&v>=30&&v<=255)advancePWM=(int)v;
  else if(!strcmp(key,"fine")&&v>=30&&v<=255)finePWM=(int)v;
  else if(!strcmp(key,"zero")&&v>=3200&&v<=3800)lineZero=(int)v;
  else if(!strcmp(key,"advcnt"))advanceCnt=(int)v;
  else if(!strcmp(key,"turncnt"))turn90Cnt=(int)v;
  else if(!strcmp(key,"synckp")&&v<=2)syncKp=v;
  else if(!strcmp(key,"syncki")&&v<=2)syncKi=v;
  else{Serial.println("Unknown command or out of range");return;}
  baseSpeed=min(baseSpeed,maxSpeed);printParams();
}
void handleSerial(){
  static char buf[64];static uint8_t len=0;static bool overflow=false;
  // Limit serial processing per pass so continuous data cannot starve control indefinitely.
  for(int n=0;n<64&&Serial.available();n++){
    char c=(char)Serial.read();
    if(c=='\r'||c=='\n'){
      if(!overflow&&len){buf[len]=0;processCommand(buf);}
      else if(overflow)Serial.println("Command too long");
      len=0;overflow=false;
    }else if(!overflow){if(len<sizeof(buf)-1)buf[len++]=c;else overflow=true;}
  }
}
void setup(){
  // Set bridge pins low before Wi-Fi or serial initialization.
  pinMode(EN_L,OUTPUT);pinMode(EN_R,OUTPUT);digitalWrite(EN_L,LOW);digitalWrite(EN_R,LOW);
  pinMode(IN1_L,OUTPUT);pinMode(IN2_L,OUTPUT);pinMode(IN1_R,OUTPUT);pinMode(IN2_R,OUTPUT);
  digitalWrite(IN1_L,LOW);digitalWrite(IN2_L,LOW);digitalWrite(IN1_R,LOW);digitalWrite(IN2_R,LOW);
  Serial.begin(115200);bool leftOK=pwmBegin(EN_L,0),rightOK=pwmBegin(EN_R,1);pwmOK=leftOK&&rightOK;stopMotors();
  WiFi.mode(WIFI_OFF);pinMode(DUMP_BTN,INPUT_PULLUP);
  analogReadResolution(12);analogSetAttenuation(ADC_11db);for(int i=0;i<8;i++)pinMode(SENSOR_PINS[i],INPUT);
  pinMode(TRIG_PIN,OUTPUT);digitalWrite(TRIG_PIN,LOW);pinMode(ECHO_PIN,INPUT);attachInterrupt(digitalPinToInterrupt(ECHO_PIN),isrEcho,CHANGE);
  pinMode(ENC_L_A,INPUT_PULLUP);pinMode(ENC_L_B,INPUT_PULLUP);pinMode(ENC_R_A,INPUT_PULLUP);pinMode(ENC_R_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A),isrL,RISING);attachInterrupt(digitalPinToInterrupt(ENC_R_A),isrR,RISING);
  loadParams();loadCal();readEnc(lastEncL,lastEncR);lastEncMs=millis();lastTickUs=micros();
  if(!pwmOK){stopState(FAULT,"PWM init failed");return;}
  Serial.println(calibrated?"V6.1 READY; s to arm":"V6.1 READY; c to calibrate, s to arm");printParams();
}
void loop(){
  static uint32_t lastDbg=0,btnStart=0;static bool btnTracking=false,btnConsumed=false;
  handleSerial();
  uint32_t now=millis();
  if(state==IDLE&&digitalRead(DUMP_BTN)==LOW){
    if(!btnTracking){btnTracking=true;btnStart=now;}
    if(!btnConsumed&&now-btnStart>=1000){btnConsumed=true;dumpLogOverUdp();return;}
  }else{btnTracking=false;if(digitalRead(DUMP_BTN)==HIGH)btnConsumed=false;}
  uint32_t us=micros();if(us-lastTickUs<LOOP_US)return;
  tickDtUs=us-lastTickUs;lastTickUs=us;if(tickDtUs>LOOP_US*2)overrunCount++;
  now=millis();
  if(state==CALIBRATING)calibrationStep(now);
  else if(state==BENCH_TEST||state==BENCH_SPIN)benchStep(now);
  else if(state==ARMING||state==RUNNING||state==OBSTACLE_STOP){updateObstacle(now);controlStep(now,us);}
  else if(debugOn&&calibrated){int pos=readPosition(currentLine);currentErr=currentLine?pos-lineZero:0;}
  recordEncoder(now);
  if(debugOn&&now-lastDbg>=100){
    lastDbg=now;
    for(int i=0;i<8;i++)Serial.printf("%4u ",sv[i]);
    Serial.printf("| state=%d m=%d line=%d err=%d dist=%.1f pwm=%d,%d target=%d,%d enc=%ld,%ld dt=%lu over=%lu sync=%.1f\n",(int)state,(int)rmode,(int)currentLine,currentErr,distCm,outL,outR,targetL,targetR,(long)speedL,(long)speedR,(unsigned long)tickDtUs,(unsigned long)overrunCount,syncCorrection);
  }
}
