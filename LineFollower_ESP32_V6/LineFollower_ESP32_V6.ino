/* V6 review candidate, based on V5 + UDP (blob 2749562a84fec79b41b7bec480b14cda1fca5b5f).
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
const uint32_t LOST_GRACE_MS=40, SETTLE_MS=300, TURN_MAX_MS=900, CORNER_HOLD_MS=250;
const uint32_t SEARCH_HARD_MAX_MS=5000, ARM_TIMEOUT_MS=2500, SONAR_STALE_MS=250;
const int CENTER_BAND=700, CORNER_TH=600, TURN_FWD_SPEED=110;
const uint8_t CORNER_CONFIRM=2, CENTER_CONFIRM=4;
const float OBSTACLE_CM=15.0f, CLEAR_CM=20.0f;
const float DERIV_ALPHA=0.4f;
float Kp=0.05f, Ki=0.0f, Kd=1.0f;
int baseSpeed=170, maxSpeed=230, minSpeed=-80, lostSpin=140, turnSpeed=230;
int turnAdvMs=40, fullTurnMs=1600, fullTurnCnt=0;
bool cornerMode=true;
enum State { IDLE, CALIBRATING, ARMING, RUNNING, OBSTACLE_STOP, FAULT, BENCH_TEST, BENCH_SPIN, UDP_DUMP };
enum RunMode { RM_FOLLOW, RM_ADVANCE, RM_TURN, RM_SEARCH };
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
const char* NVS_NS="linebot6";
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
  outL=outR=0;motorBraked=false;
}
void brakeMotors(){
  if(!pwmOK){stopMotors();return;}
  if(!motorBraked){
    pwmWrite(EN_L,0,0);pwmWrite(EN_R,1,0);
    digitalWrite(IN1_L,HIGH);digitalWrite(IN2_L,HIGH);digitalWrite(IN1_R,HIGH);digitalWrite(IN2_R,HIGH);
    pwmWrite(EN_L,0,255);pwmWrite(EN_R,1,255);
  }
  outL=outR=0;motorBraked=true;
}
void setMotors(int l,int r){
  if(!pwmOK){stopMotors();return;}
  l=constrain(l,-maxSpeed,maxSpeed);r=constrain(r,-maxSpeed,maxSpeed);
  // 6 PWM per nominal 2 ms; cap elapsed-time catch-up. Tune on real vehicle.
  int step=constrain((int)(6UL*min(tickDtUs,(uint32_t)10000)/LOOP_US),1,30);
  int nl=limitStep(outL,l,step),nr=limitStep(outR,r,step);
  driveOne(EN_L,0,IN1_L,IN2_L,nl,INVERT_LEFT,outL);
  driveOne(EN_R,1,IN1_R,IN2_R,nr,INVERT_RIGHT,outR);
  outL=nl;outR=nr;motorBraked=false;
}
void resetPid(){ integral=0;derivFilt=0;havePrev=false;lastPidUs=0; }
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
  fullTurnCnt=constrain(prefs.getInt("tcnt",fullTurnCnt),0,1000000);cornerMode=prefs.getBool("corner",cornerMode);prefs.end();
}
void saveParams(){
  if(!prefs.begin(NVS_NS,false)){Serial.println("NVS ERROR");return;}
  prefs.putFloat("kp",Kp);prefs.putFloat("ki",Ki);prefs.putFloat("kd",Kd);prefs.putInt("base",baseSpeed);prefs.putInt("max",maxSpeed);
  prefs.putInt("lost",lostSpin);prefs.putInt("tspd",turnSpeed);prefs.putInt("tadv",turnAdvMs);prefs.putInt("tfull",fullTurnMs);prefs.putInt("tcnt",fullTurnCnt);prefs.putBool("corner",cornerMode);prefs.end();
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
  // Zero old turning command immediately; resume with the ramp on next tick.
  stopMotors();rmode=RM_FOLLOW;settleStartMs=now;holdStartMs=now;holdActive=true;
  wasLost=false;resetCorner();centerCnt=0;leaveCnt=0;resetPid();
}
void startSearch(uint32_t now,bool keepDir){
  rmode=RM_SEARCH;modeStartMs=now;if(!keepDir)turnDir=lastSide<0?-1:1;
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
  int pos=readPosition(currentLine);currentErr=pos-3500;
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
    if(now-modeStartMs>=(uint32_t)turnAdvMs){
      if(center){startSettle(now);return;}
      rmode=RM_TURN;modeStartMs=now;centerCnt=leaveCnt=0;leftOldLine=false;
      // If the center already left the old line, count it on subsequent samples.
      setMotors(turnDir*turnSpeed,-turnDir*turnSpeed);return;
    }
    setMotors(TURN_FWD_SPEED,TURN_FWD_SPEED);return;
  }
  if(rmode==RM_TURN){
    uint32_t elapsed=now-modeStartMs;
    if(elapsed>=TURN_MAX_MS){startSearch(now,true);return;}
    if(!leftOldLine){if(!center){if(++leaveCnt>=CENTER_CONFIRM)leftOldLine=true;}else leaveCnt=0;}
    if(leftOldLine&&elapsed>=60&&center){if(++centerCnt>=CENTER_CONFIRM){startSettle(now);return;}}else centerCnt=0;
    int spd=(currentLine&&abs(currentErr)<1800)?turnSpeed*6/10:turnSpeed;
    setMotors(turnDir*spd,-turnDir*spd);return;
  }
  if(rmode==RM_SEARCH){
    if(searchFullTurnDone(now)){stopState(IDLE,"SEARCH limit; STOP");return;}
    if(center){if(++centerCnt>=CENTER_CONFIRM){startSettle(now);return;}}else centerCnt=0;
    int spd=currentLine?lostSpin*6/10:lostSpin;setMotors(turnDir*spd,-turnDir*spd);return;
  }
  // FOLLOW: remember direction only on valid FOLLOW data, not during rotation.
  if(!currentLine){
    resetCorner();resetPid();if(!wasLost){wasLost=true;lostSinceMs=now;}
    if(now-lostSinceMs>=LOST_GRACE_MS){startSearch(now,false);return;}
    int creep=min(baseSpeed/3,65);int bias=lastSide*min(creep/2,25);setMotors(creep+bias,creep-bias);return;
  }
  wasLost=false;
  if(currentErr<-400)lastSide=-1;else if(currentErr>400)lastSide=1;
  if(holdActive&&now-holdStartMs>=CORNER_HOLD_MS)holdActive=false;
  if(cornerMode&&!holdActive){
    bool leftHit=sv[0]>CORNER_TH&&sv[1]>CORNER_TH,rightHit=sv[6]>CORNER_TH&&sv[7]>CORNER_TH;
    bool leftAny=sv[0]>CORNER_TH||sv[1]>CORNER_TH,rightAny=sv[6]>CORNER_TH||sv[7]>CORNER_TH;
    int candidate=(leftHit&&!rightAny)?-1:((rightHit&&!leftAny)?1:0);
    if(!candidate)resetCorner();else{
      if(candidate!=cornerCandidateDir){cornerCandidateDir=candidate;cornerCnt=1;}else cornerCnt++;
      if(cornerCnt>=CORNER_CONFIRM){turnDir=candidate;lastSide=candidate;resetCorner();rmode=RM_ADVANCE;modeStartMs=now;resetPid();setMotors(TURN_FWD_SPEED,TURN_FWD_SPEED);return;}
    }
  }else resetCorner();
  // Preserve nominal V5 gain scaling: derivative per nominal sample, integral in nominal samples.
  float ratio=1.0f;float deriv=0;
  if(havePrev){ratio=constrain((nowUs-lastPidUs)/(float)LOOP_US,0.5f,5.0f);deriv=constrain((currentErr-lastErr)/ratio,-300.0f,300.0f);}
  else{havePrev=true;derivFilt=0;}
  lastPidUs=nowUs;lastErr=currentErr;
  derivFilt+=DERIV_ALPHA*(deriv-derivFilt);
  float proposed=constrain(integral+currentErr*ratio,-20000.0f,20000.0f);
  float slow=1.0f-0.5f*abs(currentErr)/3500.0f;
  float ramp=0.6f+0.4f*min((now-settleStartMs)/(float)SETTLE_MS,1.0f);
  int base=(int)(baseSpeed*slow*ramp);
  float raw=Kp*currentErr+Ki*proposed+Kd*derivFilt;
  int lower=max(minSpeed,-maxSpeed);
  float limit=(maxSpeed-lower)*0.5f;
  if(Ki==0 || fabsf(raw)<=limit || raw*currentErr<0)integral=proposed;
  float corr=constrain(Kp*currentErr+Ki*integral+Kd*derivFilt,-limit,limit);
  int mag=abs((int)corr);
  base=constrain(base,lower+mag,maxSpeed-mag); // common-mode shift preserves steering authority
  int l=constrain(base+(int)corr,lower,maxSpeed),r=constrain(base-(int)corr,lower,maxSpeed);
  setMotors(l,r);
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
void printParams(){Serial.printf("Kp=%.4f Ki=%.5f Kd=%.3f base=%d max=%d lost=%d corner=%d tspd=%d tadv=%d tfull=%d tcnt=%d\n",Kp,Ki,Kd,baseSpeed,maxSpeed,lostSpin,(int)cornerMode,turnSpeed,turnAdvMs,fullTurnMs,fullTurnCnt);}
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
  if(sscanf(buf,"%15s %f %c",key,&v,&extra)!=2||!isfinite(v)||v<0||v>1000000){Serial.println("Invalid command/value");return;}
  if(!strcmp(key,"kp")&&v<=1)Kp=v;
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
  Serial.println(calibrated?"V6 READY; s to arm":"V6 READY; c to calibrate, s to arm");printParams();
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
  else if(debugOn&&calibrated){currentErr=readPosition(currentLine)-3500;}
  recordEncoder(now);
  if(debugOn&&now-lastDbg>=100){
    lastDbg=now;
    for(int i=0;i<8;i++)Serial.printf("%4u ",sv[i]);
    Serial.printf("| state=%d m=%d line=%d err=%d dist=%.1f pwm=%d,%d enc=%ld,%ld dt=%lu over=%lu\n",(int)state,(int)rmode,(int)currentLine,currentErr,distCm,outL,outR,(long)speedL,(long)speedR,(unsigned long)tickDtUs,(unsigned long)overrunCount);
  }
}
