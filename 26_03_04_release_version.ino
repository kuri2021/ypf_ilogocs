#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <EEPROM.h>

/* -------------------------------------------------------------------------                                         
   Arduino 자동 프로토타입 삽입 이슈 대응: 타입/함수 전방 선언
   ------------------------------------------------------------------------- */
struct SetBundle;
struct ButtonState;

static void processCompleteBundle(const SetBundle& r_in);
static bool readButtonDebounced(ButtonState &b);
static void pushSetpointsToHMI();
static void pushStatusToHMI(float tTop,float tBot,float pBar,uint16_t st,uint16_t flags);
static void settingsTouch();

/* ===================== RUNTIME CONFIG & POLICY (Edit here) ===================== */

// HMI(디스플레이)가 유지시간을 관리(설정/표시/카운트)할 때: 1
#define HMI_MANAGES_HOLD_TIME 1

// UART / HMI
// #define HMI_BAUD_DEFAULT   115200
#define HMI_BAUD_DEFAULT   38400
static const unsigned long MIN_TX_SPACING_MS     = 12;
static const uint16_t STATUS_PUSH_MS             = 200;
static const unsigned long SET_POLL_MS           = 300;
static const unsigned long PUSH_SETS_MIN_GAP_MS  = 1000;

// Buttons
static const bool START_ACTIVE_LOW = false;
static const bool STOP_ACTIVE_LOW  = false;
static const unsigned long BTN_DEBOUNCE_MS = 100;

// ADC / rails
static const int ADC_NEAR_HIGH = 1018;

// LPF (permille)
static const uint16_t LPF_TOP_PERMILLE   = 10;
static const uint16_t LPF_BOT_PERMILLE   = 10;
static const uint16_t LPF_PRESS_PERMILLE = 30;

// Heater PWM window
static const uint16_t HEATER_CYCLE_MS = 500;
inline unsigned long heaterCycleMsVar(){ return (unsigned long)HEATER_CYCLE_MS; }

// Heater duty policy
static const float TH20=20.0f, TH10=10.0f, TH5=5.0f;
static const uint16_t RAMP_FLOOR20_T = 1000;
static const uint16_t RAMP_FLOOR10_T = 1000;
static const uint16_t RAMP_FLOOR5_T  = 1000;
static const uint16_t RAMP_FLOOR20_B = 1000;
static const uint16_t RAMP_FLOOR10_B = 1000;
static const uint16_t RAMP_FLOOR5_B  = 1000;
static const uint16_t NEW_FLOOR_0_5_PERMILLE = 1000;
static const uint16_t RAMP_FLOOR_MAX_T = 1000;
static const uint16_t RAMP_FLOOR_MAX_B = 1000;

inline float perm2duty(uint16_t p){ return (float)p/1000.0f; }
inline float floor05(){ return perm2duty(NEW_FLOOR_0_5_PERMILLE); }
static inline float capDutyByDelta(float){ return 1.0f; }
static inline float applyMaxCapTop(float u){ float cap = perm2duty(RAMP_FLOOR_MAX_T); return (u>cap)?cap:u; }
static inline float applyMaxCapBot(float u){ float cap = perm2duty(RAMP_FLOOR_MAX_B); return (u>cap)?cap:u; }

// Pressure policy
static const uint16_t PRESSURE_HYST_CBAR = 5;   // 0.05 bar
static inline float pressureHystVar(){ return PRESSURE_HYST_CBAR/100.0f; }
static const float  P_BAND_BAR = 0.10f;
static const unsigned long EXH_BLEED_ON_MS  = 100;
static const unsigned long EXH_BLEED_OFF_MS = 300;
static const unsigned long EXH_BLEED_DEAD_MS= 200;

// HOLD window / stop-to-idle
static const uint8_t  END_TO_IDLE_WAIT_SEC = 1;

// HOLD time bounds
static const uint16_t HOLD_MIN_SEC = 0;
static const uint16_t HOLD_MAX_SEC = 3600;

// ====== HOLD predictive bang-bang (raw) — 상수값 ======
static const float HOLD_ON_EPS_BASE  = 0.10f;  // 더 일찍 켜기(하한 0.1°C 적용)
static const float HOLD_OFF_EPS_BASE = 0.05f;  // 더 늦게 끄기
static const float PREEMPT_PER_RATE  = 0.45f;  // 냉각 시 on 임계 상향 강화
static const float PREEMPT_MAX       = 0.35f;  // 선제 보정 최대치 (이제 실제 사용: clamp)

// IO polarity
const bool HEATER_TOP_ACTIVE_HIGH = true;
const bool HEATER_BOT_ACTIVE_HIGH = true;
const bool PUMP_ACTIVE_HIGH       = true;
const bool FAN_TOP_ACTIVE_HIGH    = true;
const bool FAN_BOT_ACTIVE_HIGH    = true;
const bool EXHAUST_ACTIVE_HIGH    = true;

// Small common helpers
static inline float round1(float x){ return floorf(x*10.0f + 0.5f) / 10.0f; }
static inline int16_t toIntFloorC(float x){ return (int16_t)floorf(x); }
static inline int16_t toIntFloor01Bar(float bar){ return (int16_t)floorf(bar*10.0f + 1e-3f); }
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

// DGUS read stride mode (개별 VP 읽기/쓰기)
static bool SET_STRIDE2 = true;

/* =================== END RUNTIME CONFIG & POLICY (Stop editing) ================= */

static uint16_t save_flag = 0;

static bool Press_flag = false;
/* 압력 제어 */

/* ===================== 타입/구조체 ===================== */
struct SetBundle { uint16_t tt, tb, p01, hold, chtt, chtb; };
struct ButtonState { 
  int pin; 
  bool activeLow; 
  bool lastStable; 
  bool lastRead; 
  unsigned long lastChangeMs; 
   unsigned long pressStartMs;
  bool longPressFired;
  };

/* ===================== 디버그 ===================== */
#define DEBUG_LEVEL 2
#if DEBUG_LEVEL >= 1
  #define DBG_BEGIN()      Serial.begin(115200)
#else
  #define DBG_BEGIN()      Serial.begin(115200)
#endif

/* ===================== 핀 배치 ===================== */
const int pressurePin   = A0;
const int tempTopPin    = A2;
const int tempBotPin    = A3;
const int heaterTopPIN  = 21;
const int heaterBotPIN  = 22;
const int pumpPIN       = 32;
const int fanTopPIN     = 33;
const int fanBotPIN     = 34;
const int exhaustPIN    = 35;
const int startBtnPin   = 6;
const int stopBtnPin    = 7;

/* ===================== 출력 드라이브 ===================== */
inline void driveLevel(int pin, bool on, bool activeHigh){
  if(pin<0) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, (on ? (activeHigh?HIGH:LOW) : (activeHigh?LOW:HIGH)));
}

/* ===================== 세트포인트 기본값 ===================== */
float setTempTop       = 30.0f;
float setTempBot       = 30.0f;
float setPressureBar   = 0.5f;
unsigned long holdTimeSec = 60;
float coolEndTopHeatC  = 50.0f;
float coolEndBotHeatC  = 50.0f;



/* ===================== 상태/플래그 ===================== */
enum { B0_READY, B1_START, B2_STOP, B3_RUN, B4_PUMP, B5_TOP_HEAT, B6_BOT_HEAT, B7_RAMP, B8_HOLD, B9_TOP_FAN, B10_BOT_FAN, B11_EXH, B12_COOL, B13_SENSOR_ERR, B14_TOTAL };
uint16_t FLAGS=0;
inline void setB(uint8_t b,bool v){ if(v) FLAGS|=(1<<b); else FLAGS&=~(1<<b); }
bool set_flag = false;

/* ===================== 상태 머신 ===================== */
enum Stage : uint8_t { ST_IDLE, ST_RAMP, ST_PRESS, ST_AT_TEMP, ST_HOLD, ST_COOL, ST_END };
Stage ST = ST_IDLE;
void setStage(Stage next);

/* ===================== 런타임 타임스탬프/상태 ===================== */
unsigned long t_total_start=0, t_ramp_start=0, t_hold_start=0, t_cool_start=0, t_end_start=0;
unsigned long t_heater_cycle=0;
int   onTopMs=0, onBotMs=0;
bool  topAtSet=false, botAtSet=false, pressReached=false;
float lastDutyTop=0.0f, lastDutyBot=0.0f;
bool  holdEntered=false;

/* ========= 배기 현재 상태 ========= */
static bool g_exhaustOpen = true;

/* ========= 세트 폴링/전송 스로틀 ========= */
static unsigned long lastSetPollMs = 0;
static unsigned long lastPushSetsMs = 0;
static unsigned long lastTxTimeMs   = 0;

/* ========= 버튼 ========= */
static ButtonState btnStart = {startBtnPin, START_ACTIVE_LOW, false, false, 0};
static ButtonState btnStop  = {stopBtnPin,  STOP_ACTIVE_LOW,  false, false, 0};

/* ===================== DGUS / VP ===================== */
HardwareSerial& HMI = Serial1;
#define VP_TT        0x8000
#define VP_TB        0x8002
#define VP_P         0x8004
#define VP_ST        0x800A
#define VP_FLAGS     0x800C
#define VP_SET_TT    0x8100
#define VP_SET_TB    0x8102
#define VP_SET_P     0x8104
#define VP_SET_H     0x8106
#define VP_SET_CHTT  0x8108
#define VP_SET_CHTB  0x810A
#define VP_HOLD_ELAP   0x8010
#define VP_HOLD_REMAIN 0x8012
#define VP_ELAPSED_TIME 0x8114
#define VP_DATA_PUSH     0x8200
#define VP_ACTIVE     0x8500
#define VP_TEST2     0x8202

static int TtValue = 0;
static int TcValue = 0;
static int BtValue = 0;
static int BcValue = 0;
static int PValue = 0;
static int TValue = 0;
static int test1 = 0;
static int test2 = 0;

static int NewTtValue = 0;
static int NewTcValue = 0;
static int NewBtValue = 0;
static int NewBcValue = 0;
static int NewPValue = 0;
static int NewTValue = 0;
static int New_data = 0;
static int Newtest2 = 0;
static int active_flag = 0;

//세팅 데이터 어드레스(지금은 사용 x)
static const int TTAdress = 10;
static const int TCAdress = 13;
static const int BTAdress = 16;
static const int BCAdress = 19;
static const int PAdress = 22;
static const int TAdress = 25;

/* ===================== 유틸/센서 ===================== */
// 아날로그 신호 변환 펑션
int analogReadStable(int pin){ 
  analogRead(pin); 
  delayMicroseconds(50); 
  return analogRead(pin); 
}

//온도 센서 값 변환 펑션
float readTempFrom4_20mA(int aPin){
  int raw=analogRead(aPin); 
  float v=raw*(5.0f/1023.0f);
  float mA=v/0.25f; 
  mA=constrain(mA,4.0f,20.0f);
  return (mA-4.0f)*(250.0f/16.0f)-30.0f;
}

//압력 센서 값 변환
float readPressureBar(){
  int raw = analogRead(pressurePin); 
  float v  = raw*(5.0f/1023.0f);
  v=constrain(v,1.0f,5.0f);
  float bar=((v-1.0f)/4.0f)*10.0f; if(bar<0.15f) bar=0.0f;
  return bar;
}
// 값 전환 시 노이즈 필터
inline float lpf(float prev,float x,uint16_t a_permille){
   float k=a_permille/1000.0f; 
   return prev + k*(x - prev); 
}
bool  lpfInit=false; float f_tTop=0, f_tBot=0, f_pBar=0;

//crc 펑션(display에서 작동 안해서 지금은 더미 파일)
uint16_t crc16_modbus(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/* ===================== DGUS I/O ===================== */
//UART 통신 속도 제한 펑션(overflow나 통신 깨짐 방지용)
inline void txThrottleWait(){
  unsigned long now = millis();
  if (now - lastTxTimeMs < MIN_TX_SPACING_MS){
    delay(MIN_TX_SPACING_MS - (now - lastTxTimeMs));
  }
  lastTxTimeMs = millis();
}

//retrofit2같은 보드와 디스플레이의 통신 방법
//값을 하나만 쓸때 사용하는 함수
static void dgusWriteVP16(uint16_t vp, uint16_t val){
  txThrottleWait();
  HMI.write(0x5A); 
  HMI.write(0xA5);
  HMI.write((uint8_t)0x05);
  HMI.write((uint8_t)0x82);
  HMI.write(vp>>8); 
  HMI.write(vp&0xFF);
  HMI.write(val>>8); 
  HMI.write(val&0xFF);
}

// static void dgusWriteVP16(uint16_t vp, uint16_t val){
//   txThrottleWait();

//   uint8_t buf[8];

//   buf[0] = 0x5A;
//   buf[1] = 0xA5;
//   buf[2] = 0x05;
//   buf[3] = 0x82;
//   buf[4] = vp >> 8;
//   buf[5] = vp & 0xFF;
//   buf[6] = val >> 8;
//   buf[7] = val & 0xFF;

//   uint16_t crc = crc16_modbus(buf, 8);

//   HMI.write(buf, 8);
//   HMI.write(crc & 0xFF);       // CRC L
//   HMI.write(crc >> 8);         // CRC H
// }
//여러 값을 주소 값을 적는 함수
static void dgusWriteVPWords(uint16_t vp, const uint16_t* data, uint16_t words){
  txThrottleWait();
  uint8_t len = (uint8_t)(1 + 2 + 2*words);
  HMI.write(0x5A); HMI.write(0xA5);
  HMI.write((uint8_t)len);
  HMI.write((uint8_t)0x82);
  HMI.write((uint8_t)(vp >> 8)); 
  HMI.write((uint8_t)(vp & 0xFF));
  for (uint16_t i = 0; i < words; i++) {
    uint16_t w = data[i]; 
    HMI.write((uint8_t)(w >> 8)); 
    HMI.write((uint8_t)(w & 0xFF));
  }
}

inline void dgusWriteVP(uint16_t vp, uint16_t w){ 
  dgusWriteVP16(vp,w); 
  }
inline void dgusWriteVP(uint16_t vp, const uint16_t* d, uint16_t n){ 
  dgusWriteVPWords(vp,d,n); 
  }
  //display에게 읽기 요청 함수
static void dgusReadVP(uint16_t vp, uint8_t words=1){
  txThrottleWait();
  HMI.write(0x5A); HMI.write(0xA5);
  HMI.write((uint8_t)0x04);
  HMI.write((uint8_t)0x83);
  HMI.write(vp>>8); HMI.write(vp&0xFF);
  HMI.write(words);
}

/* ===================== 세트/상태 푸시 ===================== */
static uint16_t holdElapsedSec(){
  if (ST==ST_HOLD || ST==ST_COOL || ST==ST_END){
    unsigned long el = (millis() - t_hold_start)/1000UL;
    return (uint16_t)min(65535UL, el);
  }
  return 0;
}
static uint16_t holdRemainSec(){
  unsigned long el = holdElapsedSec();
  return (el >= holdTimeSec) ? 0 : (uint16_t)(holdTimeSec - el);
}
//상태 데이터 푸시(상판 온도, 하판 온도, 압력, 상태코드)
static void pushStatusToHMI(float tTop,float tBot,float pBar,uint16_t st,uint16_t flags){
  int16_t tt  = toIntFloorC(tTop);
  int16_t tb  = toIntFloorC(tBot);
  int16_t p01 = toIntFloor01Bar(pBar);

  dgusWriteVP(VP_TT,  (uint16_t)tt);
  dgusWriteVP(VP_TB,  (uint16_t)tb);
  if(Press_flag == true){
    
  }
  dgusWriteVP(VP_P,   (uint16_t)p01);
  dgusWriteVP(VP_ST,  st);
  dgusWriteVP(VP_FLAGS, flags);

  #if !HMI_MANAGES_HOLD_TIME
    dgusWriteVP(VP_HOLD_ELAP,   holdElapsedSec());
    dgusWriteVP(VP_HOLD_REMAIN, holdRemainSec());
  #endif
}
// 보드에서 설정 값을 세팅하는 함수
// 상시 디폴트 상태에서도 계속 세팅하는 쓰는 상태에서 보드에 한번 전원이 들어갔을때만 작동 하게 바꿈
static void pushSetpointsToHMI(){
  dgusWriteVP(VP_SET_TT,   TtValue);
  dgusWriteVP(VP_SET_CHTT,   TcValue );
  dgusWriteVP(VP_SET_TB,   BtValue );
  dgusWriteVP(VP_SET_CHTB,   BcValue );
  dgusWriteVP(VP_SET_P,   PValue );
  dgusWriteVP(VP_SET_H,   TValue );
  dgusWriteVP(VP_DATA_PUSH, 0);
  new_Data = 0;
  set_flag = true;
}
// .........................................................................................................................................................................................................................
/* ===================== DGUS 수신 파서 ===================== */
static SetBundle acc{};
static bool saneBundle_noHold(const SetBundle& r){
  if (r.tt > 300 || r.tb > 300) return false;
  if (r.p01 > 500) return false;
  if (r.chtt > 300 || r.chtb > 300) return false;
  return true;
}
// 디스플레이에서 묶음으로 데이터를 받았을때 정리하는 코드(한번에 데이터를 가공 시 데이터 형 변환으로 인해 문제가 있어 현재는 쓰고 있지않음)
static void processCompleteBundle(const SetBundle& r_in){
  SetBundle r = r_in;
  // BCD MM:SS -> sec 변환
  if (int s = [](uint16_t mmss)->int{
        auto bcd8_to_dec = [](uint8_t b)->int{
          uint8_t hi=(b>>4)&0x0F, lo=b&0x0F; if(hi>9||lo>9) return -1; return hi*10+lo;
        };
        int mm=bcd8_to_dec((mmss>>8)&0xFF), ss=bcd8_to_dec(mmss&0xFF);
        if(mm<0||ss<0||ss>59) return -1; return mm*60+ss;
      }(r.hold); s >= 0){ r.hold = (uint16_t)s; }
  r.hold = clamp_u16(r.hold, HOLD_MIN_SEC, HOLD_MAX_SEC);

  if ((r.tt|r.tb|r.p01|r.hold|r.chtt|r.chtb) == 0){ return; }
  if (!saneBundle_noHold(r)){ return; }

  bool any=false;

  if((float)r.tt != setTempTop)            { setTempTop = (float)r.tt; any=true; }
  if((float)r.tb != setTempBot)            { setTempBot = (float)r.tb; any=true; }
  if((float)r.p01/10.0f != setPressureBar) { setPressureBar = r.p01/10.0f; any=true; }
  if((unsigned long)r.hold != holdTimeSec) { holdTimeSec = r.hold; any=true; }
  if((float)r.chtt != coolEndTopHeatC)     { coolEndTopHeatC = (float)r.chtt; any=true; }
  if((float)r.chtb != coolEndBotHeatC)     { coolEndBotHeatC = (float)r.chtb; any=true; }

settingsTouch();
}
// read를 요청 시 리턴 받는 함수
// 디스플레이에서 변화된 데이터가 있을 시 데이터를 무조건 저장하는 방식이었으나 지금은 메모리(휘발성) 변수에 두어 저장이후 15초의 딜레이를 걸어 저장의 안정성을 향상
bool test_flag =false;
void pollHMI(){
  static enum {H_HDR1,H_HDR2,H_LEN,H_PAYLOAD} st=H_HDR1;
  static uint8_t len=0, idx=0; 
  static uint8_t pkt[128];

  while(HMI.available()){
    uint8_t c=HMI.read();
    switch(st){
      case H_HDR1: if(c==0x5A) st=H_HDR2; break;
      case H_HDR2: if(c==0xA5) st=H_LEN; else st=H_HDR1; break;
      case H_LEN:  
      len=c;
      idx=0; 
      st=H_PAYLOAD; 
      if(len>sizeof(pkt)) st=H_HDR1; 
      break;
      case H_PAYLOAD:
        pkt[idx++]=c;
        if(idx>=len){
  //           /* ---------- CRC 검사 ---------- */
  //현재 디스플레이에서는 crc와 맞지않아 현재에는 주석상태
  // uint16_t recv_crc =
  //     ((uint16_t)pkt[len] << 8) | pkt[len+1];

  // uint16_t calc_crc = crc16_modbus(pkt, len);

  // if(recv_crc != calc_crc){
  //   Serial.println("CRC ERROR -> drop");
  //   st = H_HDR1;
  //   idx = 0;
  //   return;
  // }
          uint8_t cmd=pkt[0];
          if(cmd==0x83){ //vp 읽기 응답 ->쓰기 응답 일 경우 0x82
            if (len >= 4) {
              uint16_t base_vp = (uint16_t(pkt[1])<<8) | pkt[2];
              uint8_t  words   = (len - 4) / 2;
              auto getByIndex = [&](int i)->uint16_t{
                int o = 4 + i*2; 
                if (o+1 >= len) return 0;
                return (uint16_t(pkt[o])<<8) | pkt[o+1];
              };
                if (words>=1){
                  uint16_t v = getByIndex(0);
                  switch(base_vp){
                    case VP_ACTIVE :{
                      active_flag = v;
                    }
                    case VP_SET_TT : {
                      NewTtValue =v;
                    if(NewTtValue != TtValue && NewTtValue > 0 && NewTtValue < 201){
                      TtValue  = NewTtValue;
                      setTempTop = (float)NewTtValue;
                      settingsTouch();
                      
                    }
                    }break;
                      case VP_SET_TB : {
                      NewBtValue=v;
                    if(NewBtValue != BtValue && NewBtValue > 0 && NewBtValue < 201){
                      BtValue  = NewBtValue;
                      setTempBot = (float)NewBtValue;
                      settingsTouch();
                    }
                    }break;
                      case VP_SET_P : {
                        NewPValue=v;
                    if(NewPValue != PValue  && NewPValue >= 0 && NewPValue < 21 && active_flag !=0){
                      PValue   = NewPValue;
                      setPressureBar =NewPValue/10.0f ;
                      settingsTouch();
                    }
                    }break;
                      case VP_SET_H : {
                    NewTValue=v;
                    if(NewTValue != TValue  && NewTValue >= 0 && NewTValue < 3601 && active_flag !=0){
                      TValue   = NewTValue;
                      holdTimeSec = NewTValue;
                      settingsTouch();
                    }
                    }break;
                      case VP_SET_CHTT : {
                    NewTcValue=v;
                    if(NewTcValue != TcValue && NewTcValue > 0 && NewTcValue < 81){
                      TcValue   = NewTcValue;
                      coolEndTopHeatC = (float)NewTcValue;
                      settingsTouch();
                    }
                    }break;
                     case VP_SET_CHTB : {
                        NewBcValue=v;
                    if(NewBcValue != BcValue  && NewBcValue > 0 && NewBcValue < 81){
                      BcValue   = NewBcValue;
                      coolEndBotHeatC = (float)NewBcValue;
                      settingsTouch();
                    }
                    }break;
                    case VP_DATA_PUSH : {
                        New_data=v;
                        if(New_data != 0){
                          data_set();
                        }
                    }break;
                     case VP_TEST2 : {
                        Newtest2=v;
                          if(Newtest2 != test2){
                      Serial.print("전 test2 데이터 = ");
                      Serial.println(test2);
                      Serial.print("뉴 test2 데이터 = ");
                      Serial.println(Newtest2);
                      test2   = Newtest2;
                          }
                    }break;
                  // }
                }
              }
            }
          }
          st=H_HDR1;
        }
        break;
    }
  }
}

/* ===================== EEPROM 저장/로드 ===================== */

static const uint16_t SETTINGS_MAGIC = 0xABCD; // EEPROM에 저장된 데이터가 정상인지 확인하는 표시값
struct Settings {
  uint16_t setTT, setTB, setP_x10, holdSec, coolTT, coolTB, magic;
};

static bool savePending=false;
static unsigned long lastChangeMs=0, lastSaveMs=0;
static const unsigned long SAVE_DEBOUNCE_MS = 100;
static const unsigned long SAVE_MIN_GAP_MS  = 1000;
//저장 할 때 쓰는 시작 함수
static void settingsTouch(){ savePending=true; lastChangeMs=millis(); }

static unsigned long lastPrintMs = 0;
static const unsigned long PRINT_INTERVAL_MS = 5000;
//저장 제어 함수
//pullhmi에서 작업하던 저장 매커니즘이 여기 함수로 이전
//저장은 기계가 작동중이 아닌 상태인 IDLE상태에서만 저장하게끔 설정
//저장이후 15초간 딜레이를 하여 15초 뒤 변경된 데이터가 있을 시 데이터 저장
static void saveTask(){
  if(ST != ST_IDLE) return;

  unsigned long now = millis();

  if(savePending &&
     (now - lastChangeMs >= SAVE_DEBOUNCE_MS) ){
      
    lastSaveMs = now;
    savePending = false;
    saveSettingsToEEPROM();
  }
}
//저장 하는 함수
static void saveSettingsToEEPROM(){
  Settings s;

  s.setTT   = TtValue;
  s.setTB   = BtValue;
  s.setP_x10= PValue;
  s.holdSec = TValue;
  s.coolTT  = TcValue;
  s.coolTB  = BcValue;
  s.magic   = SETTINGS_MAGIC;

if(TtValue==0||BtValue==0||active_flag ==0){
  Serial.println("기계 전원 off");
}else{
    EEPROM.put(37, s);
    Serial.println("EEPROM 저장 완료");
}

}

/* ===================== 디버그 보조 ===================== */
static const unsigned long BRIEF_TICK_MS = 1000;
static unsigned long lastBriefMs = 0;
static const char* stageName(uint8_t s){
  switch(s){
    case ST_IDLE: return "IDLE"; 
    case ST_RAMP: return "RAMP"; 
    case ST_PRESS: return "PRESS";
    case ST_AT_TEMP: return "AT_TEMP"; 
    case ST_HOLD: return "HOLD"; 
    case ST_COOL: return "COOL";
    case ST_END: return "END"; 
    default: return "?";
  }
}
//현재의 온도를 실시간으로 알려주는 시그널 함수
static inline void printBriefStatus(){
  if (millis() - lastBriefMs < BRIEF_TICK_MS) return;
  lastBriefMs = millis();
  Serial.print(F("[BRIEF] TT=")); Serial.print(f_tTop,1);
  Serial.print(F(" TB="));        Serial.print(f_tBot,1);
  Serial.print(F(" P="));         Serial.print(f_pBar,2);
  Serial.print(F(" ST="));        Serial.println(stageName((uint8_t)ST));
}
// 시간을 분과 초의 형식으로 바꿔주는 함수
static void printMMSS(const char* label, unsigned long ms){
  unsigned long sec = ms/1000UL; 
  unsigned int mm = (unsigned int)(sec/60UL); 
  unsigned int ss = (unsigned int)(sec%60UL);
  Serial.print(label); 
  Serial.print(' '); 
  Serial.print(mm); 
  Serial.print(F("분"));
  Serial.print(ss); 
  Serial.println(F("초"));
}
//한 사이클이 끝났을 때 작동하는 함수 
//예)
// 총 시간    : 2분10초
// 가열(RAMP) : 0분0초
// 유지(HOLD) : 1분15초
// 냉각(COOL) : 0분55초
static void printCycleTimesOnce(){
  unsigned long now = millis();
  unsigned long totalMs = (t_total_start>0) ? (now - t_total_start) : 0;
  unsigned long rampMs = 0;
  if (t_ramp_start>0){
    if (t_hold_start>t_ramp_start) rampMs = t_hold_start - t_ramp_start;
    else rampMs = now - t_ramp_start;
  }
  unsigned long holdMs = 0;
  if (t_hold_start>0){
    if (t_cool_start>t_hold_start) holdMs = t_cool_start - t_hold_start;
    else holdMs = now - t_hold_start;
  }
  unsigned long coolMs = 0;
  if (t_cool_start>0){ coolMs = now - t_cool_start; }

  Serial.println(F("=== 사이클 시간 요약 ==="));
  printMMSS("총 시간    :", totalMs);
  printMMSS("가열(RAMP) :", rampMs);
  printMMSS("유지(HOLD) :", holdMs);
  printMMSS("냉각(COOL) :", coolMs);
  Serial.println(F("======================="));
}

/* ===================== 제어/상태머신 보조 ===================== */
// 장비 전원을 리셋 시키는 함수(초기화)
void allOff(){
  driveLevel(heaterTopPIN,false,HEATER_TOP_ACTIVE_HIGH);   setB(B5_TOP_HEAT,false);
  driveLevel(heaterBotPIN,false,HEATER_BOT_ACTIVE_HIGH);   setB(B6_BOT_HEAT,false);
  driveLevel(pumpPIN,false,PUMP_ACTIVE_HIGH);              setB(B4_PUMP,false);
  driveLevel(fanTopPIN,false,FAN_TOP_ACTIVE_HIGH);         setB(B9_TOP_FAN,false);
  driveLevel(fanBotPIN,false,FAN_BOT_ACTIVE_HIGH);         setB(B10_BOT_FAN,false);
}
// 작업 시 상판 온도를 설정 온도까지 올라가는데 컨트롤하는 함수
// 설정 온도와 현재 온도를 비교 하여 출력을 제한 하는 코드
float dutyTop(float sp,float cur){
  float d = sp - cur; if (d <= 0) return 0.0f;
  float base = (d >= TH20) ? 1.0f : (d / TH20);
  float floor = (d >= TH20)?perm2duty(RAMP_FLOOR20_T):(d >= TH10)?perm2duty(RAMP_FLOOR10_T):(d >= TH5)?perm2duty(RAMP_FLOOR5_T):0.0f;
  if (d > 0 && d < TH5) floor = max(floor, floor05());
  float u = max(base, floor);
  if (u > capDutyByDelta(d)) u = capDutyByDelta(d);
  u = applyMaxCapTop(u);
  return (u>1.0f)?1.0f:u;
}
// 작업 시 하판 온도를 설정 온도까지 올라가는데 컨트롤하는 함수
// 설정 온도와 현재 온도를 비교 하여 출력을 제한 하는 코드
float dutyBot(float sp,float cur){
  float d = sp - cur; if (d <= 0) return 0.0f;
  float base = (d >= TH20) ? 1.0f : (d / TH20);
  float floor = (d >= TH20)?perm2duty(RAMP_FLOOR20_B):(d >= TH10)?perm2duty(RAMP_FLOOR10_B):(d >= TH5)?perm2duty(RAMP_FLOOR5_B):0.0f;
  if (d > 0 && d < TH5) floor = max(floor, floor05());
  float u = max(base, floor);
  if (u > capDutyByDelta(d)) u = capDutyByDelta(d);
  u = applyMaxCapBot(u);
  return (u>1.0f)?1.0f:u;
}
// 작업 시 온도를 설정 온도까지 올리는 함수
void ctrlHeaterPair(float tTop,float tBot){
  unsigned long now = millis();
  unsigned long cycle = heaterCycleMsVar();
  unsigned long el = now - t_heater_cycle;

  if(el >= cycle){ 
    t_heater_cycle = now; 
    el = 0; 
  }

  float errTop = setTempTop - tTop;
  float errBot = setTempBot - tBot;

  float dT = dutyTop(setTempTop,    tTop);
  float dB = dutyBot(setTempBot, tBot);

  if(errTop > 3.0f){
    dT = max(dT, 0.8f);  
  }  
   if(errBot > 3.0f){
    dB = max(dB, 0.8f);
  }  
  

  if(topAtSet) dT = 0; 
  if(botAtSet) dB = 0;

  lastDutyTop = dT; 
  lastDutyBot = dB;

  int onTopMs = (int)(dT * cycle); 
  int onBotMs = (int)(dB * cycle);

  bool topOn = (el < onTopMs);
  bool botOn = (el < onBotMs);

  driveLevel(heaterTopPIN, topOn, HEATER_TOP_ACTIVE_HIGH); 
  setB(B5_TOP_HEAT, topOn);

  driveLevel(heaterBotPIN, botOn, HEATER_BOT_ACTIVE_HIGH); 
  setB(B6_BOT_HEAT, botOn);
}





//현재 압력을 목표 압력에 맞게 유지하기 위해 펌프와 배기 밸브를 자동 제어하는 함수
// 작업 시작 시 베기 닫기 및 콤프레샤 관리
// 압력이 낮으면 콤프레샤 ON 높으면 OFF
// 압력이 너무 높아지면 배기를 열어 안전관리
static void maintainPressure_UIAligned(float pBar, bool isRunState){
  if (!isRunState) return;

  const float dispP   = round1(pBar);
  const float dispSet = round1(setPressureBar);

  const bool pumpOn = ((FLAGS>>B4_PUMP)&1);
  
  if (g_exhaustOpen){// 공압 배기 상태일때
      driveLevel(exhaustPIN, false, EXHAUST_ACTIVE_HIGH);//배기 기기 닫음(p35)
      setB(B11_EXH, false); // 배기 기기가 닫혀있는다는 것을 기록하는 함수(시리얼 모니터 적합)
      g_exhaustOpen = false;// 배기 기기 플래그
  }

  if(pBar < setPressureBar){
    if(!Press_flag){
      driveLevel(pumpPIN, true, PUMP_ACTIVE_HIGH);//펌프 작동
      setB(B4_PUMP, true);//펌프 작동을 기록
    }else{
        float RefactoringDispP = setPressureBar - 0.2f;
     if(RefactoringDispP>=pBar){
        Press_flag = false;
      }else{
        Press_flag = true;
      }
    if(Press_flag){
      driveLevel(pumpPIN, false, PUMP_ACTIVE_HIGH);//펌프 중지
      setB(B4_PUMP, false);// 펌프 중지를 기록
    }else{
      driveLevel(pumpPIN, true, PUMP_ACTIVE_HIGH);//펌프 작동
      setB(B4_PUMP, true);//펌프 작동을 기록
    }
    }
  
  }else{
    driveLevel(pumpPIN, false, PUMP_ACTIVE_HIGH);//펌프 중지
    setB(B4_PUMP, false);// 펌프 중지를 기록
    Press_flag = true;
  }
  static bool  bleedActive = false;
  static unsigned long bleedTglMs = 0, bleedDoneMs = 0;
  const unsigned long now = millis();
  // const float hi = setPressureBar + P_BAND_BAR;
  const float hi = setPressureBar + 0.2f;

  if (!bleedActive){
    if (pBar > hi && (now - bleedDoneMs) >= EXH_BLEED_DEAD_MS && (now - bleedTglMs) >= EXH_BLEED_OFF_MS){
      driveLevel(exhaustPIN, true, EXHAUST_ACTIVE_HIGH); setB(B11_EXH, true);
      bleedActive = true; bleedTglMs = now;
    }
  } else {
    if (now - bleedTglMs >= EXH_BLEED_ON_MS || pBar <= setPressureBar){
      driveLevel(exhaustPIN, false, EXHAUST_ACTIVE_HIGH); setB(B11_EXH, false);
      bleedActive = false; bleedTglMs = now; bleedDoneMs = now;
    }
  }
}

/* ===================== HOLD 예측형 bang-bang(raw) ===================== */

static float lastTopC = 0.0f, lastBotC = 0.0f;
static unsigned long lastRateMs = 0;

// ★ 드리프트 보정(I-바이어스)
static float holdBiasTop = 0.0f, holdBiasBot = 0.0f;  // °C
static unsigned long lastBiasMs = 0;

// 목표 온도를 안정적으로 유지하기 위해 히터 ON/OFF 시점을 유기적으로 조절하는 함수
// 온도 변화 속도를 계산 하여 온도 상승 및 하강 속도 제어
static void updateTempRates(float tTop_raw, float tBot_raw){
  unsigned long now = millis();
  if (lastRateMs == 0) { lastRateMs = now; lastTopC = tTop_raw; lastBotC = tBot_raw; return; }
  float dt = (now - lastRateMs) / 1000.0f; if (dt < 0.05f) return;
  lastTopC = tTop_raw; lastBotC = tBot_raw; lastRateMs = now;
}
static float getRiseRate(float current_raw, float last_raw){
  unsigned long now = millis();
  float dt = (now - lastRateMs) / 1000.0f; if (dt <= 0.0f) return 0.0f;
  return (current_raw - last_raw) / dt;
}
static void holdHeaterBangBang_RAW(float tTop_raw, float tBot_raw){
  float rateTop = getRiseRate(tTop_raw, lastTopC);
  float rateBot = getRiseRate(tBot_raw, lastBotC);

  // === 패치 1) PREEMPT_MAX 적용: 선제보정 clamp ===
  float posTop = max(0.0f, rateTop), negTop = max(0.0f, -rateTop);
  float posBot = max(0.0f, rateBot), negBot = max(0.0f, -rateBot);

  float preOffTop = min(PREEMPT_MAX, posTop * PREEMPT_PER_RATE);
  float preOnTop  = min(PREEMPT_MAX, negTop * PREEMPT_PER_RATE);
  float preOffBot = min(PREEMPT_MAX, posBot * PREEMPT_PER_RATE);
  float preOnBot  = min(PREEMPT_MAX, negBot * PREEMPT_PER_RATE);

  float offThTop = setTempTop - (HOLD_OFF_EPS_BASE + preOffTop);
  float onThTop  = setTempTop - max(0.1f, HOLD_ON_EPS_BASE - preOnTop);

  float offThBot = setTempBot - (HOLD_OFF_EPS_BASE + preOffBot);
  float onThBot  = setTempBot - max(0.1f, HOLD_ON_EPS_BASE - preOnBot);

  // === 패치 2) HOLD 진입 초반 OFF 억제 (3s) ===
  const unsigned long WARMUP_MS = 3000;
  bool inWarmup = (millis() - t_hold_start) < WARMUP_MS;
  if (inWarmup) {
    // 초반엔 OFF 임계를 set보다 약간 위로 당겨서 쉽게 꺼지지 않게
    offThTop = max(offThTop, setTempTop + 0.05f);
    offThBot = max(offThBot, setTempBot + 0.05f);
  }

  // === 패치 3) 장기 드리프트 보정(I-바이어스) ===
  {
    unsigned long now = millis();
    if (lastBiasMs == 0) lastBiasMs = now;
    float dtI = (now - lastBiasMs) / 1000.0f;
    if (dtI > 0.05f) {
      const float Ki = 0.003f; // 아주 약하게(느리게) 수렴
      float eTop = (tTop_raw - setTempTop); // +면 과열, -면 부족
      float eBot = (tBot_raw - setTempBot);
      holdBiasTop += (-Ki) * eTop * dtI;   // 과열(+e) → bias 음(-) → OFF 쉬워짐
      holdBiasBot += (-Ki) * eBot * dtI;
      holdBiasTop = constrain(holdBiasTop, -0.5f, 0.5f);
      holdBiasBot = constrain(holdBiasBot, -0.5f, 0.5f);
      lastBiasMs = now;
    }
    onThTop  += holdBiasTop;  offThTop += holdBiasTop;
    onThBot  += holdBiasBot;  offThBot += holdBiasBot;
  }

  bool onTop = ((FLAGS>>B5_TOP_HEAT)&1);
  bool onBot = ((FLAGS>>B6_BOT_HEAT)&1);

  if (!onTop && tTop_raw <= onThTop){
    driveLevel(heaterTopPIN,true,HEATER_TOP_ACTIVE_HIGH); setB(B5_TOP_HEAT,true);
  } else if (onTop && tTop_raw >= offThTop){
    driveLevel(heaterTopPIN,false,HEATER_TOP_ACTIVE_HIGH); setB(B5_TOP_HEAT,false);
  }

  if (!onBot && tBot_raw <= onThBot){
    driveLevel(heaterBotPIN,true,HEATER_BOT_ACTIVE_HIGH); setB(B6_BOT_HEAT,true);
  } else if (onBot && tBot_raw >= offThBot){
    driveLevel(heaterBotPIN,false,HEATER_BOT_ACTIVE_HIGH); setB(B6_BOT_HEAT,false);
  }
}

/* ===================== 런타임 리셋(EEPROM 제외) ===================== */
// 보드 함수 초기화
static void softResetRuntimeExceptEEPROM(){
  topAtSet=false; 
  botAtSet=false; 
  pressReached=false; 
  holdEntered=false;
  t_total_start=0; 
  t_ramp_start=0; 
  t_hold_start=0; 
  t_cool_start=0;
  t_end_start=0;
  t_heater_cycle=millis(); 
  onTopMs=0; 
  onBotMs=0; 
  lastDutyTop=0; 
  lastDutyBot=0;
  lpfInit=false; 
  f_tTop=0; 
  f_tBot=0; 
  f_pBar=0;
}

/* ===================== 버튼 ===================== */
//시작 버튼 컨트롤 함수
// 인펄스 방지를 위해 0.1초간 신호가 들어올 시 에만 TRUE리턴
static bool readButtonDebounced(ButtonState &b){
  pinMode(b.pin, INPUT);
  bool rawHigh = (digitalRead(b.pin) == HIGH);
  unsigned long now = millis();
  if (rawHigh != b.lastRead) { 
    b.lastRead = rawHigh; 
    b.lastChangeMs = now; 
    }
  if (now - b.lastChangeMs >= BTN_DEBOUNCE_MS) {
    bool stablePressed = b.activeLow ? !b.lastRead : b.lastRead;
    if (stablePressed != b.lastStable) {
      b.lastStable = stablePressed;
      if (stablePressed) return true;
    }
  }
  return false;
}
//정지 버튼을 컨트롤하는 함수
// 인펄스 방지를 위해 0.1초간 신호가 들어올 시 에만 리턴
// 리턴 값 1,2,3
// 1 -> 눌림 확인
// 2 -> 0.1초는 지났으나 3초는 안지남 스테이터스 COOLING
// 3 -> 3초이상 인지 스테이터스 END
static uint8_t readButtonAdvanced(ButtonState &b){
  pinMode(b.pin, INPUT);
  bool rawHigh = (digitalRead(b.pin) == HIGH);
  unsigned long now = millis();

  if (rawHigh != b.lastRead) {
    b.lastRead = rawHigh;
    b.lastChangeMs = now;
  }

  if (now - b.lastChangeMs >= BTN_DEBOUNCE_MS) {
    bool stablePressed = b.activeLow ? !b.lastRead : b.lastRead;

    if (stablePressed != b.lastStable) {
      b.lastStable = stablePressed;

      if (stablePressed) {
        b.pressStartMs = now;
        b.longPressFired = false;
        return 1;  
      } else {
        if (!b.longPressFired)
          return 2; 
      }
    }

    if (b.lastStable && !b.longPressFired) {
      if (now - b.pressStartMs >= 3000) {
        b.longPressFired = true;
        return 3;  
      }
    }
  }
  return 0;
}

//데이터 세이브 함수
//현재는 다른 방식으로 데이터를 저장함(지금은 사용 X)
static void data_save(int EEPROM_ADDR, int value){
  int intValue = value;

  EEPROM.write(EEPROM_ADDR + 0, intValue & 0xFF);         // 하위 바이트
  EEPROM.write(EEPROM_ADDR + 1, (intValue >> 8) & 0xFF);  // 상위 바이트
  
  int readIntValue = EEPROM.read(EEPROM_ADDR + 1) << 8 | EEPROM.read(EEPROM_ADDR + 0);

  switch(EEPROM_ADDR){
    case TTAdress: setTempTop = (float)value; break;
    case BTAdress: setTempBot = (float)value; break;
    case TCAdress: coolEndTopHeatC = (float)value; break;
    case BCAdress: coolEndBotHeatC = (float)value; break;
    case PAdress:  setPressureBar = value / 10.0f; break;   
    case TAdress:  holdTimeSec = value; break;
  }

}
//세이브 데이터 가져오기
//현재는 다른 방식으로 데이터를 가져옴(지금은 사용 X)
static void data_get(int EEPROM_ADDR){
  // int 값 읽기
  int readIntValue = EEPROM.read(EEPROM_ADDR + 1) << 8 | EEPROM.read(EEPROM_ADDR + 0);
}
//저장된 데이터 세팅
//신규로 작성된 데이터 세팅
// EEPROM에 저장된 배열의 데이터를 메모리 전역 변수에 데이터를 가공하여 부여
// SETUP에서 한번만 실행
static void data_set(){
if(New_data != 0){
  Settings s;
EEPROM.get(37, s);

  TtValue = s.setTT;
  BtValue = s.setTB;
  PValue  = s.setP_x10;
  TValue  = s.holdSec;
  TcValue = s.coolTT;
  BcValue = s.coolTB;

  // runtime 변수에도 반영
  setTempTop     = (float)TtValue;
  setTempBot     = (float)BtValue;
  setPressureBar = PValue / 10.0f;
  holdTimeSec    = TValue;
  coolEndTopHeatC = (float)TcValue;
  coolEndBotHeatC = (float)BcValue;
Serial.println(setTempTop);
Serial.println(setTempBot);
Serial.println(holdTimeSec);
Serial.println(setPressureBar);
Serial.println(coolEndTopHeatC);
Serial.println(coolEndBotHeatC);


  pushSetpointsToHMI();
}
  
  
}

unsigned long bootTime = 0;
bool loopReady = false;

/* ===================== setup / loop ===================== */
void setup() {
  DBG_BEGIN(); //시리얼 모니터 속도 설정
  
  delay(20);
  //센서핀 설정
  pinMode(tempTopPin,INPUT); 
  pinMode(tempBotPin,INPUT); 
  pinMode(pressurePin,INPUT);
  analogReference(DEFAULT);
  allOff();

// 배기 오픈(문제가 발생시 다시 켰을때 배기가 나오게끔 설정)
  driveLevel(exhaustPIN, true, EXHAUST_ACTIVE_HIGH);
  setB(B11_EXH, true); g_exhaustOpen = true;

//디스플레이와의 통신 설정
  HMI.begin(HMI_BAUD_DEFAULT);

//버튼 초기 상태 읽기
  pinMode(startBtnPin, INPUT); pinMode(stopBtnPin,  INPUT);
  btnStart.lastRead   = (digitalRead(startBtnPin)==HIGH);
  btnStop.lastRead    = (digitalRead(stopBtnPin)==HIGH);
  btnStart.lastStable = START_ACTIVE_LOW ? !btnStart.lastRead : btnStart.lastRead;
  btnStop.lastStable  = STOP_ACTIVE_LOW  ? !btnStop.lastRead  : btnStop.lastRead;
  btnStart.lastChangeMs = btnStop.lastChangeMs = millis();


  // Settings s;

  // s.setTT   = 100;
  // s.setTB   = 100;
  // s.setP_x10= 1;
  // s.holdSec = 90;
  // s.coolTT  = 10;
  // s.coolTB  = 10;
  // s.magic   = SETTINGS_MAGIC;



  // EEPROM.put(37, s);

  New_data = 1;

 // 저장된 설정 불러오기
  data_set();
  //스테이터스 IDLE 상태 진입
  setStage(ST_IDLE);
  bootTime = millis();
}

//각 현재 설정 된 상태의 작동되는 펑션
void setStage(Stage next){
  ST=next;
  switch(ST){
    //가장 기초적인 상태(보드의 전원이 들어오거나 작업이 끝났을때 상태코드)
    case ST_IDLE: {
      allOff();
      setB(B0_READY,true); 
      setB(B1_START,false); 
      setB(B2_STOP,false);
      setB(B3_RUN,false); 
      setB(B7_RAMP,false); 
      setB(B8_HOLD,false);
      setB(B12_COOL,false); 
      setB(B14_TOTAL,false);

      softResetRuntimeExceptEEPROM();
    } break;

    case ST_RAMP://작업이 시작했을때 작동되는 스테이터스(시간기록)
      topAtSet=false; botAtSet=false; pressReached=false;
      setB(B0_READY,false); 
      setB(B1_START,true); 
      setB(B3_RUN,true); 
      setB(B7_RAMP,true); 
      setB(B14_TOTAL,true);
      t_total_start=millis(); 
      t_ramp_start=millis();
      t_heater_cycle=millis(); 
      holdEntered=false;
      dgusWriteVP(VP_ELAPSED_TIME, TValue);
      break;

    case ST_PRESS:
    case ST_AT_TEMP:
      break;

    case ST_HOLD://설정된 온도와 설정된 압력이 들어갔을때 설정하는 스테이터스
      setB(B7_RAMP,false); 
      setB(B8_HOLD,true);
      t_hold_start=millis(); 
      holdEntered=true;
      driveLevel(fanTopPIN,false,FAN_TOP_ACTIVE_HIGH); 
      setB(B9_TOP_FAN,false);
      driveLevel(fanBotPIN,false,FAN_BOT_ACTIVE_HIGH); 
      setB(B10_BOT_FAN,false);
      break;

    case ST_COOL:// 유지시간이 끝났을때 설정하는 스테이터스
      setB(B7_RAMP, false);
      setB(B1_START, false);
      setB(B8_HOLD,false);
      setB(B12_COOL,true);
      t_cool_start=millis();
      holdEntered=true;
      driveLevel(heaterTopPIN,false,HEATER_TOP_ACTIVE_HIGH); 
      setB(B5_TOP_HEAT,false);
      driveLevel(heaterBotPIN,false,HEATER_BOT_ACTIVE_HIGH); 
      setB(B6_BOT_HEAT,false);
      // driveLevel(pumpPIN,false,PUMP_ACTIVE_HIGH);            
      // setB(B4_PUMP,false);
      driveLevel(fanTopPIN,true,FAN_TOP_ACTIVE_HIGH);        
      setB(B9_TOP_FAN,true);
      driveLevel(fanBotPIN,true,FAN_BOT_ACTIVE_HIGH);        
      setB(B10_BOT_FAN,true);
      break;

    case ST_END: {// 모든 작업이 끝났을때 설정되는 스테이터스
      setB(B7_RAMP, false);
      allOff(); 
      setB(B12_COOL,false); 
      setB(B14_TOTAL,false);
      setB(B3_RUN,false); 
      setB(B1_START,false);
      topAtSet=false; 
      botAtSet=false; 
      pressReached=false;
      t_end_start = millis();
      printCycleTimesOnce();

      if (!g_exhaustOpen){
        driveLevel(exhaustPIN, true, EXHAUST_ACTIVE_HIGH);
        setB(B11_EXH, true);
        g_exhaustOpen = true;
      }
    } break;
  }
}


static int record_time = 0;

#define AT_SET_HYS 1.5f   // 설정온도 기준 1.5도 히스테리시스
unsigned long loopStart, loopEnd;
bool stopPressed = digitalRead(btnStop.pin) == HIGH;  // 또는 네 디바운스 함수의 상태용 반환

//기계 전체 동작 및 관리하는 메인 루프
void loop(){
    if (!loopReady) {
    if (millis() - bootTime >= 5000) {
      loopReady = true;
    } else {
      return; // 아무것도 안함
    }
  }

bool isRunState = (ST!=ST_IDLE && ST!=ST_END);
if(set_flag && ST==ST_IDLE){ // 첫 데이터 세팅이 끝나고 나서 작동하기 시작(기계가 켜질때 디스플레이의 변수는 모두 0이기에 설정 값이 0으로 저장되는것을 방지)
  pollHMI();
}

  if(New_data != 0){
      Serial.print("뉴 데이터 트리거 발동 됨 ");
      data_set();
    }


  int rTop = analogReadStable(tempTopPin); //상판 아날로그 센서 데이터 변환 변수
  int rBot = analogReadStable(tempBotPin); //하판 아날로그 센서 데이터 변환 변수
  int rP   = analogReadStable(pressurePin); //압력 아날로그 센서 데이터 변환 변수
  float rtTop = readTempFrom4_20mA(tempTopPin);// 상판 데이터를 ℃단위로 변환 변수
  float rtBot = readTempFrom4_20mA(tempBotPin);// 하판 데이터를 ℃단위로 변환 변수
  float rpBar = readPressureBar();//압력 데이터를 BAR단위로 변환 변수
  bool topRail = (rTop>=ADC_NEAR_HIGH), botRail=(rBot>=ADC_NEAR_HIGH), pRail=(rP>=ADC_NEAR_HIGH);

  bool startEdge = readButtonDebounced(btnStart);//시작 버튼 리턴 함수
  uint8_t stopEdge  = readButtonAdvanced(btnStop);//정지 버튼 리턴 함수

// 시작 버튼이 눌렸을시 작동하는 IF문
  if (startEdge && (ST==ST_IDLE || ST==ST_END)&&rTop < 1023 && rBot < 1023) {
        if (g_exhaustOpen){
      driveLevel(exhaustPIN, false, EXHAUST_ACTIVE_HIGH);
      setB(B11_EXH, false);
      g_exhaustOpen = false;
    }
    setStage(ST_RAMP);
  }

  //정지버튼이 눌렸을 시 작동하는 IF문
if(ST!=ST_IDLE && ST!=ST_END){
  
  switch(stopEdge){
      case 1:
    break;

  case 2:
    setStage(ST_COOL); 
    break;

  case 3:
    setStage(ST_END); 
    break;
   }
}
 
    


  // int rTop = analogReadStable(tempTopPin); //상판 아날로그 센서 데이터 변환 변수
  // int rBot = analogReadStable(tempBotPin); //하판 아날로그 센서 데이터 변환 변수
  // int rP   = analogReadStable(pressurePin); //압력 아날로그 센서 데이터 변환 변수
  // float rtTop = readTempFrom4_20mA(tempTopPin);// 상판 데이터를 ℃단위로 변환 변수
  // float rtBot = readTempFrom4_20mA(tempBotPin);// 하판 데이터를 ℃단위로 변환 변수
  // Serial.println(rBot);
  // float rpBar = readPressureBar();//압력 데이터를 BAR단위로 변환 변수
  // bool topRail = (rTop>=ADC_NEAR_HIGH), botRail=(rBot>=ADC_NEAR_HIGH), pRail=(rP>=ADC_NEAR_HIGH);

  // (순서 변경: 여기서는 updateTempRates 호출하지 않음)

//센서 필터링(노이즈 제거)
  if(!lpfInit){
    if(!topRail) f_tTop=rtTop; 
    if(!botRail) f_tBot=rtBot; 
    if(!pRail) f_pBar=rpBar; 
    lpfInit=true;
  } else {
    if(!topRail) f_tTop=lpf(f_tTop,rtTop,LPF_TOP_PERMILLE);
    if(!botRail) f_tBot=lpf(f_tBot,rtBot,LPF_BOT_PERMILLE);
    if(!pRail)   f_pBar=lpf(f_pBar,rpBar,LPF_PRESS_PERMILLE);
  }
//센서에 문제 발생 시 히터 및 펌프 작동 정지
  bool sensorFault = (topRail || botRail || pRail);
  if(sensorFault){
    driveLevel(heaterTopPIN,false,HEATER_TOP_ACTIVE_HIGH); setB(B5_TOP_HEAT,false);
    driveLevel(heaterBotPIN,false,HEATER_BOT_ACTIVE_HIGH); setB(B6_BOT_HEAT,false);
    driveLevel(pumpPIN,false,PUMP_ACTIVE_HIGH);            setB(B4_PUMP,false);
  }

  if (ST != ST_PRESS) {
    if (f_tTop < setTempTop - 0.5f) topAtSet=false;
    if (f_tBot < setTempBot - 0.5f) botAtSet=false;
  }

  switch(ST){
    case ST_IDLE:{
      // if(rTop >= 1023 || rBot >= 1023){
      //   setB(B13_SENSOR_ERR, true);
      // }else{
      //   setB(B13_SENSOR_ERR, false);
      // }
    }
    break;

    case ST_RAMP://작업 시작
      
      if(!pRail) maintainPressure_UIAligned(f_pBar, isRunState); // 압력 컨트롤
      //문제 없이 작동 시 스테이터스 ST_PRESS로 변경
      if(!pressReached && !pRail && f_pBar>=setPressureBar){ pressReached=true; setStage(ST_PRESS); }
      if(!sensorFault) ctrlHeaterPair(f_tTop,f_tBot);//히터 컨트롤
      break;

    case ST_PRESS:// 작업 진행중
      if(!pRail) maintainPressure_UIAligned(f_pBar, isRunState);// 압력 컨트롤
      // 상판 도달 판정
     if(!topAtSet && f_tTop >= setTempTop){
       topAtSet = true;
     }else if(topAtSet && f_tTop <= setTempTop - AT_SET_HYS){
    topAtSet = false;
    }

// 하판 도달 판정 
if(!botAtSet && f_tBot >= setTempBot){
    botAtSet = true;
}
else if(botAtSet && f_tBot <= setTempBot - AT_SET_HYS){
    botAtSet = false;
}
// 상판 및 하판 압력이 설정 된 값까지 올라갈 시 스테이터스 ST_HOLD변경
      if(!holdEntered && topAtSet && botAtSet){ setStage(ST_AT_TEMP); setStage(ST_HOLD); }
      if(!sensorFault) ctrlHeaterPair(f_tTop,f_tBot);
      break;

    case ST_AT_TEMP: break;

    case ST_HOLD: {// 지연 시간 시작
      if(!pRail) maintainPressure_UIAligned(f_pBar, isRunState);//압력 컨트롤

      if(!sensorFault){
        // 제어 먼저 (뱅뱅)
        holdHeaterBangBang_RAW(rtTop, rtBot);
        // 제어 직후 속도 갱신
        updateTempRates(rtTop, rtBot);
      } else {
        driveLevel(heaterTopPIN,false,HEATER_TOP_ACTIVE_HIGH); setB(B5_TOP_HEAT,false);
        driveLevel(heaterBotPIN,false,HEATER_BOT_ACTIVE_HIGH); setB(B6_BOT_HEAT,false);
      }
      // 현재 진행 시간 디스플레이한테 전송
      if(record_time != (millis()-t_hold_start)/1000){
        dgusWriteVP(VP_ELAPSED_TIME, holdTimeSec-(millis()-t_hold_start)/1000);
        record_time = (millis()-t_hold_start)/1000;
      }
      //지연시간이 끝났을때 스테이터스 ST_COOL변경
      if((millis()-t_hold_start)/1000 >= holdTimeSec) setStage(ST_COOL);
    } break;

    case ST_COOL:{
      bool okT=(f_tTop<=coolEndTopHeatC);// 상판 쿨링 온도 제어
      bool okB=(f_tBot<=coolEndBotHeatC);//하판 쿨링 온도 제어 

      if(!pRail) maintainPressure_UIAligned(f_pBar, isRunState);//압력 컨트롤
      //상판 온도가 설정 온도까지 내려갔을때 팬 OFF
      if(okT){
        driveLevel(fanTopPIN,false,FAN_TOP_ACTIVE_HIGH); setB(B9_TOP_FAN,false);
      }
      // else{
      //   driveLevel(fanTopPIN,true,FAN_TOP_ACTIVE_HIGH); setB(B9_TOP_FAN,false);
      // }
      //하판 온도가 설정 온도까지 내려갔을때 팬 OFF
      if(okB){
        driveLevel(fanBotPIN,false,FAN_BOT_ACTIVE_HIGH); setB(B10_BOT_FAN,false);
      }
      // else{
      //   driveLevel(fanBotPIN,false,FAN_BOT_ACTIVE_HIGH);
      // }
      //상하판 온고가 설정 온도까지 내려갔을때 스테이터스 ST_END로 변경
      if(okT && okB){
        driveLevel(fanTopPIN,false,FAN_TOP_ACTIVE_HIGH); setB(B9_TOP_FAN,false);
        driveLevel(fanBotPIN,false,FAN_BOT_ACTIVE_HIGH); setB(B10_BOT_FAN,false);
        setStage(ST_END);
      }
    } break;
// 
    case ST_END://작업 종료
    //작업이 끝나고 약간의 딜레이 타임 이후 스테이터스 ST_IDLE로 변경(딜레이는 배기 때문인걸로 인지)
      if ((millis() - t_end_start) >= (unsigned long)END_TO_IDLE_WAIT_SEC * 1000UL) {
        setStage(ST_IDLE);
      }
      break;
  }
// 현재 상하판온도와 스테이터스 정보를 디스플레이한테 보내줌
//전에는 설정된 값도 같이 보냄
  static unsigned long lastStatMs=0; unsigned long now2=millis();
  if(now2 - lastStatMs >= STATUS_PUSH_MS){
    lastStatMs=now2;
    uint16_t stCode=
      (ST==ST_IDLE)?0:(ST==ST_RAMP)?1:(ST==ST_PRESS)?2:(ST==ST_AT_TEMP)?3:
      (ST==ST_HOLD)?4:(ST==ST_COOL)?5:(ST==ST_END)?8:8;
    pushStatusToHMI(f_tTop,f_tBot,f_pBar,stCode,FLAGS);
    
    if ((now2 - lastPushSetsMs) >= PUSH_SETS_MIN_GAP_MS&& !test_flag) {
      pushSetpointsToHMI();
    }
    if(now2-lastPushSetsMs>5000){
      test_flag = true;
    }
  }


//디스플레이한테 읽기 요청
  if(now2 - lastSetPollMs >= SET_POLL_MS){
    lastSetPollMs = now2;
    dgusReadVP(VP_DATA_PUSH,1);
    dgusReadVP(VP_ACTIVE,1);
      dgusReadVP(VP_SET_TT,1); 
      dgusReadVP(VP_SET_TB,1); 
      dgusReadVP(VP_SET_P,1);
      dgusReadVP(VP_SET_H,1);  
      dgusReadVP(VP_SET_CHTT,1); 
      dgusReadVP(VP_SET_CHTB,1);
      dgusReadVP(VP_TEST2,1);
  }
  //세이브 펑션
  saveTask();
  delay(10);
}
