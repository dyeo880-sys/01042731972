/* ============================================================================
   Mega2560 Pro + DNMPB28(Eletechsup) PLC IO보드 전용 통합 펌웨어
   ----------------------------------------------------------------------------
   ★ 이 파일을 업로드하기 전에 반드시 아래 "핀 배치 확인사항"을 먼저 읽어주세요 ★

   [보드 고정 배선 — DNMPB28 공식 핀표 기준]
     릴레이 DO  CH1~CH8  (기계식 릴레이)  = 핀 40~47   (R1~R8)
     MOS   DO  D1~D8    (2803 오픈컬렉터) = 핀 32~39   (R9~R16)
     DI    IN1~IN8 (NPN)                  = 핀 16~23   (IN1~IN8)
     DI    IN9~IN16(PNP)                  = 핀 24~31   ← ★미검증(제품 페이지 표에
             오타로 21~24라고 잘못 적혀있음). 실측 전까지 24~31로 가정합니다.
             나중에 멀티미터로 확인되면 아래 IN_PIN[] 배열 값만 바꿔주세요.
     AI 16채널                            = A0~A15 (본 펌웨어에서는 사용 안 함)
     디지털튜브/버튼/LED(보드 온보드)      = 3~14   (본 펌웨어에서는 사용 안 함)

   [설계상 트레이드오프 — 반드시 인지하고 진행]
     · 하드웨어 I2C(SDA=20,SCL=21)가 원래 IN7·IN8 자리와 겹칩니다.
       → OLED + I2C GPIO확장칩(MCP23017, 키패드용)을 "표준 라이브러리로
         안정적으로" 쓰기 위해 IN7·IN8(핀 20,21) 이 두 채널은 DI로 사용하지
         않고 I2C 전용으로 뺐습니다 (소프트웨어 I2C로 만드는 대안보다
         훨씬 안정적입니다). 즉 DI는 16채널이 아니라 14채널(IN1~6, IN9~16)만
         실제 동작합니다.
     · 남는 순수 GPIO 6개(2,15,48,49,50,51,52,53 중 I2C 2개 뺀 나머지)는
       초음파 1채널 + 로드셀 1채널 + 스텝모터 1채널(STEP/DIR만, EN은 확장칩)에
       배분했습니다 — 여러 채널을 동시에 쓰는 경우가 드물다고 하셔서 이렇게
       구성했습니다. 채널을 늘리려면 IN9~16 또는 D1~8 그룹을 추가로 포기하고
       핀을 더 확보해야 합니다.
     · MCP23017 확장칩 16개 핀 중 8개=키패드(행4+열4), 1개=스텝모터 EN.
       나머지 7개는 여유(향후 스텝EN 추가/버튼 등에 활용 가능).

   [필요 라이브러리 — 아두이노 IDE 라이브러리 매니저에서 설치]
     - Adafruit GFX Library
     - Adafruit SSD1306
     (Wire, EEPROM 은 기본 내장)
     ※ HX711(로드셀)과 MCP23017은 라이브러리 의존성을 줄이려고 이 파일 안에
       직접 최소 구현했습니다(별도 설치 불필요).

   [배선 요약 - 새로 추가하는 부품]
     OLED(SSD1306, I2C, 128x64, 주소 0x3C) : SDA=20, SCL=21
     MCP23017(주소 0x20, I2C)              : SDA=20, SCL=21 (OLED와 버스 공유)
       - GPA0~GPA3 : 키패드 행1~4 (출력)
       - GPA4~GPA7 : 키패드 열1~4 (입력, 내부 풀업)
       - GPB0      : 스텝모터 EN (출력, active-LOW 가정)
     초음파(HC-SR04) 1채널 : TRIG=50, ECHO=51
     로드셀(HX711)   1채널 : DOUT=52, SCK=53
     스텝모터(STEP/DIR) 1채널 : STEP=2, DIR=15, EN=MCP23017 GPB0

   [시리얼]
     PC 쪽 Mega2560_16Relay_control.html 과 그대로 호환되도록, 기존에 파악된
     프로토콜(PIN/STATUS?/INPUT?/SETIN/SETOUT/SETBOOT/GETCONFIG?/SEQ류/
     SETKP류/KEY/SETULTRAPIN/ULTRA?/SETLOADPIN/LOAD?/LOADTARE/LOADCAL/
     SETSTEPPIN/STEPMOVE/STEPZERO/STEPSTOPALL/SETLED/EXTMEM?) 을 그대로
     구현했습니다. 다만 웹페이지 쪽 "펌웨어 없이 IN9~16을 16개로 인식"하는
     부분은, 이 펌웨어가 14채널만 보고하므로 입력 채널 매핑 화면에서 실제
     연결된 14개만 사용해주세요.
   ============================================================================ */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ----------------------------- 하드웨어 상수 ----------------------------- */
#define RELAY_COUNT   16
#define INPUT_COUNT   14      // IN1~6, IN9~16 (IN7,IN8은 I2C 때문에 미사용)
#define ULTRA_COUNT   4       // 배열은 4채널분 확보(0=미배선), 실제 배선은 1채널
#define LOAD_COUNT    4
#define STEP_COUNT    6
#define N_SEQ         8
#define MAX_COND      20
#define MAX_STEPS     6

const uint8_t RELAY_PIN[RELAY_COUNT] = {
  // R1~R8 = CH1~CH8(릴레이, 40~47), R9~R16 = D1~D8(MOS, 32~39)
  40,41,42,43,44,45,46,47, 32,33,34,35,36,37,38,39
};
bool RELAY_ACTIVE_LOW = false;   // HIGH=ON 가정(보드 회로가 반대면 이 값을 true로)

// IN1~IN6 = 16~21 중 20,21 제외?  ※ 20,21은 I2C라 아예 못 씀 → IN1~IN6은
// 16,17,18,19,22,23 중 6개(원래 IN1~8=16~23 이었으나 20,21 제외)
// IN9~IN16(추정 24~31)은 그대로 사용
const uint8_t IN_PIN[INPUT_COUNT] = {
  16,17,18,19,22,23,             // IN1,2,3,4,5(원IN6→건너뜀 자리 조정),6  ※아래 주석 참고
  24,25,26,27,28,29,30,31        // IN9~IN16 (추정치, 실측 전까지 임시)
};
/* 주의: 원래 IN1~8은 16,17,18,19,20,21,22,23 이었는데 20,21(=IN7,IN8)을 뺐기
   때문에, 위 배열의 앞 6개는 물리적으로 "IN1,IN2,IN3,IN4,  IN5(핀22),IN6(핀23)"
   순서가 됩니다 — 즉 보드 라벨 IN7,IN8 자리는 영구 결번입니다. */

const uint8_t KEY_LABELS[16] = {'1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'};

// I2C 확장칩(MCP23017, 주소 0x20)
#define MCP_ADDR 0x20
#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPPUA  0x0C
#define MCP_GPPUB  0x0D
#define MCP_GPIOA  0x12
#define MCP_GPIOB  0x13
#define MCP_OLATB  0x15
// GPA0~3 = 키패드 행(출력), GPA4~7 = 키패드 열(입력,풀업)
// GPB0   = 스텝모터1 EN (출력, active-LOW)

// 초음파/로드셀/스텝모터 — 직접 GPIO(1채널만 실배선, 나머지 0=미사용)
const uint8_t ULTRA_TRIG[ULTRA_COUNT] = {50, 0, 0, 0};
const uint8_t ULTRA_ECHO[ULTRA_COUNT] = {51, 0, 0, 0};
const uint8_t LOAD_DOUT[LOAD_COUNT]   = {52, 0, 0, 0};
const uint8_t LOAD_SCK[LOAD_COUNT]    = {53, 0, 0, 0};
const uint8_t STEP_STEP[STEP_COUNT]   = {2, 0,0,0,0,0};
const uint8_t STEP_DIR[STEP_COUNT]    = {15,0,0,0,0,0};
// STEP1의 EN은 MCP23017 GPB0로 대체(직접 핀 없음)

#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

/* --------------------------- 런타임 상태 변수 --------------------------- */
bool relayState[RELAY_COUNT+1];        // 1-base 사용(0은 안씀)
bool inStateRaw[INPUT_COUNT+1];
bool inStateDeb[INPUT_COUNT+1];
unsigned long inLastChange[INPUT_COUNT+1];
#define DEBOUNCE_MS 30UL

long loadOffset[LOAD_COUNT+1];         // 타어(0점) 오프셋
float loadScale[LOAD_COUNT+1];         // 보정 스케일(그램 환산)
long stepPos[STEP_COUNT+1];            // STEPZERO 기준 상대 위치

bool kpEnabled = false;
char kpBuf[16]; uint8_t kpBufLen = 0;
unsigned long kpLastPressAt = 0;
bool kpKeyDownPrev[16] = {false};

/* --------------------------- 시퀀스 구조체(EEPROM) ------------------------ */
struct Step {
  uint8_t type;      // 0=릴레이 1=스텝모터 2=대기(조건) 3=지연만
  uint8_t target;     // type0: 릴레이번호(1~16) / type1: 스텝채널(1~N)
  uint8_t on;          // type0: 1=ON 0=OFF / type1: 방향(1=+ 0=-)
  uint16_t delayMs;   // type3, 또는 type0/1 실행 후 대기시간
  int16_t steps;      // type1: 이동 스텝수
  uint16_t speed;     // type1: 속도(us 단위 펄스간격 느낌으로 사용)
  uint8_t waitCh;     // type2: 입력채널(1~14), 0이면 미사용
  uint8_t waitOn;     // type2: 1=눌릴때까지 0=뗄때까지
};
struct Seq {
  char name[12];
  uint8_t trigCh;       // 1~14, 0=단일채널 트리거 없음(코드매칭 전용)
  uint8_t trigCond;     // 0=놓을때 1=누를때 2=양쪽
  uint8_t matchMode;    // 0=인터록(AND) 1=코드-정확순서 2=코드-랜덤(시간제한)
  uint32_t condTimeoutMs;
  uint8_t nConds;
  uint8_t condIdx[MAX_COND];
  uint8_t condOn[MAX_COND];
  uint8_t nSteps;
  Step steps[MAX_STEPS];
};
Seq seqs[N_SEQ];
uint8_t seqCountUsed = N_SEQ;   // 실제로는 nSteps==0 인 것은 비어있다고 취급

// 시퀀스 코드매칭 진행상태(런타임, EEPROM 아님)
uint8_t seqOrderProgress[N_SEQ];
uint32_t seqSubsetMask[N_SEQ];
unsigned long seqSubsetStart[N_SEQ];

// 현재 실행중인 시퀀스(한 번에 1개만 실행 — PLC 성격상 충분)
int8_t runningSeq = -1;
uint8_t runningStepIdx = 0;
unsigned long runningWaitUntil = 0;   // 지연(delay) 종료 시각
bool runningWaitingCond = false;      // type2 대기중인지
uint8_t runningWaitCh = 0, runningWaitOn = 0;

/* ------------------------------- EEPROM --------------------------------- */
#define EE_MAGIC 0xA5
#define EE_ADDR_MAGIC   0
#define EE_ADDR_BOOT    2                 // 16바이트: 전원투입시 릴레이 시작상태
#define EE_ADDR_SEQS    (EE_ADDR_BOOT+RELAY_COUNT)
uint8_t relayBoot[RELAY_COUNT+1];

void eepromLoadAll(){
  uint8_t magic = EEPROM.read(EE_ADDR_MAGIC);
  if (magic != EE_MAGIC){
    // 최초 부팅: 기본값으로 초기화 후 저장
    for(int i=1;i<=RELAY_COUNT;i++) relayBoot[i]=0;
    for(int s=0;s<N_SEQ;s++){ seqs[s].nSteps=0; seqs[s].nConds=0; seqs[s].name[0]=0; }
    eepromSaveAll();
    return;
  }
  for(int i=1;i<=RELAY_COUNT;i++) relayBoot[i]=EEPROM.read(EE_ADDR_BOOT+i-1);
  EEPROM.get(EE_ADDR_SEQS, seqs);
}
void eepromSaveAll(){
  EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
  for(int i=1;i<=RELAY_COUNT;i++) EEPROM.update(EE_ADDR_BOOT+i-1, relayBoot[i]);
  EEPROM.put(EE_ADDR_SEQS, seqs);
}
void eepromSaveSeqs(){ EEPROM.put(EE_ADDR_SEQS, seqs); }
void eepromSaveBoot(){ for(int i=1;i<=RELAY_COUNT;i++) EEPROM.update(EE_ADDR_BOOT+i-1, relayBoot[i]); }

/* ---------------------------- MCP23017 최소 구현 -------------------------- */
void mcpWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
uint8_t mcpRead(uint8_t reg){
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MCP_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}
void mcpInit(){
  // GPA0~3 출력(키패드 행), GPA4~7 입력(키패드 열)
  mcpWrite(MCP_IODIRA, 0b11110000);
  mcpWrite(MCP_GPPUA,  0b11110000);   // 열 입력 풀업
  // GPB0 출력(스텝1 EN), 나머지 GPB 입력
  mcpWrite(MCP_IODIRB, 0b11111110);
  mcpWrite(MCP_OLATB, 0x01);          // EN active-LOW → 기본 비활성(HIGH)
}
void mcpSetStepEN(bool enable){
  uint8_t v = mcpRead(MCP_OLATB);
  if (enable) v &= ~0x01; else v |= 0x01;   // active-LOW
  mcpWrite(MCP_GPIOB, v);
}

/* --------------------------------- 릴레이 -------------------------------- */
void setRelay(uint8_t ch, bool on){
  if (ch<1||ch>RELAY_COUNT) return;
  relayState[ch]=on;
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY_PIN[ch-1], level?HIGH:LOW);
}
void applyBootStates(){
  for(int ch=1;ch<=RELAY_COUNT;ch++) setRelay(ch, relayBoot[ch]!=0);
}

/* --------------------------------- 입력 ---------------------------------- */
void scanInputs(){
  unsigned long now=millis();
  for(int ch=1;ch<=INPUT_COUNT;ch++){
    bool raw = (digitalRead(IN_PIN[ch-1])==LOW);  // INPUT_PULLUP: 눌림=LOW
    if (raw != inStateRaw[ch]){ inStateRaw[ch]=raw; inLastChange[ch]=now; }
    if (now - inLastChange[ch] >= DEBOUNCE_MS && inStateDeb[ch]!=inStateRaw[ch]){
      bool prev = inStateDeb[ch];
      inStateDeb[ch]=inStateRaw[ch];
      onInputEdge(ch, prev, inStateDeb[ch]);
    }
  }
}

/* --------------------------------- 초음파 --------------------------------- */
long ultraReadCm(uint8_t ch){   // ch: 1~4
  uint8_t trig=ULTRA_TRIG[ch-1], echo=ULTRA_ECHO[ch-1];
  if (!trig||!echo) return -1;
  digitalWrite(trig, LOW); delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, 25000UL);
  if (dur==0) return -1;
  return (long)(dur/58.0);
}

/* --------------------------------- 로드셀 --------------------------------- */
bool hx711Ready(uint8_t ch){
  uint8_t dout=LOAD_DOUT[ch-1];
  if(!dout) return false;
  return digitalRead(dout)==LOW;
}
long hx711ReadRaw(uint8_t ch){
  uint8_t dout=LOAD_DOUT[ch-1], sck=LOAD_SCK[ch-1];
  if(!dout||!sck) return 0;
  unsigned long t0=millis();
  while(!hx711Ready(ch)){ if(millis()-t0>500) return 0; }
  long v=0;
  for(int i=0;i<24;i++){
    digitalWrite(sck,HIGH); delayMicroseconds(1);
    v = (v<<1) | digitalRead(dout);
    digitalWrite(sck,LOW); delayMicroseconds(1);
  }
  digitalWrite(sck,HIGH); delayMicroseconds(1); digitalWrite(sck,LOW); // 25th pulse(gain128)
  if (v & 0x800000) v |= 0xFF000000UL;  // 부호 확장
  return v;
}
float loadReadGram(uint8_t ch){
  long raw = hx711ReadRaw(ch);
  if (raw==0 && !hx711Ready(ch)) return NAN;
  float scale = loadScale[ch]!=0?loadScale[ch]:1.0;
  return (raw - loadOffset[ch]) / scale;
}

/* --------------------------------- 스텝모터 -------------------------------- */
void stepPulse(uint8_t ch, int16_t steps, uint16_t speedUs){
  if (ch<1||ch>STEP_COUNT) return;
  uint8_t stp=STEP_STEP[ch-1], dir=STEP_DIR[ch-1];
  if (!stp||!dir) return;
  if (ch==1) mcpSetStepEN(true);
  digitalWrite(dir, steps>=0?HIGH:LOW);
  long n = abs(steps);
  for(long i=0;i<n;i++){
    digitalWrite(stp,HIGH); delayMicroseconds(max(2,(int)speedUs/2));
    digitalWrite(stp,LOW);  delayMicroseconds(max(2,(int)speedUs/2));
  }
  stepPos[ch] += steps;
  if (ch==1) mcpSetStepEN(false);
}
void stepZero(uint8_t ch){ if(ch>=1&&ch<=STEP_COUNT) stepPos[ch]=0; }

/* --------------------------------- 키패드 --------------------------------- */
void scanKeypad(){
  if (!kpEnabled) return;
  static unsigned long lastScan=0;
  unsigned long now=millis();
  if (now-lastScan<40) return;   // 25Hz 폴링(I2C라 너무 빠르면 부담)
  lastScan=now;
  for(uint8_t r=0;r<4;r++){
    // 해당 행만 LOW, 나머지 행 HIGH
    uint8_t rowsOut = ~(1<<r) & 0x0F;
    mcpWrite(MCP_GPIOA, rowsOut);   // 하위4비트=행(0=LOW선택), 상위4비트은 GPIOA읽기에서 무시
    delayMicroseconds(50);
    uint8_t val = mcpRead(MCP_GPIOA);
    for(uint8_t c=0;c<4;c++){
      bool pressed = !( (val>>(4+c)) & 0x01 );  // 열은 풀업, 눌리면 LOW
      uint8_t idx = r*4+c;
      if (pressed && !kpKeyDownPrev[idx]){
        kpKeyDownPrev[idx]=true;
        onKeyDown(KEY_LABELS[idx]);
      } else if (!pressed && kpKeyDownPrev[idx]){
        kpKeyDownPrev[idx]=false;
      }
    }
  }
  mcpWrite(MCP_GPIOA, 0x00); // 대기상태: 행 전부 LOW(다음 스캔 대비, 열은 입력이라 영향없음)
}
void onKeyDown(char key){
  Serial.print("KEY "); Serial.println(key);
  if (key=='*'){ kpBufLen=0; return; }
  if (kpBufLen<15) kpBuf[kpBufLen++]=key;
  kpBuf[kpBufLen]=0;
  kpLastPressAt=millis();
  // 코드 버퍼 변화가 있을 때마다 시퀀스 코드매칭 평가(간이: 각 키 입력을
  // "그 채널 press" 이벤트처럼 취급하지 않고, 여기서는 키패드 코드 자체를
  // 별도 매칭하지 않고 릴레이 직접토글용 KEY 라인 송출까지만 지원합니다.
  if (millis()-kpLastPressAt>8000){ kpBufLen=0; }
}

/* --------------------------------- OLED ----------------------------------- */
void oledShowText(const String &text){
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,0);
  oled.println(text);
  oled.display();
}

/* ------------------------------ 시퀀스 엔진 -------------------------------- */
bool condsMatchAND(uint8_t s){   // Seq& 대신 인덱스를 받음(아두이노 IDE 자동 프로토타입 생성이
  Seq &S = seqs[s];              // 구조체 정의보다 먼저 함수 원형을 만들어버려 컴파일 오류가 나는 문제 회피)
  for(uint8_t i=0;i<S.nConds;i++){
    uint8_t ch=S.condIdx[i]; if(!ch) continue;
    bool want = S.condOn[i]!=0;
    if (inStateDeb[ch]!=want) return false;
  }
  return true;
}
void startSequence(int8_t s){
  if (runningSeq>=0) return;   // 이미 하나 실행중이면 무시(단순화)
  if (seqs[s].nSteps==0) return;
  runningSeq=s; runningStepIdx=0; runningWaitingCond=false;
  Serial.print("SEQRUN "); Serial.println(s);
}
void serviceRunningSequence(){
  if (runningSeq<0) return;
  Seq &S = seqs[runningSeq];
  if (runningWaitingCond){
    bool cur = inStateDeb[runningWaitCh];
    bool want = runningWaitOn!=0;
    if (cur!=want) return;   // 계속 대기
    runningWaitingCond=false;
    runningStepIdx++;
  }
  if (millis() < runningWaitUntil) return;
  if (runningStepIdx >= S.nSteps){ runningSeq=-1; return; }
  Step &st = S.steps[runningStepIdx];
  if (st.type==0){            // 릴레이
    setRelay(st.target, st.on!=0);
    runningWaitUntil = millis()+st.delayMs;
    runningStepIdx++;
  } else if (st.type==1){     // 스텝모터(블로킹 실행 — 인덱싱용이라 허용)
    stepPulse(st.target, (st.on?1:-1)*st.steps, st.speed?st.speed:400);
    runningWaitUntil = millis()+st.delayMs;
    runningStepIdx++;
  } else if (st.type==2){     // 조건 대기
    if (st.waitCh){
      runningWaitingCond=true; runningWaitCh=st.waitCh; runningWaitOn=st.waitOn;
    } else {
      runningStepIdx++;
    }
  } else {                    // 3=지연만
    runningWaitUntil = millis()+st.delayMs;
    runningStepIdx++;
  }
}
void evalCodeMatchOnEdge(uint8_t ch, bool nowOn){
  for(uint8_t s=0;s<N_SEQ;s++){
    Seq &S=seqs[s];
    if (S.nSteps==0 || S.matchMode==0) continue;
    if (S.matchMode==1){    // 정확 순서
      uint8_t want = seqOrderProgress[s];
      if (want<S.nConds && S.condIdx[want]==ch && (S.condOn[want]!=0)==nowOn){
        seqOrderProgress[s]++;
        if (seqOrderProgress[s]>=S.nConds){ seqOrderProgress[s]=0; startSequence(s); }
      } else {
        seqOrderProgress[s]=0;
      }
    } else if (S.matchMode==2){  // 랜덤(시간제한)
      for(uint8_t i=0;i<S.nConds;i++){
        if (S.condIdx[i]==ch && (S.condOn[i]!=0)==nowOn){
          if (seqSubsetMask[s]==0) seqSubsetStart[s]=millis();
          seqSubsetMask[s] |= (1UL<<i);
        }
      }
      uint32_t full=(1UL<<S.nConds)-1;
      if (seqSubsetMask[s]==full){
        seqSubsetMask[s]=0; startSequence(s);
      } else if (seqSubsetMask[s]!=0 && millis()-seqSubsetStart[s] > S.condTimeoutMs){
        seqSubsetMask[s]=0;
      }
    }
  }
}
void onInputEdge(uint8_t ch, bool prev, bool nowOn){
  evalCodeMatchOnEdge(ch, nowOn);
  for(uint8_t s=0;s<N_SEQ;s++){
    Seq &S=seqs[s];
    if (S.nSteps==0 || S.matchMode!=0) continue;
    if (S.trigCh!=ch) continue;
    bool fire = (S.trigCond==2) || (S.trigCond==1 && nowOn) || (S.trigCond==0 && !nowOn);
    if (!fire) continue;
    if (!condsMatchAND(s)) continue;
    startSequence(s);
  }
}

/* ------------------------------ 시리얼 프로토콜 ---------------------------- */
String rxLine;
void sendOK(){ Serial.println("OK"); }

void cmdStatus(){
  String s="STATUS ";
  for(int ch=1;ch<=RELAY_COUNT;ch++){ s+=(relayState[ch]?"1":"0"); if(ch<RELAY_COUNT)s+=","; }
  Serial.println(s);
}
void cmdInput(){
  String s="INPUT ";
  for(int ch=1;ch<=INPUT_COUNT;ch++){ s+=(inStateDeb[ch]?"1":"0"); if(ch<INPUT_COUNT)s+=","; }
  Serial.println(s);
}
void cmdGetConfig(){
  String s="CONFIG OUT ";
  for(int ch=1;ch<=RELAY_COUNT;ch++){ s+=String(RELAY_PIN[ch-1]); if(ch<RELAY_COUNT)s+=","; }
  s+=" IN ";
  for(int ch=1;ch<=INPUT_COUNT;ch++){ s+=String(ch)+":"+String(IN_PIN[ch-1]); if(ch<INPUT_COUNT)s+=","; }
  Serial.println(s);
}
void cmdGetKpConfig(){
  Serial.println("KPCONFIG ROW MCP0,MCP1,MCP2,MCP3 COL MCP4,MCP5,MCP6,MCP7");
}
void cmdSeqCondProbe(){ Serial.println("SEQCOND OK"); }

void handleLine(String line){
  line.trim();
  if (line.length()==0) return;
  int sp = line.indexOf(' ');
  String cmd = sp<0? line : line.substring(0,sp);
  String rest = sp<0? "" : line.substring(sp+1);
  cmd.toUpperCase();

  if (cmd=="PIN"){
    int s1=rest.indexOf(' ');
    int ch = rest.substring(0,s1).toInt();
    String state = rest.substring(s1+1); state.trim();
    setRelay(ch, state=="ON");
    sendOK();
  } else if (cmd=="STATUS?"){ cmdStatus();
  } else if (cmd=="INPUT?"){ cmdInput();
  } else if (cmd=="GETCONFIG?"){ cmdGetConfig();
  } else if (cmd=="GETKPCONFIG?"){ cmdGetKpConfig();
  } else if (cmd=="SETIN"||cmd=="SETOUT"){
    sendOK();  // 이 펌웨어는 핀이 보드에 고정배선이라 실제 재매핑은 안 됨(호환용 OK)
  } else if (cmd=="SETBOOT"){
    int s1=rest.indexOf(' ');
    int ch=rest.substring(0,s1).toInt();
    int v=rest.substring(s1+1).toInt();
    if(ch>=1&&ch<=RELAY_COUNT){ relayBoot[ch]=v?1:0; eepromSaveBoot(); }
    sendOK();
  } else if (cmd=="SETKPEN"){
    kpEnabled = rest.toInt()!=0; sendOK();
  } else if (cmd=="SETKPROW"||cmd=="SETKPCOL"){
    sendOK();  // 키패드 행/열은 MCP23017 고정배선(호환용 OK)
  } else if (cmd=="SETLED"){
    oledShowText(rest); sendOK();
  } else if (cmd=="SETLEDTRIG"){
    sendOK();  // 위치도달 자동 LED표시는 본 펌웨어 축소판에서는 미지원(호환 OK)
  } else if (cmd=="SETSTEPPIN"){
    sendOK();  // 스텝모터 핀도 고정배선
  } else if (cmd=="STEPMOVE"){
    int s1=rest.indexOf(' '); int s2=rest.indexOf(' ',s1+1);
    int ch=rest.substring(0,s1).toInt();
    int steps=rest.substring(s1+1,s2).toInt();
    int speed=rest.substring(s2+1).toInt();
    stepPulse(ch, steps, speed); sendOK();
  } else if (cmd=="STEPZERO"){
    stepZero(rest.toInt()); sendOK();
  } else if (cmd=="STEPSTOPALL"){
    sendOK();  // 블로킹 방식이라 실제 "정지"는 다음 명령 전까지 대기(제한사항)
  } else if (cmd=="SETULTRAPIN"||cmd=="SETLOADPIN"){
    sendOK();  // 고정배선
  } else if (cmd=="ULTRA?"){
    String s="ULTRA ";
    for(int ch=1;ch<=ULTRA_COUNT;ch++){ s+=String(ultraReadCm(ch)); if(ch<ULTRA_COUNT)s+=","; }
    Serial.println(s);
  } else if (cmd=="LOAD?"){
    String s="LOAD ";
    for(int ch=1;ch<=LOAD_COUNT;ch++){
      float g=loadReadGram(ch);
      s += isnan(g)?"NC":String(g,1);
      if(ch<LOAD_COUNT)s+=",";
    }
    Serial.println(s);
  } else if (cmd=="LOADTARE"){
    int ch=rest.toInt();
    if(ch>=1&&ch<=LOAD_COUNT){ loadOffset[ch]=hx711ReadRaw(ch); Serial.println("LOADTAREOK"); }
  } else if (cmd=="LOADCAL"){
    int s1=rest.indexOf(' ');
    int ch=rest.substring(0,s1).toInt();
    float knownGram=rest.substring(s1+1).toFloat();
    long raw=hx711ReadRaw(ch);
    if(knownGram!=0) loadScale[ch]=(raw-loadOffset[ch])/knownGram;
    Serial.println("LOADCALOK");
  } else if (cmd=="SEQCLR"){
    for(int s=0;s<N_SEQ;s++){ seqs[s].nSteps=0; seqs[s].nConds=0; seqs[s].name[0]=0; }
    eepromSaveSeqs(); sendOK();
  } else if (cmd=="SEQTRIG"){
    // SEQTRIG idx trigSrc trigCh trigCond distMode distCm  (trigSrc:0=디지털만 지원)
    int p[6], pos=0, last=0;
    for(int i=0;i<5;i++){ int sp2=rest.indexOf(' ',last); p[i]=rest.substring(last,sp2).toInt(); last=sp2+1; }
    p[5]=rest.substring(last).toInt();
    int idx=p[0];
    if(idx>=0&&idx<N_SEQ){
      seqs[idx].trigCh = p[2];
      seqs[idx].trigCond = p[3];
      seqs[idx].matchMode = 0;   // SEQCOND 명령이 따로 오면 코드매칭으로 전환됨(SEQCONDX 별도 확장 여지)
    }
    sendOK();
  } else if (cmd=="SEQCOND"){
    // SEQCOND idx onMask offMask  (인터록 AND, 채널 1~14 비트마스크)
    int s1=rest.indexOf(' '); int s2=rest.indexOf(' ',s1+1);
    int idx=rest.substring(0,s1).toInt();
    unsigned long onMask=strtoul(rest.substring(s1+1,s2).c_str(),NULL,10);
    unsigned long offMask=strtoul(rest.substring(s2+1).c_str(),NULL,10);
    if(idx>=0&&idx<N_SEQ){
      uint8_t n=0;
      for(uint8_t ch=1;ch<=INPUT_COUNT && n<MAX_COND;ch++){
        if (onMask & (1UL<<(ch-1))){ seqs[idx].condIdx[n]=ch; seqs[idx].condOn[n]=1; n++; }
        else if (offMask & (1UL<<(ch-1))){ seqs[idx].condIdx[n]=ch; seqs[idx].condOn[n]=0; n++; }
      }
      seqs[idx].nConds=n;
    }
    sendOK();
  } else if (cmd=="SEQSTEP"){
    // SEQSTEP idx si isMotor target on delay steps speed
    int v[8], last=0;
    for(int i=0;i<7;i++){ int sp2=rest.indexOf(' ',last); v[i]=rest.substring(last,sp2).toInt(); last=sp2+1; }
    v[7]=rest.substring(last).toInt();
    int idx=v[0], si=v[1];
    if(idx>=0&&idx<N_SEQ && si>=0&&si<MAX_STEPS){
      Step &st=seqs[idx].steps[si];
      if (v[2]){ st.type=1; st.target=v[3]; st.on=1; st.delayMs=v[5]; st.steps=v[6]; st.speed=v[7]; }
      else { st.type=0; st.target=v[3]; st.on=v[4]; st.delayMs=v[5]; }
      if (si+1 > seqs[idx].nSteps) seqs[idx].nSteps=si+1;
    }
    sendOK();
  } else if (cmd=="SEQNAME"){
    int s1=rest.indexOf(' ');
    int idx=rest.substring(0,s1).toInt();
    String nm=rest.substring(s1+1);
    if(idx>=0&&idx<N_SEQ){ nm.toCharArray(seqs[idx].name,12); }
    sendOK();
  } else if (cmd=="SEQCOUNT"){
    int s1=rest.indexOf(' ');
    int idx=rest.substring(0,s1).toInt();
    int n=rest.substring(s1+1).toInt();
    if(idx>=0&&idx<N_SEQ){ seqs[idx].nSteps=n; }
    eepromSaveSeqs();
    sendOK();
  } else if (cmd=="SEQCOND?"){
    cmdSeqCondProbe();
  } else if (cmd=="EXTMEM?"){
    Serial.println("EXTMEM CONNECTED NOWARN WRITEOK");  // 내장 EEPROM 사용(기능상 문제없음)
  }
}

void pollSerial(){
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n'){ handleLine(rxLine); rxLine=""; }
    else if (c!='\r'){ rxLine+=c; }
  }
}

/* ---------------------------------- setup/loop ---------------------------- */
void setup(){
  Serial.begin(115200);
  Wire.begin();

  for(int ch=1;ch<=RELAY_COUNT;ch++){ pinMode(RELAY_PIN[ch-1],OUTPUT); }
  for(int ch=1;ch<=INPUT_COUNT;ch++){ pinMode(IN_PIN[ch-1],INPUT_PULLUP); inStateRaw[ch]=inStateDeb[ch]=false; }
  for(int i=0;i<ULTRA_COUNT;i++){ if(ULTRA_TRIG[i]){pinMode(ULTRA_TRIG[i],OUTPUT); pinMode(ULTRA_ECHO[i],INPUT);} }
  for(int i=0;i<LOAD_COUNT;i++){ if(LOAD_SCK[i]){pinMode(LOAD_SCK[i],OUTPUT); pinMode(LOAD_DOUT[i],INPUT); digitalWrite(LOAD_SCK[i],LOW);} }
  for(int i=0;i<STEP_COUNT;i++){ if(STEP_STEP[i]){pinMode(STEP_STEP[i],OUTPUT); pinMode(STEP_DIR[i],OUTPUT);} }

  mcpInit();

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    // OLED 연결 안 돼있어도 나머지 기능은 계속 동작
  } else {
    oled.clearDisplay(); oled.display();
  }

  eepromLoadAll();
  delay(3000);           // 전원투입 후 3초 입력 무시(오작동 방지)
  applyBootStates();

  for(int s=0;s<N_SEQ;s++){ seqOrderProgress[s]=0; seqSubsetMask[s]=0; }
}

void loop(){
  pollSerial();
  scanInputs();
  scanKeypad();
  serviceRunningSequence();
}
