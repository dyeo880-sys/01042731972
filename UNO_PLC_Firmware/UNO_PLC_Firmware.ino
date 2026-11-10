// ============================================================
//  UNO_PLC_Firmware.ino  v2.0   (Arduino UNO R3 / 115200bps)
//  UNO.html (UNO PLC 제어 프로그램)과 시리얼로 통신하는 보드용 펌웨어
//
//  - 릴레이 R1~R4  : 핀 3,4,5,6   (기본 active-LOW: LOW = ON)
//  - 입력   CH1~12 : 핀 7,8,9,10,11,12,2,13,A0,A1,A2,A3
//                    INPUT_PULLUP, 버튼이 GND로 붙으면 "눌림"
//  - 핀 0,1(RX/TX)은 USB 통신용이므로 절대 배선하지 마세요.
//  - 시퀀스 최대 12개 / 시퀀스당 조건 4개 / 동작 10줄, EEPROM에 저장
//  - 모드 0 = PC 제어(보드는 시퀀스를 실행하지 않음)
//    모드 2 = 시퀀스(저장된 시퀀스를 보드가 단독 실행, 컴퓨터 없이 동작)
//
//  통신 명령 (줄 끝 CR/LF):
//    ID? GETCONFIG? GETMODE? SETMODE m STATUS? INPUT?
//    PIN n ON|OFF            (응답 없음)
//    GETINV? SETINV ch v     GETBOOT? SETBOOT r v   GETOUTINV? SETOUTINV r v
//    GETSEQORDER? SETSEQORDER v
//    SEQCLR SEQDEF SEQCOND SEQSTEP SEQSAVE SEQABORT SEQGET?
// ============================================================
#include <EEPROM.h>

#define FW_NAME "UNO_PLC"
#define FW_VER  "2.0"

#define N_OUT      4
#define N_IN       12
#define MAX_SEQ    12
#define MAX_STEPS  10
#define MAX_CONDS  4

#define DEBOUNCE_MS  30UL
#define BOOT_HOLD_MS 3000UL

const uint8_t OUT_PIN[N_OUT] = {3, 4, 5, 6};
const uint8_t IN_PIN[N_IN]   = {7, 8, 9, 10, 11, 12, 2, 13, A0, A1, A2, A3};

// ---------------- 시퀀스 저장 구조 (EEPROM에 그대로 기록) ----------------
struct Step {
  uint8_t  b;   // bit0-1: 릴레이-1, bit2: ON, bit3: 대기 상태(1=눌림), bit4-7: 대기 채널(0=대기 안 함)
  uint16_t d;   // 지연(ms)
};
struct Seq {
  uint8_t trigCh;     // 0 = 자동 시작 없음, 1~12 = CH
  uint8_t trigCond;   // 0 = 놓을 때, 1 = 누를 때, 2 = 양쪽, 3 = 누르고 있는 동안에만
  uint8_t condMode;   // 0 = AND(모두 만족), 1 = OR(하나라도 만족)
  uint8_t nConds;
  uint8_t nSteps;
  uint8_t cond[MAX_CONDS];   // bit0-6: 채널, bit7: 요구 상태(1=눌림)
  Step    st[MAX_STEPS];
};

static inline uint8_t stRelay(const Step &s)  { return (s.b & 3) + 1; }
static inline bool    stOn(const Step &s)     { return (s.b >> 2) & 1; }
static inline bool    stWaitOn(const Step &s) { return (s.b >> 3) & 1; }
static inline uint8_t stWaitCh(const Step &s) { return s.b >> 4; }

// ---------------- EEPROM 배치 ----------------
#define EA_MAGIC   0
#define EA_MODE    1
#define EA_ORDER   2
#define EA_NSEQ    3
#define EA_INV     4     // 12바이트
#define EA_BOOT    16    // 4바이트
#define EA_OUTINV  20    // 4바이트
#define EA_SEQ     32    // Seq x 12
#define MAGIC      0xA7

// ---------------- 상태 ----------------
uint8_t mode = 0;
bool    seqOrderMode = false;
uint8_t inInv[N_IN], outBoot[N_OUT], outInv[N_OUT];
Seq     seqs[MAX_SEQ];
uint8_t nSeq = 0;
bool    staging = false;          // 시퀀스를 받는 중에는 보드 실행을 멈춤

bool relayOn[N_OUT];
bool inLast[N_IN], inStable[N_IN];
unsigned long inChangedAt[N_IN];

bool sqRun[MAX_SEQ], sqPrev[MAX_SEQ], sqCondWait[MAX_SEQ], sqWaitBase[MAX_SEQ];
uint8_t sqStep[MAX_SEQ];
unsigned long sqDue[MAX_SEQ];
uint8_t seqOrderIdx = 0;
unsigned long bootAt = 0;

char    lineBuf[64];
uint8_t lineLen = 0;

// ---------------- 릴레이/입력 ----------------
void setRelay(uint8_t ch, bool on) {          // ch = 1~4
  if (ch < 1 || ch > N_OUT) return;
  relayOn[ch - 1] = on;
  bool activeLow = (outInv[ch - 1] == 0);      // 출력 반전이 켜지면 active-HIGH
  digitalWrite(OUT_PIN[ch - 1], (on == activeLow) ? LOW : HIGH);
}

void applyBootState() {
  for (uint8_t i = 0; i < N_OUT; i++) setRelay(i + 1, outBoot[i] != 0);
}

void updateInputs() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < N_IN; i++) {
    bool raw = ((digitalRead(IN_PIN[i]) == LOW) != (inInv[i] != 0));
    if (raw != inLast[i]) { inLast[i] = raw; inChangedAt[i] = now; }
    else if (raw != inStable[i] && (now - inChangedAt[i]) >= DEBOUNCE_MS) inStable[i] = raw;
  }
}

// ---------------- EEPROM ----------------
void loadAll() {
  if (EEPROM.read(EA_MAGIC) != MAGIC) {       // 처음 쓰는 보드: 기본값으로 초기화
    EEPROM.update(EA_MAGIC, MAGIC);
    EEPROM.update(EA_MODE, 0);
    EEPROM.update(EA_ORDER, 0);
    EEPROM.update(EA_NSEQ, 0);
    for (uint8_t i = 0; i < N_IN; i++)  EEPROM.update(EA_INV + i, 0);
    for (uint8_t i = 0; i < N_OUT; i++) { EEPROM.update(EA_BOOT + i, 0); EEPROM.update(EA_OUTINV + i, 0); }
  }
  mode = EEPROM.read(EA_MODE); if (mode > 2) mode = 0;
  seqOrderMode = (EEPROM.read(EA_ORDER) == 1);
  for (uint8_t i = 0; i < N_IN; i++)  inInv[i]   = EEPROM.read(EA_INV + i) ? 1 : 0;
  for (uint8_t i = 0; i < N_OUT; i++) { outBoot[i] = EEPROM.read(EA_BOOT + i) ? 1 : 0; outInv[i] = EEPROM.read(EA_OUTINV + i) ? 1 : 0; }
  loadSeqs();
}

void loadSeqs() {
  memset(seqs, 0, sizeof(seqs));
  nSeq = EEPROM.read(EA_NSEQ); if (nSeq > MAX_SEQ) nSeq = 0;
  for (uint8_t i = 0; i < nSeq; i++) {
    EEPROM.get(EA_SEQ + i * sizeof(Seq), seqs[i]);
    // 손상된 값 방어
    if (seqs[i].nSteps > MAX_STEPS) seqs[i].nSteps = MAX_STEPS;
    if (seqs[i].nConds > MAX_CONDS) seqs[i].nConds = MAX_CONDS;
    if (seqs[i].trigCh > N_IN) seqs[i].trigCh = 0;
  }
}

void saveSeqs() {
  EEPROM.update(EA_NSEQ, nSeq);
  for (uint8_t i = 0; i < nSeq; i++) EEPROM.put(EA_SEQ + i * sizeof(Seq), seqs[i]);   // put은 바뀐 바이트만 기록
}

// ---------------- 시퀀스 실행 ----------------
void stopAllSeq() {
  for (uint8_t s = 0; s < MAX_SEQ; s++) { sqRun[s] = false; sqCondWait[s] = false; sqWaitBase[s] = false; sqStep[s] = 0; }
  seqOrderIdx = 0;
}

bool interlockOK(const Seq &S) {
  if (S.nConds == 0) return true;
  if (S.condMode == 1) {                         // 하나라도 만족(OR)
    for (uint8_t c = 0; c < S.nConds; c++) {
      uint8_t ch = S.cond[c] & 0x7F;
      if (ch >= 1 && ch <= N_IN && inStable[ch - 1] == ((S.cond[c] & 0x80) != 0)) return true;
    }
    return false;
  }
  for (uint8_t c = 0; c < S.nConds; c++) {       // 모두 만족(AND)
    uint8_t ch = S.cond[c] & 0x7F;
    if (ch >= 1 && ch <= N_IN && inStable[ch - 1] != ((S.cond[c] & 0x80) != 0)) return false;
  }
  return true;
}

void startSeq(uint8_t s) {
  if (s >= nSeq || sqRun[s] || seqs[s].nSteps == 0) return;
  sqRun[s] = true; sqStep[s] = 0; sqCondWait[s] = false; sqDue[s] = millis();
}

void runSeqs() {
  unsigned long now = millis();
  bool enabled = (mode == 2) && !staging;
  bool holding = (now - bootAt) < BOOT_HOLD_MS;   // 전원/모드 시작 직후 3초간은 입력 무시
  for (uint8_t s = 0; s < nSeq; s++) {
    const Seq &S = seqs[s];
    if (S.trigCh >= 1 && S.trigCh <= N_IN) {
      bool cur = inStable[S.trigCh - 1];
      if (!enabled || holding) sqPrev[s] = cur;
      else if (cur != sqPrev[s]) {
        sqPrev[s] = cur;
        bool fire = cur ? (S.trigCond == 1 || S.trigCond == 2 || S.trigCond == 3)
                        : (S.trigCond == 0 || S.trigCond == 2);
        bool orderOK = !seqOrderMode || (s == seqOrderIdx);
        if (fire && interlockOK(S) && orderOK) startSeq(s);
        if (!cur && S.trigCond == 3) {            // 누르고 있는 동안에만: 떼면 즉시 중단 + 관련 릴레이 OFF
          sqRun[s] = false; sqCondWait[s] = false;
          for (uint8_t k = 0; k < S.nSteps; k++) setRelay(stRelay(S.st[k]), false);
        }
      }
    }
    if (!enabled) continue;
    while (sqRun[s]) {
      uint8_t r = sqStep[s];
      const Step &st = S.st[r];
      if (sqCondWait[s]) {                          // 릴레이 동작은 끝났고 조건만 기다리는 중
        bool curW = inStable[stWaitCh(st) - 1];
        if (curW != stWaitOn(st) || curW == sqWaitBase[s]) break;   // 아직 안 맞음(또는 실제로 바뀐 적 없음)
        sqCondWait[s] = false;
      } else {
        if ((long)(now - sqDue[s]) < 0) break;      // 지연 시간이 아직 안 지남
        setRelay(stRelay(st), stOn(st));
        if (stWaitCh(st) != 0) { sqCondWait[s] = true; sqWaitBase[s] = inStable[stWaitCh(st) - 1]; break; }
        sqDue[s] = now + st.d;
      }
      sqStep[s] = r + 1;
      if (sqStep[s] >= S.nSteps) {
        sqRun[s] = false;
        if (seqOrderMode && s == seqOrderIdx) seqOrderIdx = (uint8_t)((s + 1 < nSeq) ? (s + 1) : 0);
        break;
      }
      if (stWaitCh(st) == 0 && st.d > 0) break;     // 지연이 0이면 같은 순간에 여러 줄이 연달아 실행됨
    }
  }
}

// ---------------- 명령 처리 ----------------
void replyOK()  { Serial.println(F("OK")); }
void replyERR() { Serial.println(F("ERR BAD")); }

void printList(const __FlashStringHelper *head, const uint8_t *a, uint8_t n) {
  Serial.print(head);
  for (uint8_t i = 0; i < n; i++) { if (i) Serial.print(','); Serial.print(a[i] ? 1 : 0); }
  Serial.println();
}

void execCmd(char *line) {
  for (char *p = line; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  char *sp = NULL;
  char *cmd = strtok_r(line, " ", &sp);
  if (!cmd) return;
  long a[7]; uint8_t na = 0;
  char *t;
  while (na < 7 && (t = strtok_r(NULL, " ", &sp)) != NULL) {
    if (!strcmp(t, "ON")) a[na++] = 1; else if (!strcmp(t, "OFF")) a[na++] = 0; else a[na++] = atol(t);
  }

  if (!strcmp(cmd, "ID?")) {
    Serial.println(F("ID " FW_NAME " " FW_VER));
  } else if (!strcmp(cmd, "PIN")) {                       // 응답 없음
    if (na >= 2 && a[0] >= 1 && a[0] <= N_OUT) setRelay((uint8_t)a[0], a[1] != 0);
  } else if (!strcmp(cmd, "STATUS?")) {
    Serial.print(F("STATUS "));
    for (uint8_t i = 0; i < N_OUT; i++) { if (i) Serial.print(','); Serial.print(relayOn[i] ? 1 : 0); }
    Serial.println();
  } else if (!strcmp(cmd, "INPUT?")) {
    Serial.print(F("INPUT "));
    for (uint8_t i = 0; i < N_IN; i++) { if (i) Serial.print(','); Serial.print(inStable[i] ? 1 : 0); }
    Serial.println();
  } else if (!strcmp(cmd, "GETCONFIG?")) {
    Serial.print(F("CONFIG OUT "));
    for (uint8_t i = 0; i < N_OUT; i++) { if (i) Serial.print(','); Serial.print(i + 1); Serial.print(':'); Serial.print(OUT_PIN[i]); }
    Serial.print(F(" IN "));
    for (uint8_t i = 0; i < N_IN; i++) { if (i) Serial.print(','); Serial.print(i + 1); Serial.print(':'); Serial.print(IN_PIN[i]); }
    Serial.println();
  } else if (!strcmp(cmd, "GETMODE?")) {
    Serial.print(F("MODE ")); Serial.println(mode);
  } else if (!strcmp(cmd, "SETMODE")) {
    if (na < 1 || a[0] < 0 || a[0] > 2) { replyERR(); return; }
    mode = (uint8_t)a[0]; EEPROM.update(EA_MODE, mode);
    stopAllSeq(); applyBootState(); bootAt = millis();     // 재시작과 같은 효과: 릴레이 시작 상태로, 3초 입력 무시
    for (uint8_t s = 0; s < MAX_SEQ; s++) sqPrev[s] = false;
    replyOK();
  } else if (!strcmp(cmd, "GETINV?")) {
    printList(F("INV "), inInv, N_IN);
  } else if (!strcmp(cmd, "SETINV")) {
    if (na < 2 || a[0] < 1 || a[0] > N_IN) { replyERR(); return; }
    inInv[a[0] - 1] = a[1] ? 1 : 0; EEPROM.update(EA_INV + a[0] - 1, inInv[a[0] - 1]);
    inLast[a[0] - 1] = inStable[a[0] - 1] = ((digitalRead(IN_PIN[a[0] - 1]) == LOW) != (inInv[a[0] - 1] != 0));
    replyOK();
  } else if (!strcmp(cmd, "GETBOOT?")) {
    printList(F("BOOT "), outBoot, N_OUT);
  } else if (!strcmp(cmd, "SETBOOT")) {
    if (na < 2 || a[0] < 1 || a[0] > N_OUT) { replyERR(); return; }
    outBoot[a[0] - 1] = a[1] ? 1 : 0; EEPROM.update(EA_BOOT + a[0] - 1, outBoot[a[0] - 1]);
    replyOK();
  } else if (!strcmp(cmd, "GETOUTINV?")) {
    printList(F("OUTINV "), outInv, N_OUT);
  } else if (!strcmp(cmd, "SETOUTINV")) {
    if (na < 2 || a[0] < 1 || a[0] > N_OUT) { replyERR(); return; }
    outInv[a[0] - 1] = a[1] ? 1 : 0; EEPROM.update(EA_OUTINV + a[0] - 1, outInv[a[0] - 1]);
    setRelay((uint8_t)a[0], relayOn[a[0] - 1]);            // 현재 ON/OFF 의미는 유지하고 극성만 바꿔 다시 출력
    replyOK();
  } else if (!strcmp(cmd, "GETSEQORDER?")) {
    Serial.print(F("SEQORDER ")); Serial.println(seqOrderMode ? 1 : 0);
  } else if (!strcmp(cmd, "SETSEQORDER")) {
    seqOrderMode = (na >= 1 && a[0] != 0); EEPROM.update(EA_ORDER, seqOrderMode ? 1 : 0);
    seqOrderIdx = 0; replyOK();
  } else if (!strcmp(cmd, "SEQCLR")) {
    staging = true; stopAllSeq(); nSeq = 0; memset(seqs, 0, sizeof(seqs)); replyOK();
  } else if (!strcmp(cmd, "SEQDEF")) {                    // SEQDEF i trigCh trigCond 1 condMode
    if (na < 3 || a[0] < 0 || a[0] >= MAX_SEQ || a[1] < 0 || a[1] > N_IN || a[2] < 0 || a[2] > 3) { replyERR(); return; }
    staging = true; stopAllSeq();
    Seq &S = seqs[a[0]]; memset(&S, 0, sizeof(S));
    S.trigCh = (uint8_t)a[1]; S.trigCond = (uint8_t)a[2]; S.condMode = (na >= 5 && a[4] == 1) ? 1 : 0;
    if (a[0] >= nSeq) nSeq = (uint8_t)(a[0] + 1);
    replyOK();
  } else if (!strcmp(cmd, "SEQCOND")) {                   // SEQCOND i ch state
    if (na < 3 || a[0] < 0 || a[0] >= nSeq || a[1] < 1 || a[1] > N_IN || seqs[a[0]].nConds >= MAX_CONDS) { replyERR(); return; }
    Seq &S = seqs[a[0]]; S.cond[S.nConds++] = (uint8_t)(a[1] | (a[2] ? 0x80 : 0)); replyOK();
  } else if (!strcmp(cmd, "SEQSTEP")) {                   // SEQSTEP i relay state delay [waitCh waitState]
    if (na < 4 || a[0] < 0 || a[0] >= nSeq || a[1] < 1 || a[1] > N_OUT || a[3] < 0 || a[3] > 65535 || seqs[a[0]].nSteps >= MAX_STEPS) { replyERR(); return; }
    uint8_t wch = 0, won = 0;
    if (na >= 6) { if (a[4] < 1 || a[4] > N_IN) { replyERR(); return; } wch = (uint8_t)a[4]; won = a[5] ? 1 : 0; }
    Seq &S = seqs[a[0]]; Step &st = S.st[S.nSteps++];
    st.b = (uint8_t)((a[1] - 1) | ((a[2] ? 1 : 0) << 2) | (won << 3) | (wch << 4));
    st.d = wch ? 0 : (uint16_t)a[3];
    replyOK();
  } else if (!strcmp(cmd, "SEQSAVE")) {
    saveSeqs(); staging = false; stopAllSeq(); bootAt = millis(); replyOK();
  } else if (!strcmp(cmd, "SEQABORT")) {
    loadSeqs(); staging = false; stopAllSeq(); replyOK();
  } else if (!strcmp(cmd, "SEQGET?")) {
    for (uint8_t i = 0; i < nSeq; i++) {
      const Seq &S = seqs[i];
      Serial.print(F("SEQD ")); Serial.print(i); Serial.print(' '); Serial.print(S.trigCh); Serial.print(' ');
      Serial.print(S.trigCond); Serial.print(F(" 1 ")); Serial.println(S.condMode);
      for (uint8_t c = 0; c < S.nConds; c++) {
        Serial.print(F("SEQC ")); Serial.print(i); Serial.print(' '); Serial.print(S.cond[c] & 0x7F); Serial.print(' ');
        Serial.println((S.cond[c] & 0x80) ? 1 : 0);
      }
      for (uint8_t k = 0; k < S.nSteps; k++) {
        const Step &st = S.st[k];
        Serial.print(F("SEQS ")); Serial.print(i); Serial.print(' '); Serial.print(stRelay(st)); Serial.print(' ');
        Serial.print(stOn(st) ? 1 : 0); Serial.print(' '); Serial.print(st.d);
        if (stWaitCh(st)) { Serial.print(' '); Serial.print(stWaitCh(st)); Serial.print(' '); Serial.print(stWaitOn(st) ? 1 : 0); }
        Serial.println();
      }
    }
    Serial.println(F("SEQEND"));
  } else {
    Serial.println(F("ERR UNKNOWN"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen) { lineBuf[lineLen] = 0; lineLen = 0; execCmd(lineBuf); }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                                          // 너무 긴 줄은 버림
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  loadAll();
  for (uint8_t i = 0; i < N_OUT; i++) {                      // 먼저 최종 레벨을 깔고 출력으로 전환(깜빡임 방지)
    relayOn[i] = (outBoot[i] != 0);
    bool activeLow = (outInv[i] == 0);
    digitalWrite(OUT_PIN[i], (relayOn[i] == activeLow) ? LOW : HIGH);
    pinMode(OUT_PIN[i], OUTPUT);
  }
  for (uint8_t i = 0; i < N_IN; i++) {
    pinMode(IN_PIN[i], INPUT_PULLUP);
    inLast[i] = inStable[i] = ((digitalRead(IN_PIN[i]) == LOW) != (inInv[i] != 0));
    inChangedAt[i] = millis();
  }
  stopAllSeq();
  for (uint8_t s = 0; s < MAX_SEQ; s++) sqPrev[s] = false;
  bootAt = millis();
}

void loop() {
  readSerial();
  updateInputs();
  runSeqs();
}
