// 릴레이 수신용 Uno: 감지 Uno의 신호를 받아 릴레이를 작동
// 배선: 감지Uno D8 -> 이 보드 D2 / GND끼리 연결 / D7 -> 릴레이 모듈 IN / 5V, GND -> 릴레이 VCC, GND
#define PIN_IN     2      // 감지 Uno 신호 입력
#define PIN_RELAY  7      // 릴레이 모듈 IN

const bool IN_ALARM_HIGH   = true;   // 감지 Uno 출력 극성: HIGH=경보면 true, LOW=경보면 false
const bool RELAY_ACTIVE_LOW = true;  // 릴레이 모듈이 LOW에서 동작하면 true (대부분)
const unsigned long DEBOUNCE_MS = 50;      // 신호 떨림 방지
const unsigned long MIN_ON_MS   = 2000;    // 릴레이 최소 유지시간
const bool LATCH = false;                  // true: 한 번 작동하면 유지 (리셋버튼 D4)
#define PIN_RST 4

bool relayOn = false, lastRaw = false, stable = false;
unsigned long tChange = 0, tOn = 0;

void setRelay(bool on){
  relayOn = on;
  if (on) tOn = millis();
  digitalWrite(PIN_RELAY, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setup(){
  Serial.begin(9600);
  pinMode(PIN_IN, INPUT);        // 외부 10k 풀다운 권장 (선 빠짐 대비)
  pinMode(PIN_RST, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);
}

void loop(){
  bool raw = (digitalRead(PIN_IN) == HIGH) == IN_ALARM_HIGH;   // true = 경보 신호
  unsigned long now = millis();

  if (raw != lastRaw){ lastRaw = raw; tChange = now; }
  if (now - tChange >= DEBOUNCE_MS) stable = raw;

  if (stable && !relayOn) setRelay(true);
  if (!stable && relayOn && !LATCH && now - tOn >= MIN_ON_MS) setRelay(false);
  if (LATCH && digitalRead(PIN_RST) == LOW && !stable) setRelay(false);

  static unsigned long tp = 0;
  if (now - tp >= 500){ tp = now; Serial.print("IN="); Serial.print(stable); Serial.print(",RELAY="); Serial.println(relayOn); }
}
