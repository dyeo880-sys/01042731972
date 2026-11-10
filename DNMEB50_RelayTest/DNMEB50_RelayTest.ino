// DNMEB50 + Mega2560 릴레이 단독 시험 (웹 펌웨어와 무관, 하드웨어 확인용)
// 릴레이 CH1~CH8을 하나씩 1초씩 켭니다. 먼저 HIGH=ON 방식, 다음 LOW=ON 방식, 마지막으로 MOS 출력 D1~D8.
// 쉬는 핀은 INPUT(떠 있는 상태)으로 두어 어느 방식이든 꺼진 상태가 되도록 했습니다.
// 시리얼 모니터(115200)에 진행 상황이 출력됩니다. 시험할 때는 부하를 연결하지 마세요.
const byte RELAY[8] = {41,39,37,35,33,31,29,27};
const byte MOS[8]   = {43,45,47,49,50,51,52,53};

void sweep(const byte* p, const char* name, bool activeHigh){
  Serial.print(name); Serial.println(activeHigh ? " : HIGH = ON 방식" : " : LOW = ON 방식");
  for(int i=0;i<8;i++){
    pinMode(p[i], OUTPUT);
    digitalWrite(p[i], activeHigh ? HIGH : LOW);
    Serial.print("  CH"); Serial.print(i+1); Serial.print(" (pin "); Serial.print(p[i]); Serial.println(") ON");
    delay(1000);
    pinMode(p[i], INPUT);
  }
}
void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== DNMEB50 시험 시작 ===");
}
void loop(){
  sweep(RELAY,"릴레이",true);  delay(1500);
  sweep(RELAY,"릴레이",false); delay(1500);
  sweep(MOS,"MOS",true);       delay(1500);
  Serial.println("--- 한 바퀴 끝, 반복 ---");
}
