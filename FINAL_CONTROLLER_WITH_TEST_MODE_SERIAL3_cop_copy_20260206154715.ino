/* =========================================================
   FINAL CONTROLLER + TEST MODE + PC CONTROL
   순차 ON/OFF 리셋 완벽 방지 + 통합 테스트 기능
   메모리 최적화 버전 (F() 매크로 사용)
   블렌더 예비 회전 + 느린 초기 하강 적용
   
   시리얼 포트:
   - Serial (USB): 디버깅 전용
   - Serial3 (D14/D15): PC 제어 전용
   ========================================================= */

#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// 메모리 절약을 위해 테스트 모드 비활성화 (필요시 주석 해제)
#define ENABLE_TEST_MODE  // ← 활성화!

// 시리얼 포트 정의
#define DEBUG_SERIAL Serial   // USB 디버깅용
#define PC_SERIAL Serial3     // PC 제어용 (D14/D15) - Serial3 사용!

/* ================= PIN MAP ================= */

// PHOTO SENSOR
#define PHOTO_PIN 3
#define PHOTO_STABLE_MS 50

// BUTTON
#define BTN_START 52
#define BTN_DEBOUNCE_MS 50

// RELAY (AM-RB04-B : ACTIVE HIGH)
#define RELAY_VALVE1 A4
#define RELAY_VALVE2 A5
#define RELAY_PUMP   A6
#define RELAY_LED    A7

// SSR
#define SSR_BLENDER 8
#define SSR_HEATER  9

// DC MOTOR
#define DC_IN1 A0
#define DC_IN2 A1
#define DC_SW_BOTTOM 18
#define DC_SW_TOP    19

// STEPPER
#define STP_EN   10
#define STP_DIR  11
#define STP_STEP 12
#define STP_SW_BOTTOM 20
#define STP_SW_TOP    21

/* ================= CONSTANT ================= */

#define RELAY_ON  HIGH
#define RELAY_OFF LOW
#define SSR_ON    HIGH
#define SSR_OFF   LOW

#define STEPS_PER_REV 3200
#define PULSE_INTERVAL_US 1000

// EEPROM
#define EEPROM_ADDR_RUNFLAG 0
#define EEPROM_ADDR_STEP    1
#define EEPROM_ADDR_MODE    2  // 테스트 모드 플래그
#define RUNFLAG_IDLE    0xAA
#define RUNFLAG_RUNNING 0x55
#define RUNFLAG_FIRST   0xFF
#define MODE_NORMAL     0xCC
#define MODE_TEST       0xDD

/* ================= STATE ================= */

enum RunState {
  POWER_ON,
  HOMING,
  READY,
  RUNNING,
  PAUSED,
  RETURNING_HOME,
  COMPLETION_FLASH,
  WAITING_FOR_REMOVAL,
  CLEANING_WAIT_CONFIRM,  
  CLEANING_DESCEND,
  CLEANING_PROCESS,
  CLEANING_RETURN_HOME,
  EMERGENCY_RETURN_HOME,
  ERROR_STATE
};

RunState runState = POWER_ON;
int stepIndex = 0;
unsigned long stepTimer = 0;

bool emergencyStopTriggered = false;
bool testMode = false;  // 테스트 모드 플래그

long stepperTargetSteps = 0;
long stepperCurrentStep = 0;
unsigned long stepperLastPulse = 0;

long cleaningStepperPosition = 0;
unsigned long cleaningDcDownTime = 0;
unsigned long slowStepperStartTime = 0;  // 느린 스테퍼 시작 시간

// 시리얼 통신
String serialBuffer = "";
unsigned long lastStatusSend = 0;
#define STATUS_INTERVAL 1000  // 1초마다 상태 전송

/* ================= 동시 홈잉 상태 ================= */

enum HomingPhase {
  HOME_INIT,
  HOME_STEPPER_FAST,
  HOME_STEPPER_BACKOFF1,
  HOME_STEPPER_FAST2,
  HOME_STEPPER_BACKOFF2,
  HOME_STEPPER_SLOW,
  HOME_DC_RUNNING,
  HOME_COMPLETE
};

HomingPhase stepperPhase = HOME_INIT;
HomingPhase dcPhase = HOME_INIT;
unsigned long stepperTimer = 0;
unsigned long dcTimer = 0;
int stepperBackoffCount = 0;

/* ================= BASIC IO ================= */

void relayOn(int p){ 
  digitalWrite(p, RELAY_ON); 
  delay(10);
}

void relayOff(int p){ 
  digitalWrite(p, RELAY_OFF); 
  delay(10);
}

void ssrOn(int p){ 
  digitalWrite(p, SSR_ON); 
  DEBUG_SERIAL.print(F(">>> SSR ON: 핀 "));
  DEBUG_SERIAL.println(p);
  delay(10);
}

void ssrOff(int p){ 
  digitalWrite(p, SSR_OFF); 
  DEBUG_SERIAL.print(F(">>> SSR OFF: 핀 "));
  DEBUG_SERIAL.println(p);
  delay(10);
}

/* ================= PHOTO SENSOR ================= */

bool photoDetectedNow() {
  return digitalRead(PHOTO_PIN) == LOW;
}

bool photoDetectedStable() {
  static unsigned long lowStart = 0;
  if (digitalRead(PHOTO_PIN) == LOW) {
    if (lowStart == 0) lowStart = millis();
    if (millis() - lowStart >= PHOTO_STABLE_MS) return true;
  } else {
    lowStart = 0;
  }
  return false;
}

bool photoObjectRemoved() {
  static unsigned long highStart = 0;
  if (digitalRead(PHOTO_PIN) == HIGH) {
    if (highStart == 0) highStart = millis();
    if (millis() - highStart >= PHOTO_STABLE_MS) return true;
  } else {
    highStart = 0;
  }
  return false;
}

bool photoInterlockEnabled() {
  return (runState == RUNNING || runState == PAUSED);
}

/* ================= BUTTON WITH DEBOUNCE ================= */

bool btnStartPressed() {
  static unsigned long lastPress = 0;
  static bool lastState = HIGH;
  
  bool currentState = digitalRead(BTN_START);
  
  if (currentState == LOW && lastState == HIGH) {
    if (millis() - lastPress > BTN_DEBOUNCE_MS) {
      lastPress = millis();
      lastState = LOW;
      return true;
    }
  }
  
  if (currentState == HIGH) {
    lastState = HIGH;
  }
  
  return false;
}

/* ================= DC MOTOR ================= */

void dcDown() { digitalWrite(DC_IN1, HIGH); digitalWrite(DC_IN2, LOW); }
void dcUp()   { digitalWrite(DC_IN1, LOW);  digitalWrite(DC_IN2, HIGH); }
void dcStop() { digitalWrite(DC_IN1, LOW);  digitalWrite(DC_IN2, LOW); }

/* ================= STEPPER ================= */

void stepperEnable(bool en){ digitalWrite(STP_EN, en ? HIGH : LOW); }
void stepperDirDown(){ digitalWrite(STP_DIR, LOW); }
void stepperDirUp(){ digitalWrite(STP_DIR, HIGH); }

void stepPulseFast(){
  digitalWrite(STP_STEP, HIGH); 
  delayMicroseconds(250);
  digitalWrite(STP_STEP, LOW);  
  delayMicroseconds(250);
}

void stepPulseSlow(){
  digitalWrite(STP_STEP, HIGH); 
  delayMicroseconds(1000);
  digitalWrite(STP_STEP, LOW);  
  delayMicroseconds(1000);
}

void stepperStartMove(long steps, bool directionDown) {
  stepperTargetSteps = steps;
  stepperCurrentStep = 0;
  stepperLastPulse = 0;
  stepperEnable(true);
  if (directionDown) {
    stepperDirDown();
  } else {
    stepperDirUp();
  }
}

bool stepperMoveDone() {
  if (stepperCurrentStep >= stepperTargetSteps) {
    stepperEnable(false);
    return true;
  }
  
  int pulsesThisCycle = 0;
  unsigned long now = micros();
  
  while (stepperCurrentStep < stepperTargetSteps && pulsesThisCycle < 50) {
    digitalWrite(STP_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(STP_STEP, LOW);
    delayMicroseconds(240);
    
    stepperCurrentStep++;
    pulsesThisCycle++;
  }
  
  return false;
}

// 65초 동안 천천히 이동하는 함수
bool stepperMoveDoneSlow120s() {
  if (stepperCurrentStep >= stepperTargetSteps) {
    stepperEnable(false);
    slowStepperStartTime = 0;  // 리셋
    return true;
  }
  
  // 타임아웃 체크 추가 (70초 = 65초 + 5초 여유)
  if (slowStepperStartTime > 0 && (millis() - slowStepperStartTime > 70000)) {
    DEBUG_SERIAL.println(F("!!! 느린 스테퍼 이동 타임아웃 !!!"));
    stepperEnable(false);
    slowStepperStartTime = 0;
    return true;  // 타임아웃으로 완료 처리
  }
  
  // 65초 동안 44,800 스텝 이동 (14회전)
  // = 689.23 스텝/초
  // = 약 7 스텝/10ms
  
  static unsigned long lastBatchTime = 0;
  unsigned long now = millis();
  
  // 첫 실행 시 slowStepperStartTime이 설정되어 있어야 함
  if (slowStepperStartTime == 0) {
    return false;  // 아직 준비 안 됨
  }
  
  // 첫 실행 시 lastBatchTime 초기화
  if (lastBatchTime == 0) {
    lastBatchTime = now;
  }
  
  // 10ms마다 7스텝씩 실행 (700스텝/초 = 64초에 44,800스텝)
  if (now - lastBatchTime >= 10) {
    for (int i = 0; i < 7 && stepperCurrentStep < stepperTargetSteps; i++) {
      digitalWrite(STP_STEP, HIGH);
      delayMicroseconds(5);
      digitalWrite(STP_STEP, LOW);
      delayMicroseconds(5);  // 최소한의 딜레이만
      stepperCurrentStep++;
    }
    lastBatchTime = now;
  }
  
  return false;
}


/* ================= SAFETY ================= */

void safeInitOutputs(){
  relayOff(RELAY_VALVE1);
  relayOff(RELAY_VALVE2);
  relayOff(RELAY_PUMP);
  relayOff(RELAY_LED);
  ssrOff(SSR_BLENDER);
  ssrOff(SSR_HEATER);
  dcStop();
  stepperEnable(false);
}

void pauseAll(){
  safeInitOutputs();
  DEBUG_SERIAL.println(F("!! 일시정지 : 포토 센서 감지 안됨 !!"));
}

/* ================= EEPROM 관리 ================= */

void saveRunFlag(byte flag) {
  EEPROM.update(EEPROM_ADDR_RUNFLAG, flag);
}

byte loadRunFlag() {
  return EEPROM.read(EEPROM_ADDR_RUNFLAG);
}

void saveStepIndex(int step) {
  EEPROM.update(EEPROM_ADDR_STEP, step);
}

int loadStepIndex() {
  return EEPROM.read(EEPROM_ADDR_STEP);
}

void saveModeFlag(byte mode) {
  EEPROM.update(EEPROM_ADDR_MODE, mode);
}

byte loadModeFlag() {
  return EEPROM.read(EEPROM_ADDR_MODE);
}

void clearEEPROM() {
  saveRunFlag(RUNFLAG_IDLE);
  saveStepIndex(0);
}

/* ================= 스테퍼 홈잉 프로세스 ================= */

void processStepperHoming() {
  switch (stepperPhase) {
    case HOME_INIT:
      DEBUG_SERIAL.println(F("[스테퍼] 홈잉 시작"));
      DEBUG_SERIAL.print(F("[스테퍼] 상단 리미트: "));
      DEBUG_SERIAL.println(digitalRead(STP_SW_TOP) == LOW ? "ON" : "OFF");
      DEBUG_SERIAL.print(F("[스테퍼] 하단 리미트: "));
      DEBUG_SERIAL.println(digitalRead(STP_SW_BOTTOM) == LOW ? "ON" : "OFF");
      
      stepperEnable(true);
      delay(50);
      
      DEBUG_SERIAL.print(F("[스테퍼] Enable 핀: "));
      DEBUG_SERIAL.println(digitalRead(STP_EN));
      
      // 하단 리미트 스위치 체크 추가
      if (digitalRead(STP_SW_BOTTOM) == LOW) {
        DEBUG_SERIAL.println(F("[스테퍼] ★ 하단 스위치 감지 -> 상단으로 이동 시작"));
        stepperDirUp();
        delay(100);
        DEBUG_SERIAL.print(F("[스테퍼] DIR 핀: "));
        DEBUG_SERIAL.println(digitalRead(STP_DIR));
        DEBUG_SERIAL.println(F("[스테퍼] 빠른 상승 시작..."));
        stepperTimer = millis();
        stepperPhase = HOME_STEPPER_FAST;
      }
      else if (digitalRead(STP_SW_TOP) == LOW) {
        DEBUG_SERIAL.println(F("[스테퍼] 상단 스위치 이미 눌림 -> 백오프"));
        stepperDirDown();
        stepperBackoffCount = 0;
        stepperPhase = HOME_STEPPER_BACKOFF1;
      } else {
        DEBUG_SERIAL.println(F("[스테퍼] 1단계: 빠른 접근"));
        stepperDirUp();
        stepperTimer = millis();
        stepperPhase = HOME_STEPPER_FAST;
      }
      break;
    case HOME_STEPPER_FAST:
      if (digitalRead(STP_SW_TOP) == LOW) {
        DEBUG_SERIAL.println(F("[스테퍼] 스위치 감지 (1차)"));
        stepperDirDown();
        stepperBackoffCount = 0;
        stepperPhase = HOME_STEPPER_BACKOFF1;
      } else if (millis() - stepperTimer > 60000) {  // 40초 → 60초로 증가
        DEBUG_SERIAL.println(F("[스테퍼] 1차 접근 타임아웃!"));
        stepperEnable(false);
        stepperPhase = HOME_COMPLETE;
      } else {
        // 매 1초마다 진행 상황 출력
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 1000) {
          DEBUG_SERIAL.print(F("[스테퍼] 상승 중... "));
          DEBUG_SERIAL.print((millis() - stepperTimer) / 1000);
          DEBUG_SERIAL.println(F("초"));
          lastDebug = millis();
        }
        stepPulseFast();
      }
      break;
    case HOME_STEPPER_BACKOFF1:
      if (stepperBackoffCount < 600) {
        stepPulseFast();
        stepperBackoffCount++;
      } else {
        DEBUG_SERIAL.println(F("[스테퍼] 2단계: 백오프 완료"));
        delay(200);
        DEBUG_SERIAL.println(F("[스테퍼] 3단계: 빠른 재접근"));
        stepperDirUp();
        stepperTimer = millis();
        stepperPhase = HOME_STEPPER_FAST2;
      }
      break;
    case HOME_STEPPER_FAST2:
      if (digitalRead(STP_SW_TOP) == LOW) {
        DEBUG_SERIAL.println(F("[스테퍼] 스위치 재감지"));
        stepperDirDown();
        stepperBackoffCount = 0;
        stepperPhase = HOME_STEPPER_BACKOFF2;
      } else if (millis() - stepperTimer > 15000) {
        DEBUG_SERIAL.println(F("[스테퍼] 2차 접근 타임아웃!"));
        stepperEnable(false);
        stepperPhase = HOME_COMPLETE;
      } else {
        stepPulseFast();
      }
      break;
    case HOME_STEPPER_BACKOFF2:
      if (stepperBackoffCount < 150) {
        stepPulseFast();
        stepperBackoffCount++;
      } else {
        DEBUG_SERIAL.println(F("[스테퍼] 4단계: 작은 백오프 완료"));
        delay(200);
        DEBUG_SERIAL.println(F("[스테퍼] 5단계: 느린 접근"));
        stepperDirUp();
        stepperTimer = millis();
        stepperPhase = HOME_STEPPER_SLOW;
      }
      break;
    case HOME_STEPPER_SLOW:
      if (digitalRead(STP_SW_TOP) == LOW) {
        stepperEnable(false);
        delay(100);
        DEBUG_SERIAL.println(F("[스테퍼] 홈잉 완료!"));
        stepperPhase = HOME_COMPLETE;
      } else if (millis() - stepperTimer > 5000) {
        DEBUG_SERIAL.println(F("[스테퍼] 느린 접근 타임아웃!"));
        stepperEnable(false);
        stepperPhase = HOME_COMPLETE;
      } else {
        stepPulseSlow();
      }
      break;
    case HOME_COMPLETE:
      break;
    default:
      break;
  }
}

/* ================= DC 모터 홈잉 프로세스 ================= */

void processDCHoming() {
  switch (dcPhase) {
    case HOME_INIT:
      DEBUG_SERIAL.println(F("[DC] 홈잉 시작"));
      dcUp();
      dcTimer = millis();
      dcPhase = HOME_DC_RUNNING;
      break;
    case HOME_DC_RUNNING:
      if (digitalRead(DC_SW_TOP) == LOW) {
        dcStop();
        delay(100);
        DEBUG_SERIAL.println(F("[DC] 홈잉 완료!"));
        dcPhase = HOME_COMPLETE;
      } else if (millis() - dcTimer > 25000) {
        dcStop();
        DEBUG_SERIAL.println(F("[DC] 홈잉 타임아웃!"));
        dcPhase = HOME_COMPLETE;
      }
      break;
    case HOME_COMPLETE:
      break;
    default:
      break;
  }
}

/* ================= 동시 홈잉 메인 함수 ================= */

bool homeStepperOnly() {
  DEBUG_SERIAL.println(F("===== 스테퍼 모터 단독 홈잉 시작 ====="));
  stepperPhase = HOME_INIT;
  stepperTimer = 0;
  stepperBackoffCount = 0;
  unsigned long startTime = millis();
  unsigned long timeout = 60000;
  
  while (stepperPhase != HOME_COMPLETE) {
    if (millis() - startTime > timeout) {
      DEBUG_SERIAL.println(F("스테퍼 홈잉 타임아웃!"));
      stepperEnable(false);
      return false;
    }
    processStepperHoming();
  }
  
  if (stepperPhase == HOME_COMPLETE) {
    DEBUG_SERIAL.println(F("===== 스테퍼 모터 홈잉 완료 ====="));
    return true;
  } else {
    DEBUG_SERIAL.println(F("===== 스테퍼 홈잉 실패 ====="));
    return false;
  }
}

bool homeDCOnly() {
  DEBUG_SERIAL.println(F("===== DC 모터 단독 홈잉 시작 ====="));
  dcPhase = HOME_INIT;
  dcTimer = 0;
  unsigned long startTime = millis();
  unsigned long timeout = 30000;
  
  while (dcPhase != HOME_COMPLETE) {
    if (millis() - startTime > timeout) {
      DEBUG_SERIAL.println(F("DC 홈잉 타임아웃!"));
      dcStop();
      return false;
    }
    processDCHoming();
  }
  
  if (dcPhase == HOME_COMPLETE) {
    DEBUG_SERIAL.println(F("===== DC 모터 홈잉 완료 ====="));
    return true;
  } else {
    DEBUG_SERIAL.println(F("===== DC 홈잉 실패 ====="));
    return false;
  }
}

bool homeAllSimultaneous() {
  DEBUG_SERIAL.println(F("===== 동시 홈잉 시작 ====="));
  stepperPhase = HOME_INIT;
  dcPhase = HOME_INIT;
  stepperTimer = 0;
  dcTimer = 0;
  stepperBackoffCount = 0;
  unsigned long startTime = millis();
  unsigned long timeout = 60000;
  while (stepperPhase != HOME_COMPLETE || dcPhase != HOME_COMPLETE) {
    if (millis() - startTime > timeout) {
      DEBUG_SERIAL.println(F("전체 홈잉 타임아웃!"));
      stepperEnable(false);
      dcStop();
      return false;
    }
    processStepperHoming();
    processDCHoming();
  }
  if (stepperPhase == HOME_COMPLETE && dcPhase == HOME_COMPLETE) {
    DEBUG_SERIAL.println(F("===== 동시 홈잉 완료 ====="));
    return true;
  } else {
    DEBUG_SERIAL.println(F("===== 홈잉 실패 ====="));
    return false;
  }
}

bool homeAll() {
  return homeAllSimultaneous();
}

/* ================= LED STATUS ================= */

void updateStatusLED(){
  static unsigned long lastMs = 0;
  static bool ledState = false;
  unsigned long interval = 1000;
  switch (runState) {
    case POWER_ON: interval = 200; break;
    case HOMING: interval = 500; break;
    case PAUSED: interval = 700; break;
    case ERROR_STATE: interval = 100; break;
    case RETURNING_HOME: interval = 300; break;
    case WAITING_FOR_REMOVAL: interval = 400; break;
    case CLEANING_WAIT_CONFIRM: interval = 200; break;
    case CLEANING_DESCEND: interval = 250; break;
    case CLEANING_PROCESS: interval = 150; break;
    case CLEANING_RETURN_HOME: interval = 350; break;
    case EMERGENCY_RETURN_HOME: interval = 100; break;
    case READY:
      relayOn(RELAY_LED);
      return;
    case RUNNING:
      relayOff(RELAY_LED);
      return;
    case COMPLETION_FLASH:
      return;
  }
  if (millis() - lastMs >= interval) {
    lastMs = millis();
    ledState = !ledState;
    digitalWrite(RELAY_LED, ledState ? RELAY_ON : RELAY_OFF);
  }
}

/* ================= TEST MODE FUNCTIONS ================= */

#ifdef ENABLE_TEST_MODE

void printTestMenu() {
  DEBUG_SERIAL.println(F("\n========================================"));
  DEBUG_SERIAL.println(F("       테스트 모드 메뉴"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("[ 릴레이 테스트 (개별) ]"));
  DEBUG_SERIAL.println(F("1. 밸브1 ON/OFF (5초)"));
  DEBUG_SERIAL.println(F("2. 밸브2 ON/OFF (5초)"));
  DEBUG_SERIAL.println(F("3. 펌프 ON/OFF (5초)"));
  DEBUG_SERIAL.println(F("4. LED ON/OFF (5초)"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("[ SSR 테스트 (개별) ]"));
  DEBUG_SERIAL.println(F("5. 히터 ON/OFF (5초)"));
  DEBUG_SERIAL.println(F("6. 블렌더 ON/OFF (5초)"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("[ DC 모터 테스트 ]"));
  DEBUG_SERIAL.println(F("7. DC 상승 (3초)"));
  DEBUG_SERIAL.println(F("8. DC 하강 (3초)"));
  DEBUG_SERIAL.println(F("9. DC 리미트 스위치 확인"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("[ 스테퍼 모터 테스트 ]"));
  DEBUG_SERIAL.println(F("s. 스테퍼 홈잉"));
  DEBUG_SERIAL.println(F("u. 스테퍼 상승 (1회전)"));
  DEBUG_SERIAL.println(F("d. 스테퍼 하강 (1회전)"));
  DEBUG_SERIAL.println(F("w. 스테퍼 리미트 스위치 확인"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("[ 조합 테스트 ]"));
  DEBUG_SERIAL.println(F("a. 밸브1 + 펌프 (5초)"));
  DEBUG_SERIAL.println(F("b. 밸브2 + 펌프 (5초)"));
  DEBUG_SERIAL.println(F("c. 밸브1 + 밸브2 + 펌프 (5초)"));
  DEBUG_SERIAL.println(F("e. 밸브2 + 펌프 + 히터 + 블렌더 (5초)"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("[ 순차 테스트 ]"));
  DEBUG_SERIAL.println(F("f. 모든 릴레이 순차 테스트 (각 3초)"));
  DEBUG_SERIAL.println(F("g. 전체 시스템 테스트"));
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("0. 모든 출력 OFF"));
  DEBUG_SERIAL.println(F("h. 메뉴 다시 보기"));
  DEBUG_SERIAL.println(F("x. 정상 모드로 전환"));
  DEBUG_SERIAL.println(F("========================================"));
}

void testValve1() {
  DEBUG_SERIAL.println(F("\n[밸브1 테스트]"));
  DEBUG_SERIAL.println(F("  밸브1 ON"));
  relayOn(RELAY_VALVE1);
  delay(5000);
  DEBUG_SERIAL.println(F("  밸브1 OFF"));
  relayOff(RELAY_VALVE1);
  DEBUG_SERIAL.println(F("  완료"));
}

void testValve2() {
  DEBUG_SERIAL.println(F("\n[밸브2 테스트]"));
  DEBUG_SERIAL.println(F("  밸브2 ON"));
  relayOn(RELAY_VALVE2);
  delay(5000);
  DEBUG_SERIAL.println(F("  밸브2 OFF"));
  relayOff(RELAY_VALVE2);
  DEBUG_SERIAL.println(F("  완료"));
}

void testPump() {
  DEBUG_SERIAL.println(F("\n[펌프 테스트]"));
  DEBUG_SERIAL.println(F("  펌프 ON"));
  relayOn(RELAY_PUMP);
  delay(5000);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  DEBUG_SERIAL.println(F("  완료"));
}

void testLED() {
  DEBUG_SERIAL.println(F("\n[LED 테스트]"));
  DEBUG_SERIAL.println(F("  LED ON"));
  relayOn(RELAY_LED);
  delay(5000);
  DEBUG_SERIAL.println(F("  LED OFF"));
  relayOff(RELAY_LED);
  DEBUG_SERIAL.println(F("  완료"));
}

void testHeater() {
  DEBUG_SERIAL.println(F("\n[히터 테스트]"));
  DEBUG_SERIAL.println(F("  히터 ON (5초)"));
  ssrOn(SSR_HEATER);
  delay(5000);
  DEBUG_SERIAL.println(F("  히터 OFF"));
  ssrOff(SSR_HEATER);
  DEBUG_SERIAL.println(F("  완료"));
}

void testBlender() {
  DEBUG_SERIAL.println(F("\n[블렌더 테스트]"));
  DEBUG_SERIAL.println(F("  블렌더 ON (5초)"));
  ssrOn(SSR_BLENDER);
  delay(5000);
  DEBUG_SERIAL.println(F("  블렌더 OFF"));
  ssrOff(SSR_BLENDER);
  DEBUG_SERIAL.println(F("  완료"));
}

void testDCUp() {
  DEBUG_SERIAL.println(F("\n[DC 모터 상승 테스트]"));
  DEBUG_SERIAL.println(F("  DC 모터 상승 (3초)"));
  dcUp();
  delay(3000);
  DEBUG_SERIAL.println(F("  DC 모터 정지"));
  dcStop();
  DEBUG_SERIAL.println(F("  완료"));
}

void testDCDown() {
  DEBUG_SERIAL.println(F("\n[DC 모터 하강 테스트]"));
  DEBUG_SERIAL.println(F("  DC 모터 하강 (3초)"));
  dcDown();
  delay(3000);
  DEBUG_SERIAL.println(F("  DC 모터 정지"));
  dcStop();
  DEBUG_SERIAL.println(F("  완료"));
}

void testDCSwitch() {
  DEBUG_SERIAL.println(F("\n[DC 리미트 스위치 확인]"));
  DEBUG_SERIAL.print(F("  상단 스위치: "));
  DEBUG_SERIAL.println(digitalRead(DC_SW_TOP) == LOW ? "눌림 (LOW) ●" : "안눌림 (HIGH) ○");
  DEBUG_SERIAL.print(F("  하단 스위치: "));
  DEBUG_SERIAL.println(digitalRead(DC_SW_BOTTOM) == LOW ? "눌림 (LOW) ●" : "안눌림 (HIGH) ○");
}

void testStepperHoming() {
  DEBUG_SERIAL.println(F("\n[스테퍼 홈잉 테스트]"));
  if (homeAll()) {
    DEBUG_SERIAL.println(F("  홈잉 완료"));
  } else {
    DEBUG_SERIAL.println(F("  홈잉 실패"));
  }
}

void testStepperUp() {
  DEBUG_SERIAL.println(F("\n[스테퍼 상승 테스트 - 1회전]"));
  stepperStartMove(STEPS_PER_REV, false);
  while (!stepperMoveDone()) {
    // 완료 대기
  }
  DEBUG_SERIAL.println(F("  완료"));
}

void testStepperDown() {
  DEBUG_SERIAL.println(F("\n[스테퍼 하강 테스트 - 1회전]"));
  stepperStartMove(STEPS_PER_REV, true);
  while (!stepperMoveDone()) {
    // 완료 대기
  }
  DEBUG_SERIAL.println(F("  완료"));
}

void testStepperSwitch() {
  DEBUG_SERIAL.println(F("\n[스테퍼 리미트 스위치 확인]"));
  DEBUG_SERIAL.print(F("  상단 스위치: "));
  DEBUG_SERIAL.println(digitalRead(STP_SW_TOP) == LOW ? "눌림 (LOW) ●" : "안눌림 (HIGH) ○");
  DEBUG_SERIAL.print(F("  하단 스위치: "));
  DEBUG_SERIAL.println(digitalRead(STP_SW_BOTTOM) == LOW ? "눌림 (LOW) ●" : "안눌림 (HIGH) ○");
}

void testValve1Pump() {
  DEBUG_SERIAL.println(F("\n[밸브1 + 펌프 조합 테스트]"));
  DEBUG_SERIAL.println(F("  밸브1 ON"));
  relayOn(RELAY_VALVE1);
  delay(1000);
  DEBUG_SERIAL.println(F("  펌프 ON (1초 후)"));
  relayOn(RELAY_PUMP);
  delay(4000);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브1 OFF"));
  relayOff(RELAY_VALVE1);
  DEBUG_SERIAL.println(F("  완료"));
}

void testValve2Pump() {
  DEBUG_SERIAL.println(F("\n[밸브2 + 펌프 조합 테스트]"));
  DEBUG_SERIAL.println(F("  밸브2 ON"));
  relayOn(RELAY_VALVE2);
  delay(1000);
  DEBUG_SERIAL.println(F("  펌프 ON (1초 후)"));
  relayOn(RELAY_PUMP);
  delay(4000);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브2 OFF"));
  relayOff(RELAY_VALVE2);
  DEBUG_SERIAL.println(F("  완료"));
}

void testBothValvesPump() {
  DEBUG_SERIAL.println(F("\n[밸브1 + 밸브2 + 펌프 조합 테스트]"));
  DEBUG_SERIAL.println(F("  밸브1 ON"));
  relayOn(RELAY_VALVE1);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브2 ON (0.5초 후)"));
  relayOn(RELAY_VALVE2);
  delay(1000);
  DEBUG_SERIAL.println(F("  펌프 ON (1초 후)"));
  relayOn(RELAY_PUMP);
  delay(5000);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브1 OFF"));
  relayOff(RELAY_VALVE1);
  delay(200);
  DEBUG_SERIAL.println(F("  밸브2 OFF"));
  relayOff(RELAY_VALVE2);
  DEBUG_SERIAL.println(F("  완료"));
}

void testAllCombination() {
  DEBUG_SERIAL.println(F("\n[전체 조합 테스트]"));
  DEBUG_SERIAL.println(F("  밸브2 ON"));
  relayOn(RELAY_VALVE2);
  delay(1000);
  DEBUG_SERIAL.println(F("  펌프 ON (1초 후)"));
  relayOn(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  히터 ON (0.5초 후)"));
  ssrOn(SSR_HEATER);
  delay(1000);
  DEBUG_SERIAL.println(F("  블렌더 ON (1초 후)"));
  ssrOn(SSR_BLENDER);
  delay(3000);
  DEBUG_SERIAL.println(F("  히터 OFF"));
  ssrOff(SSR_HEATER);
  delay(200);
  DEBUG_SERIAL.println(F("  블렌더 OFF"));
  ssrOff(SSR_BLENDER);
  delay(200);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브2 OFF"));
  relayOff(RELAY_VALVE2);
  DEBUG_SERIAL.println(F("  완료"));
}

void testSequential() {
  DEBUG_SERIAL.println(F("\n[순차 테스트 시작]"));
  
  DEBUG_SERIAL.println(F("\n1. 밸브1 (3초)"));
  relayOn(RELAY_VALVE1);
  delay(3000);
  relayOff(RELAY_VALVE1);
  delay(500);
  
  DEBUG_SERIAL.println(F("2. 밸브2 (3초)"));
  relayOn(RELAY_VALVE2);
  delay(3000);
  relayOff(RELAY_VALVE2);
  delay(500);
  
  DEBUG_SERIAL.println(F("3. 펌프 (3초)"));
  relayOn(RELAY_PUMP);
  delay(3000);
  relayOff(RELAY_PUMP);
  delay(500);
  
  DEBUG_SERIAL.println(F("4. LED (3초)"));
  relayOn(RELAY_LED);
  delay(3000);
  relayOff(RELAY_LED);
  delay(500);
  
  DEBUG_SERIAL.println(F("5. 히터 (3초)"));
  ssrOn(SSR_HEATER);
  delay(3000);
  ssrOff(SSR_HEATER);
  delay(500);
  
  DEBUG_SERIAL.println(F("6. 블렌더 (3초)"));
  ssrOn(SSR_BLENDER);
  delay(3000);
  ssrOff(SSR_BLENDER);
  delay(500);
  
  DEBUG_SERIAL.println(F("\n순차 테스트 완료!"));
}

void testFullSystem() {
  DEBUG_SERIAL.println(F("\n[전체 시스템 테스트]"));
  
  DEBUG_SERIAL.println(F("\n1단계: 밸브1 + 펌프 (5초)"));
  relayOn(RELAY_VALVE1);
  delay(1000);
  relayOn(RELAY_PUMP);
  delay(5000);
  relayOff(RELAY_PUMP);
  delay(500);
  relayOff(RELAY_VALVE1);
  delay(1000);
  
  DEBUG_SERIAL.println(F("\n2단계: 블렌더 (5초)"));
  ssrOn(SSR_BLENDER);
  delay(5000);
  ssrOff(SSR_BLENDER);
  delay(1000);
  
  DEBUG_SERIAL.println(F("\n3단계: 세척 시뮬레이션"));
  DEBUG_SERIAL.println(F("  밸브2 ON"));
  relayOn(RELAY_VALVE2);
  delay(1000);
  DEBUG_SERIAL.println(F("  펌프 ON"));
  relayOn(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  히터 ON"));
  ssrOn(SSR_HEATER);
  delay(1000);
  DEBUG_SERIAL.println(F("  블렌더 ON"));
  ssrOn(SSR_BLENDER);
  delay(3000);
  DEBUG_SERIAL.println(F("  히터 OFF"));
  ssrOff(SSR_HEATER);
  delay(1000);
  DEBUG_SERIAL.println(F("  블렌더 OFF"));
  ssrOff(SSR_BLENDER);
  delay(200);
  DEBUG_SERIAL.println(F("  펌프 OFF"));
  relayOff(RELAY_PUMP);
  delay(500);
  DEBUG_SERIAL.println(F("  밸브2 OFF"));
  relayOff(RELAY_VALVE2);
  
  DEBUG_SERIAL.println(F("\n전체 시스템 테스트 완료!"));
}

void handleTestMode() {
  if (DEBUG_SERIAL.available() > 0) {
    char input = DEBUG_SERIAL.read();
    
    while (DEBUG_SERIAL.available() > 0) {
      DEBUG_SERIAL.read();
    }
    
    if (input == '\n' || input == '\r') {
      return;
    }
    
    DEBUG_SERIAL.print(F("\n>> 선택: "));
    DEBUG_SERIAL.println(input);
    
    switch (input) {
      case '1': testValve1(); break;
      case '2': testValve2(); break;
      case '3': testPump(); break;
      case '4': testLED(); break;
      case '5': testHeater(); break;
      case '6': testBlender(); break;
      case '7': testDCUp(); break;
      case '8': testDCDown(); break;
      case '9': testDCSwitch(); break;
      case 's': case 'S': testStepperHoming(); break;
      case 'u': case 'U': testStepperUp(); break;
      case 'd': case 'D': testStepperDown(); break;
      case 'w': case 'W': testStepperSwitch(); break;
      case 'a': case 'A': testValve1Pump(); break;
      case 'b': case 'B': testValve2Pump(); break;
      case 'c': case 'C': testBothValvesPump(); break;
      case 'e': case 'E': testAllCombination(); break;
      case 'f': case 'F': testSequential(); break;
      case 'g': case 'G': testFullSystem(); break;
      case '0':
        DEBUG_SERIAL.println(F("\n[전체 OFF]"));
        safeInitOutputs();
        DEBUG_SERIAL.println(F("  → 모든 출력 OFF"));
        break;
      case 'h': case 'H':
        printTestMenu();
        break;
      case 'x': case 'X':
        DEBUG_SERIAL.println(F("\n========================================"));
        DEBUG_SERIAL.println(F("  정상 모드로 전환합니다"));
        DEBUG_SERIAL.println(F("  3초 후 재부팅..."));
        DEBUG_SERIAL.println(F("========================================"));
        saveModeFlag(MODE_NORMAL);
        delay(3000);
        asm volatile ("jmp 0");  // 소프트웨어 리셋
        break;
      default:
        DEBUG_SERIAL.println(F("\n✗ 잘못된 입력입니다."));
        DEBUG_SERIAL.println(F("'h'를 입력하여 메뉴를 다시 확인하세요."));
        break;
    }
    
    if (input != '\n' && input != '\r' && input != 'h' && input != 'H') {
      DEBUG_SERIAL.println(F("\n명령 완료. 다음 명령을 입력하세요."));
    }
  }
}

#endif  // ENABLE_TEST_MODE

/* ================= 시리얼 통신 함수 ================= */

String getStateString() {
  switch(runState) {
    case POWER_ON: return "POWER_ON";
    case HOMING: return "HOMING";
    case READY: return "READY";
    case RUNNING: return "RUNNING";
    case PAUSED: return "PAUSED";
    case RETURNING_HOME: return "RETURNING_HOME";
    case COMPLETION_FLASH: return "COMPLETION_FLASH";
    case WAITING_FOR_REMOVAL: return "WAITING_FOR_REMOVAL";
    case CLEANING_WAIT_CONFIRM: return "CLEANING_WAIT_CONFIRM";
    case CLEANING_DESCEND: return "CLEANING_DESCEND";
    case CLEANING_PROCESS: return "CLEANING_PROCESS";
    case CLEANING_RETURN_HOME: return "CLEANING_RETURN_HOME";
    case EMERGENCY_RETURN_HOME: return "EMERGENCY_RETURN_HOME";
    case ERROR_STATE: return "ERROR_STATE";
    default: return "UNKNOWN";
  }
}

void sendStatus() {
  PC_SERIAL.print(F("{\"type\":\"status\","));
  PC_SERIAL.print(F("\"state\":\""));
  PC_SERIAL.print(getStateString());
  PC_SERIAL.print(F("\",\"step\":"));
  PC_SERIAL.print(stepIndex);
  PC_SERIAL.print(F(",\"photo\":"));
  PC_SERIAL.print(photoDetectedNow() ? "true" : "false");
  
  // 에러 플래그 추가
  PC_SERIAL.print(F(",\"emergency\":"));
  PC_SERIAL.print(emergencyStopTriggered ? "true" : "false");
  
  PC_SERIAL.print(F(",\"dc_top\":"));
  PC_SERIAL.print(digitalRead(DC_SW_TOP) == LOW ? "true" : "false");
  PC_SERIAL.print(F(",\"dc_bottom\":"));
  PC_SERIAL.print(digitalRead(DC_SW_BOTTOM) == LOW ? "true" : "false");
  PC_SERIAL.print(F(",\"stp_top\":"));
  PC_SERIAL.print(digitalRead(STP_SW_TOP) == LOW ? "true" : "false");
  PC_SERIAL.print(F(",\"stp_bottom\":"));
  PC_SERIAL.print(digitalRead(STP_SW_BOTTOM) == LOW ? "true" : "false");
  PC_SERIAL.print(F(",\"valve1\":"));
  PC_SERIAL.print(digitalRead(RELAY_VALVE1) == RELAY_ON ? "true" : "false");
  PC_SERIAL.print(F(",\"valve2\":"));
  PC_SERIAL.print(digitalRead(RELAY_VALVE2) == RELAY_ON ? "true" : "false");
  PC_SERIAL.print(F(",\"pump\":"));
  PC_SERIAL.print(digitalRead(RELAY_PUMP) == RELAY_ON ? "true" : "false");
  PC_SERIAL.print(F(",\"blender\":"));
  PC_SERIAL.print(digitalRead(SSR_BLENDER) == SSR_ON ? "true" : "false");
  PC_SERIAL.print(F(",\"heater\":"));
  PC_SERIAL.print(digitalRead(SSR_HEATER) == SSR_ON ? "true" : "false");
  
  // 스테퍼 진행률 추가 (RUNNING 상태일 때만)
  if (runState == RUNNING && stepperTargetSteps > 0) {
    PC_SERIAL.print(F(",\"stepper_progress\":"));
    PC_SERIAL.print((stepperCurrentStep * 100) / stepperTargetSteps);
  }
  
  PC_SERIAL.println(F("}"));
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  
  if (cmd == "STATUS") {
    sendStatus();
  }
  else if (cmd == "START") {
    if (runState == READY && photoDetectedNow()) {
      PC_SERIAL.println(F("{\"response\":\"OK\",\"msg\":\"Starting\"}"));
      DEBUG_SERIAL.println(F("PC 명령: START 수신"));
      stepIndex = 0;
      runState = RUNNING;
      saveRunFlag(RUNFLAG_RUNNING);
      saveStepIndex(0);
    } else {
      PC_SERIAL.println(F("{\"response\":\"ERROR\",\"msg\":\"Not ready or no object\"}"));
      DEBUG_SERIAL.println(F("PC 명령: START 거부 (준비 안됨)"));
    }
  }
  else if (cmd == "STOP") {
    PC_SERIAL.println(F("{\"response\":\"OK\",\"msg\":\"Emergency stop\"}"));
    DEBUG_SERIAL.println(F("PC 명령: STOP 수신"));
    safeInitOutputs();
    emergencyStopTriggered = true;
    runState = EMERGENCY_RETURN_HOME;
  }
  else if (cmd == "HOME") {
    PC_SERIAL.println(F("{\"response\":\"OK\",\"msg\":\"Homing\"}"));
    DEBUG_SERIAL.println(F("PC 명령: HOME 수신"));
    runState = HOMING;
    if (homeAll()) {
      runState = READY;
    } else {
      runState = ERROR_STATE;
    }
  }
  else if (cmd.startsWith("VALVE1:")) {
    bool state = cmd.substring(7) == "ON";
    if (state) relayOn(RELAY_VALVE1);
    else relayOff(RELAY_VALVE1);
    PC_SERIAL.println(F("{\"response\":\"OK\"}"));
    DEBUG_SERIAL.print(F("PC 명령: VALVE1 "));
    DEBUG_SERIAL.println(state ? "ON" : "OFF");
  }
  else if (cmd.startsWith("VALVE2:")) {
    bool state = cmd.substring(7) == "ON";
    if (state) relayOn(RELAY_VALVE2);
    else relayOff(RELAY_VALVE2);
    PC_SERIAL.println(F("{\"response\":\"OK\"}"));
    DEBUG_SERIAL.print(F("PC 명령: VALVE2 "));
    DEBUG_SERIAL.println(state ? "ON" : "OFF");
  }
  else if (cmd.startsWith("PUMP:")) {
    bool state = cmd.substring(5) == "ON";
    if (state) relayOn(RELAY_PUMP);
    else relayOff(RELAY_PUMP);
    PC_SERIAL.println(F("{\"response\":\"OK\"}"));
    DEBUG_SERIAL.print(F("PC 명령: PUMP "));
    DEBUG_SERIAL.println(state ? "ON" : "OFF");
  }
  else if (cmd.startsWith("BLENDER:")) {
    bool state = cmd.substring(8) == "ON";
    if (state) ssrOn(SSR_BLENDER);
    else ssrOff(SSR_BLENDER);
    PC_SERIAL.println(F("{\"response\":\"OK\"}"));
    DEBUG_SERIAL.print(F("PC 명령: BLENDER "));
    DEBUG_SERIAL.println(state ? "ON" : "OFF");
  }
  else if (cmd.startsWith("HEATER:")) {
    bool state = cmd.substring(7) == "ON";
    if (state) ssrOn(SSR_HEATER);
    else ssrOff(SSR_HEATER);
    PC_SERIAL.println(F("{\"response\":\"OK\"}"));
    DEBUG_SERIAL.print(F("PC 명령: HEATER "));
    DEBUG_SERIAL.println(state ? "ON" : "OFF");
  }
  else {
    PC_SERIAL.println(F("{\"response\":\"ERROR\",\"msg\":\"Unknown command\"}"));
    DEBUG_SERIAL.print(F("PC 명령: 알 수 없는 명령 - "));
    DEBUG_SERIAL.println(cmd);
  }
}

void processSerialInput() {
  while (PC_SERIAL.available() > 0) {
    char c = PC_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      // 버퍼 크기 제한 추가
      if (serialBuffer.length() < 64) {
        serialBuffer += c;
      } else {
        // 버퍼 오버플로우 방지
        serialBuffer = "";
        PC_SERIAL.println(F("{\"response\":\"ERROR\",\"msg\":\"Command too long\"}"));
      }
    }
  }
  
  // 주기적으로 상태 전송
  if (millis() - lastStatusSend >= STATUS_INTERVAL) {
    sendStatus();
    lastStatusSend = millis();
  }
}

/* ================= SETUP ================= */

void setup(){
  cli();
  wdt_reset();
  wdt_disable();
  
  #if defined(MCUSR)
    MCUSR = 0;
  #elif defined(MCUCSR)
    MCUCSR = 0;
  #endif
  
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
  sei();
  
  // 시리얼 포트 초기화
  DEBUG_SERIAL.begin(115200);  // USB 디버깅용
  PC_SERIAL.begin(115200);     // PC 제어용 (D14/D15)
  delay(200);
  
  pinMode(PHOTO_PIN, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(RELAY_VALVE1, OUTPUT);
  pinMode(RELAY_VALVE2, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_LED, OUTPUT);
  pinMode(SSR_BLENDER, OUTPUT);
  pinMode(SSR_HEATER, OUTPUT);
  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  pinMode(DC_SW_TOP, INPUT_PULLUP);
  pinMode(DC_SW_BOTTOM, INPUT_PULLUP);
  pinMode(STP_EN, OUTPUT);
  pinMode(STP_DIR, OUTPUT);
  pinMode(STP_STEP, OUTPUT);
  pinMode(STP_SW_TOP, INPUT_PULLUP);
  pinMode(STP_SW_BOTTOM, INPUT_PULLUP);

  safeInitOutputs();
  delay(500);

#ifdef ENABLE_TEST_MODE
  // 모드 확인
  byte mode = loadModeFlag();
  
  if (mode == MODE_TEST) {
    testMode = true;
    DEBUG_SERIAL.println(F("\n\n========================================"));
    DEBUG_SERIAL.println(F("       테스트 모드 시작"));
    DEBUG_SERIAL.println(F("========================================"));
    printTestMenu();
    return;  // setup 종료 - loop에서 테스트 모드 실행
  }
#endif
  
  // 정상 모드 시작
  DEBUG_SERIAL.println(F("\n\n========================================"));
  DEBUG_SERIAL.println(F("     시스템 부팅 시작 (정상 모드)"));
  DEBUG_SERIAL.println(F("     TB6600 + KH42 + 동시 홈잉"));
  DEBUG_SERIAL.println(F("     블렌더 예비회전 + 느린초기하강"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("시리얼 포트 설정:"));
  DEBUG_SERIAL.println(F("  - Serial (USB): 디버깅"));
  DEBUG_SERIAL.println(F("  - Serial3 (D14/D15): PC 제어"));
  DEBUG_SERIAL.println(F("========================================"));
  
#ifdef ENABLE_TEST_MODE
  DEBUG_SERIAL.println(F("\n★★★ 테스트 모드 진입 방법 ★★★"));
  DEBUG_SERIAL.println(F("  [t] - 재부팅 후 테스트 모드 (EEPROM 저장)"));
  DEBUG_SERIAL.println(F("  [s] - 즉시 테스트 모드 (일회성)"));
  DEBUG_SERIAL.println(F("★★★ 10초 대기 중... ★★★\n"));
  
  // 테스트 모드 전환 대기 (10초)
  unsigned long bootTime = millis();
  while (millis() - bootTime < 10000) {
    if (DEBUG_SERIAL.available() > 0) {
      char c = DEBUG_SERIAL.read();
      if (c == 't' || c == 'T') {
        DEBUG_SERIAL.println(F("\n테스트 모드로 전환합니다..."));
        saveModeFlag(MODE_TEST);
        delay(1000);
        asm volatile ("jmp 0");  // 소프트웨어 리셋
      }
      else if (c == 's' || c == 'S') {
        DEBUG_SERIAL.println(F("\n즉시 테스트 모드로 진입합니다..."));
        delay(500);
        testMode = true;
        relayOff(RELAY_LED);
        DEBUG_SERIAL.println(F("\n\n========================================"));
        DEBUG_SERIAL.println(F("       테스트 모드 시작"));
        DEBUG_SERIAL.println(F("========================================"));
        printTestMenu();
        return;  // setup 종료 - loop에서 테스트 모드 실행
      }
    }
    // LED 깜빡임으로 대기 표시
    digitalWrite(RELAY_LED, (millis() / 200) % 2);
  }
  relayOff(RELAY_LED);
#endif

  // 안전 부팅 - 항상 홈잉 수행
  DEBUG_SERIAL.println(F("\n안전 부팅 - 홈잉 수행"));
  clearEEPROM();
  runState = HOMING;

  // 홈잉
  if (runState == HOMING) {
    DEBUG_SERIAL.println(F("\n홈잉 시작..."));
    if (homeAll()) {
      runState = READY;
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  홈잉 완료 - 대기 상태"));
      DEBUG_SERIAL.println(F("  시스템 준비 완료"));
      DEBUG_SERIAL.println(F("========================================\n"));
    } else {
      runState = ERROR_STATE;
      DEBUG_SERIAL.println(F("\n✗✗✗ 홈잉 실패 - 오류 상태 ✗✗✗\n"));
    }
  }
}

/* ================= LOOP ================= */

void loop(){
#ifdef ENABLE_TEST_MODE
  if (testMode) {
    handleTestMode();
    return;
  }
#endif

  // PC 통신 처리
  processSerialInput();

  // 상태 LED 업데이트
  updateStatusLED();

  // 버튼 처리
  if (btnStartPressed()) {
    if (runState == READY) {
      if (photoDetectedNow()) {
        DEBUG_SERIAL.println(F("\n>>> 버튼으로 시작 명령 수신"));
        stepIndex = 0;
        runState = RUNNING;
        saveRunFlag(RUNFLAG_RUNNING);
        saveStepIndex(0);
      } else {
        DEBUG_SERIAL.println(F("\n!!! 포토 센서 감지 안됨 - 시작 불가 !!!"));
      }
    } else if (runState == CLEANING_WAIT_CONFIRM) {
      DEBUG_SERIAL.println(F("\n>>> 세척 시작 버튼 누름"));
      stepIndex = 0;
      runState = CLEANING_DESCEND;
    }
  }

  // 상태별 처리
  if (runState == ERROR_STATE) {
    return;
  }

  if (runState == COMPLETION_FLASH) {
    static int flashCount = 0;
    static unsigned long flashTimer = 0;
    static bool ledOn = false;
    
    if (flashCount == 0) {
      flashTimer = millis();
      flashCount = 1;
    }
    
    if (millis() - flashTimer >= 500) {
      ledOn = !ledOn;
      digitalWrite(RELAY_LED, ledOn ? RELAY_ON : RELAY_OFF);
      flashTimer = millis();
      flashCount++;
      
      if (flashCount > 20) {
        flashCount = 0;
        relayOff(RELAY_LED);
        runState = WAITING_FOR_REMOVAL;
        DEBUG_SERIAL.println(F("\n========================================"));
        DEBUG_SERIAL.println(F("  완료 신호 종료"));
        DEBUG_SERIAL.println(F("  컵 제거 대기 중..."));
        DEBUG_SERIAL.println(F("========================================\n"));
      }
    }
    return;
  }

  if (runState == WAITING_FOR_REMOVAL) {
    if (photoObjectRemoved()) {
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  컵 제거 감지"));
      DEBUG_SERIAL.println(F("  세척 확인 대기 중..."));
      DEBUG_SERIAL.println(F("  버튼을 눌러 세척 시작"));
      DEBUG_SERIAL.println(F("========================================\n"));
      runState = CLEANING_WAIT_CONFIRM;
    }
    return;
  }

  if (runState == CLEANING_DESCEND) {
    switch (stepIndex) {
      case 0:
        DEBUG_SERIAL.println(F("[세척 하강] DC 모터 + 스테퍼 동시 하강 시작"));
        dcDown();  // DC 모터 하강 시작
        stepperStartMove(cleaningStepperPosition, true);  // 스테퍼 하강 시작
        stepTimer = millis();
        stepIndex = 1;
        break;
        
      case 1:
        // 스테퍼 계속 이동
        stepperMoveDone();
        
        // DC 모터 시간 체크
        if (millis() - stepTimer >= cleaningDcDownTime) {
          dcStop();
          DEBUG_SERIAL.println(F("[세척 하강] DC 모터 정지"));
          stepIndex = 2;
        }
        break;
        
      case 2:
        // 스테퍼 완료 대기
        stepperMoveDone();
        if (stepperCurrentStep >= stepperTargetSteps) {
          DEBUG_SERIAL.println(F("[세척 하강] 스테퍼 하강 완료"));
          DEBUG_SERIAL.println(F("[세척 하강] 완료 - 세척 프로세스 시작"));
          delay(200);
          stepIndex = 0;
          runState = CLEANING_PROCESS;
        }
        break;
    }
    return;
  }
  
  if (runState == CLEANING_PROCESS) {
    switch (stepIndex) {
      case 0:
        DEBUG_SERIAL.println(F("[세척] 밸브2 열기"));
        relayOn(RELAY_VALVE2);
        DEBUG_SERIAL.println(F("[세척] 블렌더 시작"));
        ssrOn(SSR_BLENDER);
        stepTimer = millis();
        stepIndex = 1;
        break;
        
      case 1:
        if (millis() - stepTimer >= 500) {
          DEBUG_SERIAL.println(F("[세척] 펌프 시작 (0.5초 후)"));
          relayOn(RELAY_PUMP);
          stepTimer = millis();
          stepIndex = 2;
        }
        break;
        
      case 2:
        if (millis() - stepTimer >= 5000) {
          DEBUG_SERIAL.println(F("[세척] 5초 경과 - 밸브1 열기"));
          relayOn(RELAY_VALVE1);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] 밸브2 닫기 (즉시)"));
          relayOff(RELAY_VALVE2);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] 스테퍼 50%까지 상승 시작"));
          // 50% 위치 = cleaningStepperPosition / 2
          stepperStartMove(cleaningStepperPosition / 2, false);
          stepTimer = millis();
          stepIndex = 3;
        }
        break;
        
      case 3:
        // 50% 위치까지 상승
        stepperMoveDone();
        if (stepperCurrentStep >= stepperTargetSteps) {
          DEBUG_SERIAL.println(F("[세척] 스테퍼 50% 도착 - 10초 정지"));
          stepTimer = millis();
          stepIndex = 31;  // 새로운 중간 단계
        }
        break;
        
      case 31:
        // 10초 대기
        if (millis() - stepTimer >= 10000) {
          DEBUG_SERIAL.println(F("[세척] 10초 정지 완료 - 나머지 50% 상승 시작"));
          DEBUG_SERIAL.println(F("[세척] 밸브1 닫기"));
          relayOff(RELAY_VALVE1);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] 밸브2 열기"));
          relayOn(RELAY_VALVE2);
          delay(200);
          // 나머지 50% 상승
          stepperStartMove(cleaningStepperPosition / 2, false);
          stepTimer = millis();
          stepIndex = 32;
        }
        break;
        
      case 32:
        // 나머지 50% 상승 (홈까지)
        stepperMoveDone();
        if (stepperCurrentStep >= stepperTargetSteps) {
          DEBUG_SERIAL.println(F("[세척] 스테퍼 홈 도착 완료"));
          DEBUG_SERIAL.println(F("[세척] 블렌더 정지"));
          ssrOff(SSR_BLENDER);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] 펌프 정지"));
          relayOff(RELAY_PUMP);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] 밸브2 정지"));
          relayOff(RELAY_VALVE2);
          delay(200);
          DEBUG_SERIAL.println(F("[세척] DC 모터 홈 복귀 시작"));
          stepIndex = 0;
          runState = CLEANING_RETURN_HOME;
        }
        break;
    }
    return;
  }
  
  if (runState == CLEANING_RETURN_HOME) {
    if (homeDCOnly()) {
      clearEEPROM();
      
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  DC 홈 복귀 완료"));
      DEBUG_SERIAL.println(F("  스테퍼 홈잉 시작"));
      DEBUG_SERIAL.println(F("========================================\n"));
      
      // 스테퍼 홈잉 수행
      if (homeStepperOnly()) {
        runState = READY;
        DEBUG_SERIAL.println(F("\n========================================"));
        DEBUG_SERIAL.println(F("  세척 완료 - 대기 상태"));
        DEBUG_SERIAL.println(F("  다음 서비스 준비 완료"));
        DEBUG_SERIAL.println(F("========================================\n"));
      } else {
        runState = ERROR_STATE;
        DEBUG_SERIAL.println(F("!!!세척 후 스테퍼 홈잉 실패!!!"));
      }
    } else {
      runState = ERROR_STATE;
      DEBUG_SERIAL.println(F("!!!세척 후 DC 홈 복귀 실패!!!"));
    }
    return;
  }
  
  if (runState == RUNNING) {
    // 작업 중 컵 제거 감지
    if (photoInterlockEnabled() && !photoDetectedNow()) {
      DEBUG_SERIAL.println(F("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
      DEBUG_SERIAL.println(F("  경고: 작업 중 컵 제거 감지!"));
      DEBUG_SERIAL.println(F("  비상 정지 - 홈 복귀 시작"));
      DEBUG_SERIAL.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
      
      safeInitOutputs();
      emergencyStopTriggered = true;
      runState = EMERGENCY_RETURN_HOME;
      saveStepIndex(0);
      clearEEPROM();
      return;
    }
    
    switch (stepIndex) {
      case 0:
        DEBUG_SERIAL.println(F("[단계 0] DC 모터 하강 시작 (6초)"));
        dcDown();
        stepTimer = millis();
        stepIndex = 1;
        saveStepIndex(1);
        break;
        
      case 1:
        if (millis() - stepTimer >= 6000) {
          dcStop();
          delay(200);
          cleaningDcDownTime = 6000;
          DEBUG_SERIAL.println(F("[단계 1] DC 모터 정지"));
          stepIndex = 2;
          saveStepIndex(2);
        }
        break;
        
      case 2:
        DEBUG_SERIAL.println(F("[단계 2] 밸브1 열기"));
        relayOn(RELAY_VALVE1);
        stepTimer = millis();
        stepIndex = 21;
        saveStepIndex(21);
        break;
        
      case 21:
        if (millis() - stepTimer >= 500) {
          DEBUG_SERIAL.println(F("[단계 2-1] 펌프 시작 (0.5초 후)"));
          relayOn(RELAY_PUMP);
          stepTimer = millis();
          stepIndex = 22;
          saveStepIndex(22);
        }
        break;
        
      case 22:
        if (millis() - stepTimer >= 12000) {
          stepIndex = 23;
          saveStepIndex(23);
        }
        break;
        
      case 23:
        DEBUG_SERIAL.println(F("[단계 2-3] 펌프 정지"));
        relayOff(RELAY_PUMP);
        delay(200);
        DEBUG_SERIAL.println(F("[단계 2-4] 밸브1 정지"));
        relayOff(RELAY_VALVE1);
        delay(200);
        DEBUG_SERIAL.println(F("[단계 2] 완료"));
        stepIndex = 3;
        saveStepIndex(3);
        break;
        
      /* ================= 여기서부터 수정된 부분 (블렌더 예비 회전 + 느린 초기 하강) ================= */
      
      case 3:
        DEBUG_SERIAL.println(F("[단계 3] ★★★ 블렌더 예비 회전 시작 (3초) ★★★"));
        ssrOn(SSR_BLENDER);
        stepTimer = millis();
        stepIndex = 30;
        saveStepIndex(30);
        break;

      case 30:
        // 블렌더 예비 회전 3초
        if (millis() - stepTimer >= 3000) {
          DEBUG_SERIAL.println(F("[단계 3-0] ★ 블렌더 준비 완료 ★"));
          DEBUG_SERIAL.println(F("[단계 3-0] 스테퍼 천천히 하강 준비 (첫 1회전)"));
          cleaningStepperPosition = STEPS_PER_REV * 14L;
          stepIndex = 301;
          saveStepIndex(301);
        }
        break;

      case 301:
        DEBUG_SERIAL.println(F("[단계 3-0-1] ★ 첫 1회전 매우 느리게 하강 시작 ★"));
        DEBUG_SERIAL.println(F("[단계 3-0-1] (재료가 서서히 블렌더로 들어감)"));
        stepperStartMove(STEPS_PER_REV, true);  // 1회전만
        stepTimer = millis();
        stepIndex = 302;
        saveStepIndex(302);
        break;

      case 302:
        // 매우 느린 속도로 이동 (첫 1회전 = 약 6초)
        {
          static unsigned long lastBatchTime = 0;
          unsigned long now = millis();
          
          if (lastBatchTime == 0) {
            lastBatchTime = now;
          }
          
          // 15ms마다 5스텝 = 약 333스텝/초 = 6초에 1회전 (일반 1초보다 6배 느림)
          if (now - lastBatchTime >= 15) {
            for (int i = 0; i < 5 && stepperCurrentStep < stepperTargetSteps; i++) {
              digitalWrite(STP_STEP, HIGH);
              delayMicroseconds(10);
              digitalWrite(STP_STEP, LOW);
              delayMicroseconds(10);
              stepperCurrentStep++;
            }
            lastBatchTime = now;
          }
          
          if (stepperCurrentStep >= stepperTargetSteps) {
            lastBatchTime = 0;
            DEBUG_SERIAL.println(F("[단계 3-0-2] ★ 첫 1회전 완료 (6초 소요) ★"));
            DEBUG_SERIAL.println(F("[단계 3-0-2] ★ 이제 정상 속도로 전환 ★"));
            delay(500);
            DEBUG_SERIAL.println(F("[단계 3-1] 블렌더 펄스 + 나머지 하강 시작"));
            stepIndex = 31;
            saveStepIndex(31);
          }
        }
        break;

      /* ================= 여기부터 기존 펄스 블렌딩 로직 (블렌더는 이미 ON 상태) ================= */
      
      case 31:
        DEBUG_SERIAL.println(F("[단계 3-1] 블렌더 1회 유지 + 스테퍼 1/5회전 하강"));
        // 블렌더는 이미 켜져 있음 (case 3에서 시작)
        stepperStartMove(STEPS_PER_REV / 5, true);  // 1/5 회전 (640 스텝) 하강
        stepTimer = millis();
        stepIndex = 32;
        saveStepIndex(32);
        break;

      case 32:
        stepperMoveDone();  // 스테퍼 계속 구동
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-2] 블렌더 1회 OFF (1/5회전 완료)"));
          ssrOff(SSR_BLENDER);
          stepTimer = millis();
          stepIndex = 33;
          saveStepIndex(33);
        }
        break;

      case 33:
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-3] 블렌더 2회 ON + 스테퍼 1/5회전 하강"));
          ssrOn(SSR_BLENDER);
          stepperStartMove(STEPS_PER_REV / 5, true);  // 1/5 회전 (640 스텝) 하강
          stepTimer = millis();
          stepIndex = 34;
          saveStepIndex(34);
        }
        break;

      case 34:
        stepperMoveDone();  // 스테퍼 계속 구동
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-4] 블렌더 2회 OFF (2/5회전 완료)"));
          ssrOff(SSR_BLENDER);
          stepTimer = millis();
          stepIndex = 35;
          saveStepIndex(35);
        }
        break;

      case 35:
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-5] 블렌더 3회 ON + 스테퍼 1/5회전 하강"));
          ssrOn(SSR_BLENDER);
          stepperStartMove(STEPS_PER_REV / 5, true);  // 1/5 회전 (640 스텝) 하강
          stepTimer = millis();
          stepIndex = 36;
          saveStepIndex(36);
        }
        break;

      case 36:
        stepperMoveDone();  // 스테퍼 계속 구동
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-6] 블렌더 3회 OFF (3/5회전 완료)"));
          ssrOff(SSR_BLENDER);
          stepTimer = millis();
          stepIndex = 37;
          saveStepIndex(37);
        }
        break;

      case 37:
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-7] 블렌더 4회 ON + 스테퍼 1/5회전 하강"));
          ssrOn(SSR_BLENDER);
          stepperStartMove(STEPS_PER_REV / 5, true);  // 1/5 회전 (640 스텝) 하강
          stepTimer = millis();
          stepIndex = 38;
          saveStepIndex(38);
        }
        break;

      case 38:
        stepperMoveDone();  // 스테퍼 계속 구동
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-8] 블렌더 4회 OFF (4/5회전 완료)"));
          ssrOff(SSR_BLENDER);
          stepTimer = millis();
          stepIndex = 39;
          saveStepIndex(39);
        }
        break;

      case 39:
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-9] 블렌더 5회 ON + 스테퍼 1/5회전 하강"));
          ssrOn(SSR_BLENDER);
          stepperStartMove(STEPS_PER_REV / 5, true);  // 1/5 회전 (640 스텝) 하강
          stepTimer = millis();
          stepIndex = 40;
          saveStepIndex(40);
        }
        break;

      case 40:
        stepperMoveDone();  // 스테퍼 계속 구동
        if (millis() - stepTimer >= 1000) {
          DEBUG_SERIAL.println(F("[단계 3-10] 블렌더 5회 OFF (펄스 1회전 완료!)"));
          ssrOff(SSR_BLENDER);
          delay(200);
          DEBUG_SERIAL.println(F("[단계 3-11] 블렌더 다시 ON (나머지 13회전 하강)"));  // ← 여기 메시지 수정
          ssrOn(SSR_BLENDER);
          delay(200);
          DEBUG_SERIAL.println(F("[단계 3-12] 스테퍼 나머지 하강 시작 (13회전 = 60초)"));  // ← 여기 메시지 수정
          stepperStartMove(STEPS_PER_REV * 13L, true);  // ← 여기를 13L로 수정
          slowStepperStartTime = millis();
          stepTimer = millis();
          stepIndex = 41;
          saveStepIndex(41);
  }
  break;

      case 41:
        {
          static unsigned long lastPrint = 0;
          unsigned long elapsed = millis() - stepTimer;
          if (elapsed - lastPrint >= 10000) {
            DEBUG_SERIAL.print(F("[단계 3-12] 진행 중... "));
            DEBUG_SERIAL.print(elapsed / 1000);
            DEBUG_SERIAL.print(F("초 / "));
            DEBUG_SERIAL.print(stepperCurrentStep);
            DEBUG_SERIAL.print(F(" 스텝 ("));
            DEBUG_SERIAL.print((stepperCurrentStep * 100) / stepperTargetSteps);
            DEBUG_SERIAL.println(F("%)"));
            lastPrint = elapsed;
          }
        }
        
        if (stepperMoveDoneSlow120s()) {
          ssrOff(SSR_BLENDER);
          delay(200);
          DEBUG_SERIAL.println(F("[단계 3-13] 스테퍼 하강 완료! (총 14회전)"));
          DEBUG_SERIAL.println(F("[단계 3-13] 블렌더 정지"));
          stepIndex = 4;
          stepTimer = millis();
          saveStepIndex(4);
        }
        break;
        
      case 4:
        if (millis() - stepTimer >= 3000) {
          DEBUG_SERIAL.println(F("[단계 4] 대기 완료 -> 홈 복귀"));
          runState = RETURNING_HOME;
          saveStepIndex(5);
        }
        break;
    }
  }

  if (runState == RETURNING_HOME) {
    static bool returnStarted = false;
    
    if (!returnStarted) {
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  작업 완료 - 홈 복귀 시작"));
      DEBUG_SERIAL.println(F("========================================"));
      
      // 블렌더만 켜고 나머지는 정지
      relayOff(RELAY_VALVE1);
      relayOff(RELAY_VALVE2);
      relayOff(RELAY_PUMP);
      ssrOff(SSR_HEATER);
      ssrOn(SSR_BLENDER);  // 블렌더는 스테퍼 복귀 중 작동!
      dcStop();
      
      delay(500);
      
      DEBUG_SERIAL.println(F("[홈복귀] 블렌더 작동 시작 (스테퍼 복귀 중)"));
      DEBUG_SERIAL.println(F("[홈복귀] 스테퍼 상승 시작"));
      stepperStartMove(cleaningStepperPosition, false);
      returnStarted = true;
    }
    
    if (stepperMoveDone()) {
      if (stepperCurrentStep >= stepperTargetSteps) {
        DEBUG_SERIAL.println(F("[홈복귀] 스테퍼 완료"));
        
        // 스테퍼 완료 후 블렌더 정지
        ssrOff(SSR_BLENDER);
        DEBUG_SERIAL.println(F("[홈복귀] 블렌더 정지 (스테퍼 도착)"));
        delay(200);
        
        DEBUG_SERIAL.println(F("[홈복귀] DC 모터 상승 시작"));
        dcUp();
        delay(cleaningDcDownTime);
        dcStop();
        delay(200);
        DEBUG_SERIAL.println(F("[홈복귀] DC 모터 완료"));
        
        returnStarted = false;
        clearEEPROM();
        runState = COMPLETION_FLASH;
        
        DEBUG_SERIAL.println(F("\n========================================"));
        DEBUG_SERIAL.println(F("  홈 복귀 완료"));
        DEBUG_SERIAL.println(F("  완료 신호 시작"));
        DEBUG_SERIAL.println(F("========================================\n"));
      }
    }
    return;
  }

  if (runState == EMERGENCY_RETURN_HOME) {
    static bool emergencyStarted = false;
    
    if (!emergencyStarted) {
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  비상 홈 복귀 시작"));
      DEBUG_SERIAL.println(F("========================================"));
      emergencyStarted = true;
    }
    
    if (homeAll()) {
      emergencyStarted = false;
      emergencyStopTriggered = false;
      cleaningStepperPosition = 0;  // 초기화 추가
      cleaningDcDownTime = 0;        // 초기화 추가
      runState = READY;
      DEBUG_SERIAL.println(F("\n========================================"));
      DEBUG_SERIAL.println(F("  비상 홈 복귀 완료 - 대기 상태"));
      DEBUG_SERIAL.println(F("========================================\n"));
    } else {
      runState = ERROR_STATE;
      DEBUG_SERIAL.println(F("!!!비상 홈 복귀 실패!!!"));
    }
    return;
  }
}
