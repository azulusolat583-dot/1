#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

const uint8_t NRF_CE_PIN      = 9;
const uint8_t NRF_CSN_PIN     = 3;
const uint8_t SERVO_TILT_PIN  = 7;
const uint8_t SERVO_PAN_PIN   = 6;
const uint8_t LASER_PIN       = 5;
const uint8_t STOP_BTN_PIN    = 4;

RF24 rfUnit(NRF_CE_PIN, NRF_CSN_PIN);
Servo tiltAxis;
Servo panAxis;

const byte HUB_ADDR[6]  = "BASE1";
const byte NODE_ADDR[6] = "CUBE1";
const uint8_t NODE_ID   = 1;

const uint16_t LASER_ON_TIME   = 2500;
const uint16_t SERVO_STEP_PAUSE = 400;

enum CmdKind : uint8_t {
  CMD_NOP = 0,
  CMD_RUN_ALL,
  CMD_RUN_LASER,
  CMD_RUN_SERVOS,
  CMD_RUN_RADIO,
  CMD_RUN_MCU,
  CMD_SWEEP_H,
  CMD_SWEEP_V,
  CMD_SWEEP_D1,
  CMD_SWEEP_D2,
  CMD_GOTO_ZERO,
  CMD_SWEEP_FULL
};

enum WorkMode : uint8_t {
  WM_IDLE = 0,
  WM_WAIT,
  WM_H,
  WM_V,
  WM_D1,
  WM_D2,
  WM_DIAG
};

enum CheckKind : uint8_t {
  CHECK_NOP = 0,
  CHECK_LASER,
  CHECK_SERVOS,
  CHECK_RADIO,
  CHECK_MCU,
  CHECK_EVERYTHING
};

struct CmdFrame {
  uint8_t  dev;
  CmdKind  code;
};

struct TelemetryFrame {
  uint8_t  dev;
  int8_t   tiltDeg;
  int8_t   panDeg;
  WorkMode mode;
};

struct TestFrame {
  uint8_t   dev;
  CheckKind test;
  bool      started;
  bool      finished;
  bool      ok;
};

int8_t tiltPosDeg = 0;
int8_t panPosDeg  = 0;
WorkMode currentMode = WM_IDLE;

int degToServoPulse(int8_t a) {
  if (a < -40) a = -40;
  if (a > 40)  a = 40;
  return map(a, -40, 40, 50, 130);
}

bool isStopPressed() {
  return digitalRead(STOP_BTN_PIN) == LOW;
}

void moveHead(int8_t targetTiltDeg, int8_t targetPanDeg, uint16_t stepDelayMs = 20) {
  int destTilt = constrain(targetTiltDeg, -40, 40);
  int destPan  = constrain(targetPanDeg,  -40, 40);

  int startTilt = tiltPosDeg;
  int startPan  = panPosDeg;

  int spanTilt = abs(destTilt - startTilt);
  int spanPan  = abs(destPan  - startPan);
  int steps = max(spanTilt, spanPan);

  if (steps <= 0) {
    tiltAxis.write(degToServoPulse(destTilt));
    panAxis.write(degToServoPulse(destPan));
    tiltPosDeg = destTilt;
    panPosDeg  = destPan;
    return;
  }

  for (int i = 1; i <= steps; ++i) {
    if (isStopPressed()) break;
    int tmpTilt = startTilt + (destTilt - startTilt) * i / steps;
    int tmpPan  = startPan  + (destPan  - startPan)  * i / steps;
    tiltAxis.write(degToServoPulse(tmpTilt));
    panAxis.write(degToServoPulse(tmpPan));
    delay(stepDelayMs);
  }

  tiltPosDeg = destTilt;
  panPosDeg  = destPan;
}

void pushTelemetry() {
  TelemetryFrame t;
  t.dev      = NODE_ID;
  t.tiltDeg  = -tiltPosDeg;
  t.panDeg   =  panPosDeg;
  t.mode     = currentMode;

  rfUnit.stopListening();
  rfUnit.openWritingPipe(HUB_ADDR);
  rfUnit.write(&t, sizeof(t));
  rfUnit.startListening();
}

void pushTestTelemetry(CheckKind type, bool started, bool finished, bool ok) {
  TestFrame t;
  t.dev      = NODE_ID;
  t.test     = type;
  t.started  = started;
  t.finished = finished;
  t.ok       = ok;

  rfUnit.stopListening();
  rfUnit.openWritingPipe(HUB_ADDR);
  rfUnit.write(&t, sizeof(t));
  rfUnit.startListening();
}

void checkLaser() {
  currentMode = WM_DIAG;
  pushTestTelemetry(CHECK_LASER, true, false, true);

  for (int i = 0; i < 3; ++i) {
    if (isStopPressed()) {
      pushTestTelemetry(CHECK_LASER, false, true, false);
      return;
    }
    digitalWrite(LASER_PIN, HIGH);
    delay(300);
    digitalWrite(LASER_PIN, LOW);
    delay(300);
  }

  pushTestTelemetry(CHECK_LASER, false, true, true);
}

void checkServos() {
  currentMode = WM_DIAG;
  pushTestTelemetry(CHECK_SERVOS, true, false, true);

  moveHead(0, 0, 10);
  delay(300);
  if (isStopPressed()) { pushTestTelemetry(CHECK_SERVOS, false, true, false); return; }

  moveHead(-40, -40, 10);
  delay(500);
  if (isStopPressed()) { pushTestTelemetry(CHECK_SERVOS, false, true, false); return; }

  moveHead(40, 40, 10);
  delay(500);
  if (isStopPressed()) { pushTestTelemetry(CHECK_SERVOS, false, true, false); return; }

  moveHead(0, 0, 10);
  delay(300);

  pushTestTelemetry(CHECK_SERVOS, false, true, true);
}

void checkRadio() {
  currentMode = WM_DIAG;
  pushTestTelemetry(CHECK_RADIO, true, false, true);

  for (int i = 0; i < 3; ++i) {
    if (isStopPressed()) {
      pushTestTelemetry(CHECK_RADIO, false, true, false);
      return;
    }
    pushTelemetry();
    delay(500);
  }

  pushTestTelemetry(CHECK_RADIO, false, true, true);
}

void checkMCU() {
  currentMode = WM_DIAG;
  pushTestTelemetry(CHECK_MCU, true, false, true);

  for (int i = 0; i < 2; ++i) {
    if (isStopPressed()) {
      pushTestTelemetry(CHECK_MCU, false, true, false);
      return;
    }
    digitalWrite(LASER_PIN, HIGH);
    delay(100);
    digitalWrite(LASER_PIN, LOW);
    delay(100);
  }

  pushTestTelemetry(CHECK_MCU, false, true, true);
}

void runAllChecks() {
  currentMode = WM_DIAG;
  pushTestTelemetry(CHECK_EVERYTHING, true, false, true);

  checkLaser();
  checkServos();
  checkRadio();
  checkMCU();

  pushTestTelemetry(CHECK_EVERYTHING, false, true, true);
}

void scanHorizontal() {
  currentMode = WM_H;
  for (int tilt = 40; tilt >= -40; tilt -= 10) {
    if (isStopPressed()) break;
    moveHead(tilt, 0);
    delay(100);
    digitalWrite(LASER_PIN, HIGH);
    pushTelemetry();
    delay(LASER_ON_TIME);
    digitalWrite(LASER_PIN, LOW);
    delay(SERVO_STEP_PAUSE);
  }
}

void scanVertical() {
  currentMode = WM_V;
  for (int pan = -40; pan <= 40; pan += 10) {
    if (isStopPressed()) break;
    moveHead(0, pan);
    delay(100);
    digitalWrite(LASER_PIN, HIGH);
    pushTelemetry();
    delay(LASER_ON_TIME);
    digitalWrite(LASER_PIN, LOW);
    delay(SERVO_STEP_PAUSE);
  }
}

void scanDiagOne() {
  currentMode = WM_D1;
  for (int a = -40; a <= 40; a += 10) {
    if (isStopPressed()) break;
    moveHead(-a, a);
    delay(100);
    digitalWrite(LASER_PIN, HIGH);
    pushTelemetry();
    delay(LASER_ON_TIME);
    digitalWrite(LASER_PIN, LOW);
    delay(SERVO_STEP_PAUSE);
  }
}

void scanDiagTwo() {
  currentMode = WM_D2;
  for (int a = -40; a <= 40; a += 10) {
    if (isStopPressed()) break;
    moveHead(-a, -a);
    delay(100);
    digitalWrite(LASER_PIN, HIGH);
    pushTelemetry();
    delay(LASER_ON_TIME);
    digitalWrite(LASER_PIN, LOW);
    delay(SERVO_STEP_PAUSE);
  }
}

void scanFullPattern() {
  scanHorizontal();
  if (isStopPressed()) return;
  scanVertical();
  if (isStopPressed()) return;
  scanDiagOne();
  if (isStopPressed()) return;
  scanDiagTwo();
}

void resetHead() {
  moveHead(0, 0, 10);
  currentMode = WM_IDLE;
  pushTelemetry();
}

void processCmd(const CmdFrame &c) {
  if (c.dev != NODE_ID && c.dev != 255) return;

  switch (c.code) {
    case CMD_RUN_ALL:
      runAllChecks();
      resetHead();
      break;
    case CMD_RUN_LASER:
      checkLaser();
      break;
    case CMD_RUN_SERVOS:
      checkServos();
      break;
    case CMD_RUN_RADIO:
      checkRadio();
      break;
    case CMD_RUN_MCU:
      checkMCU();
      break;
    case CMD_SWEEP_H:
      scanHorizontal();
      resetHead();
      break;
    case CMD_SWEEP_V:
      scanVertical();
      resetHead();
      break;
    case CMD_SWEEP_D1:
      scanDiagOne();
      resetHead();
      break;
    case CMD_SWEEP_D2:
      scanDiagTwo();
      resetHead();
      break;
    case CMD_SWEEP_FULL:
      scanFullPattern();
      resetHead();
      break;
    case CMD_GOTO_ZERO:
      resetHead();
      break;
    default:
      break;
  }

  currentMode = WM_WAIT;
}

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(STOP_BTN_PIN, INPUT_PULLUP);

  tiltAxis.attach(SERVO_TILT_PIN);
  panAxis.attach(SERVO_PAN_PIN);

  moveHead(0, 0, 10);

  rfUnit.begin();
  rfUnit.setChannel(90);
  rfUnit.setDataRate(RF24_250KBPS);
  rfUnit.setPALevel(RF24_PA_LOW);
  rfUnit.openReadingPipe(1, NODE_ADDR);
  rfUnit.startListening();

  currentMode = WM_WAIT;
  delay(500);
  pushTelemetry();
}

void loop() {
  if (isStopPressed()) {
    digitalWrite(LASER_PIN, LOW);
    delay(100);
    return;
  }

  if (rfUnit.available()) {
    CmdFrame incoming;
    rfUnit.read(&incoming, sizeof(incoming));
    processCmd(incoming);
  }

  static unsigned long lastPingMs = 0;
  unsigned long now = millis();
  if (now - lastPingMs > 5000) {
    lastPingMs = now;
    if (currentMode == WM_WAIT || currentMode == WM_IDLE) {
      pushTelemetry();
    }
  }
}
