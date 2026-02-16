#include <SPI.h>
#include <RF24.h>

enum CmdKind : uint8_t {
  CK_NONE = 0,
  CK_ALL,
  CK_LASER,
  CK_SERVOS,
  CK_RADIO,
  CK_MCU,
  CK_SCAN_H,
  CK_SCAN_V,
  CK_SCAN_D1,
  CK_SCAN_D2,
  CK_ZERO,
  CK_SCAN_FULL
};

enum DevMode : uint8_t {
  DM_IDLE = 0,
  DM_WAIT,
  DM_H,
  DM_V,
  DM_D1,
  DM_D2,
  DM_TEST
};

enum DiagType : uint8_t {
  DT_NONE = 0,
  DT_LASER,
  DT_SERVOS,
  DT_RADIO,
  DT_MCU,
  DT_ALL
};

struct CmdFrame {
  uint8_t  devId;
  CmdKind  kind;
};

struct TelemetryFrame {
  uint8_t  devId;
  int8_t   tilt;
  int8_t   pan;
  DevMode  mode;
};

struct DiagFrame {
  uint8_t  devId;
  DiagType kind;
  bool     started;
  bool     done;
  bool     ok;
};

const uint8_t RF_CE_PIN  = 7;
const uint8_t RF_CSN_PIN = 8;

RF24 rf(RF_CE_PIN, RF_CSN_PIN);

const byte GROUND_ADDR[6] = "BASE1";
const byte NODE_ADDR[6]   = "CUBE1";

const uint8_t NODE_TARGET_ID = 1;

void pushCommand(CmdKind k) {
  CmdFrame f;
  f.devId = NODE_TARGET_ID;
  f.kind  = k;

  rf.stopListening();
  rf.openWritingPipe(NODE_ADDR);
  bool sentOk = rf.write(&f, sizeof(f));
  rf.startListening();

  Serial.print("CMD=");
  Serial.print((int)k);
  Serial.print(" res=");
  Serial.println(sentOk ? "OK" : "FAIL");
}

const char* modeName(DevMode m) {
  switch (m) {
    case DM_IDLE: return "IDLE";
    case DM_WAIT: return "WAIT";
    case DM_H:    return "SCAN_H";
    case DM_V:    return "SCAN_V";
    case DM_D1:   return "SCAN_D1";
    case DM_D2:   return "SCAN_D2";
    case DM_TEST: return "TEST";
    default:      return "UNK";
  }
}

const char* testName(DiagType t) {
  switch (t) {
    case DT_LASER: return "LASER";
    case DT_SERVOS:return "SERVOS";
    case DT_RADIO: return "RADIO";
    case DT_MCU:   return "MCU";
    case DT_ALL:   return "ALL";
    default:       return "NONE";
  }
}

void printHelp() {
  Serial.println("Base console ready");
  Serial.println("Keys:");
  Serial.println(" 1 - ALL TESTS");
  Serial.println(" 2 - LASER TEST");
  Serial.println(" 3 - SERVOS TEST");
  Serial.println(" 4 - RADIO TEST");
  Serial.println(" 5 - MCU TEST");
  Serial.println(" h - H SCAN");
  Serial.println(" v - V SCAN");
  Serial.println(" d - DIAG1 SCAN");
  Serial.println(" f - DIAG2 SCAN");
  Serial.println(" s - FULL SCAN");
  Serial.println(" 0 - ZERO POSITION");
}

void handleSerialChar(char c) {
  switch (c) {
    case '1': pushCommand(CK_ALL);        break;
    case '2': pushCommand(CK_LASER);      break;
    case '3': pushCommand(CK_SERVOS);     break;
    case '4': pushCommand(CK_RADIO);      break;
    case '5': pushCommand(CK_MCU);        break;
    case 'h': pushCommand(CK_SCAN_H);     break;
    case 'v': pushCommand(CK_SCAN_V);     break;
    case 'd': pushCommand(CK_SCAN_D1);    break;
    case 'f': pushCommand(CK_SCAN_D2);    break;
    case 's': pushCommand(CK_SCAN_FULL);  break;
    case '0': pushCommand(CK_ZERO);       break;
    default: break;
  }
}

void printTestFrame(const DiagFrame* df) {
  Serial.print("[T] id=");
  Serial.print(df->devId);
  Serial.print(" t=");
  Serial.print(testName(df->kind));
  Serial.print(" start=");
  Serial.print(df->started ? "1" : "0");
  Serial.print(" end=");
  Serial.print(df->done ? "1" : "0");
  Serial.print(" ok=");
  Serial.println(df->ok ? "1" : "0");
}

void printTelFrame(const TelemetryFrame* tf) {
  Serial.print("[M] id=");
  Serial.print(tf->devId);
  Serial.print(" mode=");
  Serial.print(modeName(tf->mode));
  Serial.print(" tilt=");
  Serial.print(tf->tilt);
  Serial.print(" pan=");
  Serial.println(tf->pan);
}

void handleRadioPacket() {
  uint8_t raw[32];
  rf.read(&raw, sizeof(raw));

  DiagFrame* df = (DiagFrame*)raw;
  bool plausibleTest =
    df->devId == NODE_TARGET_ID &&
    df->kind >= DT_LASER &&
    df->kind <= DT_ALL;

  if (plausibleTest) {
    printTestFrame(df);
  } else {
    TelemetryFrame* tf = (TelemetryFrame*)raw;
    printTelFrame(tf);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  printHelp();

  rf.begin();
  rf.setChannel(90);
  rf.setDataRate(RF24_250KBPS);
  rf.setPALevel(RF24_PA_LOW);
  rf.openReadingPipe(1, GROUND_ADDR);
  rf.startListening();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    handleSerialChar(c);
  }

  if (rf.available()) {
    handleRadioPacket();
  }
}
