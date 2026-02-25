//the IR codes for each command may vary
//the speed of the linear actuator was measured and then used in the script
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>

// ==================== PIN DEFINITIONS ====================
const int ENA_PIN   = 7;
const int IN1_PIN   = 6;
const int IN2_PIN   = 5;

const int RELAY_PIN = 4;     // relay control pin
const uint8_t IR_PIN = 2;    // IR receiver Y -> D2

// Relay polarity (flip if your relay is active-low)
const int RELAY_ON_LEVEL  = HIGH;
const int RELAY_OFF_LEVEL = LOW;

// ==================== LCD ====================
// Change 0x27 to 0x3F if your LCD uses that address
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== CALIBRATION ====================
float time_per_cm_ms = 1050.0f;
const unsigned long HOME_RETRACT_MS = 20000;
static const unsigned long DEBOUNCE_MS = 200;

// ==================== IR CODES ====================
// NEC commands
static const uint8_t CMD_POWER = 0x45;

// Confirm/Cancel keys using RIGHT/LEFT
static const uint8_t CMD_RIGHT = 0x43;  // confirm / yes / extend / proceed
static const uint8_t CMD_LEFT  = 0x44;  // cancel / no  / retract / clear

// Digits (confirmed)
static const uint8_t CMD_0 = 0x16;
static const uint8_t CMD_1 = 0x0C;
static const uint8_t CMD_2 = 0x18;
static const uint8_t CMD_3 = 0x5E;
static const uint8_t CMD_4 = 0x08;
static const uint8_t CMD_5 = 0x1C;
static const uint8_t CMD_6 = 0x5A;
static const uint8_t CMD_7 = 0x42;

// Likely for your remote family; update if your 8/9 differ
static const uint8_t CMD_8 = 0x52;
static const uint8_t CMD_9 = 0x4A;

// ==================== TYPES ====================
enum Key {
  KEY_NONE,
  KEY_POWER,
  KEY_CONFIRM,   // RIGHT
  KEY_CANCEL,    // LEFT
  KEY_DIGIT_0, KEY_DIGIT_1, KEY_DIGIT_2, KEY_DIGIT_3, KEY_DIGIT_4,
  KEY_DIGIT_5, KEY_DIGIT_6, KEY_DIGIT_7, KEY_DIGIT_8, KEY_DIGIT_9
};

// ==================== LOW-LEVEL HELPERS ====================
static inline void relayOff() { digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL); }
static inline void relayOn()  { digitalWrite(RELAY_PIN, RELAY_ON_LEVEL);  }

void stopActuator() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void homeActuator() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  delay(HOME_RETRACT_MS);
  stopActuator();
}

void extendDistance(float cm) {
  unsigned long duration = (unsigned long)(cm * time_per_cm_ms);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  delay(duration);
  stopActuator();
}

void retractDistance(float cm) {
  unsigned long duration = (unsigned long)(cm * time_per_cm_ms);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  delay(duration);
  stopActuator();
}

// UNO R4-safe simple reset for this workflow
void softReset() {
  void (*resetFunc)(void) = 0;
  resetFunc();
}

// ==================== LCD HELPERS ====================
void lcdShow(const char* top, const String& bottom) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(top);
  lcd.setCursor(0, 1);
  lcd.print(bottom);
}

// ==================== IR HELPERS ====================
int keyToDigit(Key k) {
  switch (k) {
    case KEY_DIGIT_0: return 0;
    case KEY_DIGIT_1: return 1;
    case KEY_DIGIT_2: return 2;
    case KEY_DIGIT_3: return 3;
    case KEY_DIGIT_4: return 4;
    case KEY_DIGIT_5: return 5;
    case KEY_DIGIT_6: return 6;
    case KEY_DIGIT_7: return 7;
    case KEY_DIGIT_8: return 8;
    case KEY_DIGIT_9: return 9;
    default: return -1;
  }
}

Key getKeyOnce() {
  static uint32_t lastRaw = 0;
  static uint8_t lastCmd = 0;
  static unsigned long lastMs = 0;

  if (!IrReceiver.decode()) return KEY_NONE;

  auto &d = IrReceiver.decodedIRData;
  uint32_t raw = d.decodedRawData;           // ✅ FIXED
  bool isRepeat = (d.flags & IRDATA_FLAGS_IS_REPEAT);

  IrReceiver.resume();

  if (isRepeat) return KEY_NONE;
  if (raw == 0x0) return KEY_NONE;

  unsigned long now = millis();
  if ((now - lastMs) < DEBOUNCE_MS) {
    if (raw == lastRaw) return KEY_NONE;
    if (d.protocol == NEC && (uint8_t)d.command == lastCmd) return KEY_NONE;
  }
  lastMs = now;
  lastRaw = raw;
  lastCmd = (uint8_t)d.command;

  if (d.protocol != NEC) return KEY_NONE;

  uint8_t cmd = (uint8_t)d.command;

  if (cmd == CMD_POWER) return KEY_POWER;
  if (cmd == CMD_RIGHT) return KEY_CONFIRM;
  if (cmd == CMD_LEFT)  return KEY_CANCEL;

  if (cmd == CMD_0) return KEY_DIGIT_0;
  if (cmd == CMD_1) return KEY_DIGIT_1;
  if (cmd == CMD_2) return KEY_DIGIT_2;
  if (cmd == CMD_3) return KEY_DIGIT_3;
  if (cmd == CMD_4) return KEY_DIGIT_4;
  if (cmd == CMD_5) return KEY_DIGIT_5;
  if (cmd == CMD_6) return KEY_DIGIT_6;
  if (cmd == CMD_7) return KEY_DIGIT_7;
  if (cmd == CMD_8) return KEY_DIGIT_8;
  if (cmd == CMD_9) return KEY_DIGIT_9;

  return KEY_NONE;
}

// ==================== INPUT UI ====================
String readNumber(const char* prompt) {
  String entry = "";
  lcdShow(prompt, entry);

  while (true) {
    Key k = getKeyOnce();
    if (k == KEY_NONE) continue;

    if (k == KEY_POWER) {
      lcdShow("RESET", "");
      delay(200);
      softReset();
    }

    if (k == KEY_CANCEL) {
      entry = "";
      lcdShow(prompt, entry);
      continue;
    }

    if (k == KEY_CONFIRM) {
      if (entry.length() == 0) entry = "0";
      return entry;
    }

    int d = keyToDigit(k);
    if (d >= 0) {
      entry += char('0' + d);
      lcdShow(prompt, entry);
    }
  }
}

int readDirection() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DIR");
  lcd.setCursor(0, 1);
  lcd.print("R=EXT L=RET");

  while (true) {
    Key k = getKeyOnce();
    if (k == KEY_NONE) continue;

    if (k == KEY_POWER) {
      lcdShow("RESET", "");
      delay(200);
      softReset();
    }

    if (k == KEY_CONFIRM) return +1;  // RIGHT
    if (k == KEY_CANCEL)  return -1;  // LEFT
  }
}

bool readAgain() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AGAIN?");
  lcd.setCursor(0, 1);
  lcd.print("R=Y L=N");

  while (true) {
    Key k = getKeyOnce();
    if (k == KEY_NONE) continue;

    if (k == KEY_POWER) {
      lcdShow("RESET", "");
      delay(200);
      softReset();
    }

    if (k == KEY_CONFIRM) return true;
    if (k == KEY_CANCEL)  return false;
  }
}

// ==================== STIMULATION ====================
void runStimulation(float totalSeconds, float periodSeconds, float dutyCycle) {
  if (totalSeconds <= 0 || periodSeconds <= 0) {
    relayOff();
    return;
  }
  if (dutyCycle < 0) dutyCycle = 0;
  if (dutyCycle > 1) dutyCycle = 1;

  unsigned long totalMs  = (unsigned long)(totalSeconds * 1000.0f);
  unsigned long periodMs = (unsigned long)(periodSeconds * 1000.0f);

  unsigned long onMs  = (unsigned long)(periodMs * dutyCycle);
  unsigned long offMs = periodMs - onMs;

  lcdShow("STIM", "RUN");
  relayOff();

  unsigned long start = millis();
  while ((millis() - start) < totalMs) {
    unsigned long elapsed = millis() - start;
    unsigned long remaining = (elapsed < totalMs) ? (totalMs - elapsed) : 0;

    if (onMs > 0 && remaining > 0) {
      relayOn();
      unsigned long thisOn = (onMs > remaining) ? remaining : onMs;
      delay(thisOn);
    } else {
      relayOff();
    }

    elapsed = millis() - start;
    remaining = (elapsed < totalMs) ? (totalMs - elapsed) : 0;

    if (offMs > 0 && remaining > 0) {
      relayOff();
      unsigned long thisOff = (offMs > remaining) ? remaining : offMs;
      delay(thisOff);
    }
  }

  relayOff();
  lcdShow("STIM", "DONE");
  delay(800);
}

// ==================== MAIN ====================
void setup() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);

  pinMode(RELAY_PIN, OUTPUT);
  relayOff();

  Wire.begin();
  lcd.init();
  lcd.backlight();

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  lcdShow("BOOT", "RETRACT");
  relayOff();
  homeActuator();

  lcdShow("READY", "");
  delay(600);
}

void loop() {
  // Ask distance magnitude
  String distStr = readNumber("DIST");
  float distCm = distStr.toFloat();

  // Choose direction if dist > 0
  int dir = +1;
  if (distCm > 0.0f) {
    dir = readDirection(); // RIGHT extend, LEFT retract
  }

  // Move actuator
  lcdShow("MOVE", distStr);
  delay(300);

  if (distCm > 0.0f) {
    if (dir > 0) extendDistance(distCm);
    else         retractDistance(distCm);
  }

  lcdShow("MOVE", "DONE");
  delay(600);

  // Stimulation loop at location
  while (true) {
    String durStr = readNumber("DUR(s)");
    float totalS = durStr.toFloat();

    String perStr = readNumber("PER(s)");
    float perS = perStr.toFloat();

    String dutyStr = readNumber("DUTY0-10");
    int duty10 = constrain(dutyStr.toInt(), 0, 10);
    float duty = duty10 / 10.0f;

    // quick summary
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print((int)totalS);
    lcd.print(" P:");
    lcd.print((int)perS);
    lcd.setCursor(0, 1);
    lcd.print("DC:");
    lcd.print(duty10);
    lcd.print("/10");
    delay(900);

    runStimulation(totalS, perS, duty);

    bool again = readAgain();
    if (!again) break;
  }
}
