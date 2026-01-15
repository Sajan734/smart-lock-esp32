#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <vector>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define JOY_VRX_PIN 34    // Joystick X-axis (analog)
#define JOY_VRY_PIN 35    // Joystick Y-axis (analog)
#define JOY_SW_PIN 32     // Joystick button (digital)

#define ENC_CLK_PIN 25    // Rotary encoder CLK
#define ENC_DT_PIN 26     // Rotary encoder DT
#define ENC_SW_PIN 27     // Rotary encoder button

#define KNOCK_PIN 33      // Knock sensor digital output (DO only)

#define LCD_SDA 21        // I2C SDA (default ESP32)
#define LCD_SCL 22        // I2C SCL (default ESP32)
#define LCD_ADDR 0x27     // Common I2C address (try 0x3F if not working)

// ============================================================================
// SYSTEM CONSTANTS
// ============================================================================
#define JOYSTICK_CENTER 2048
#define JOYSTICK_DEADZONE 512
#define JOYSTICK_DEBOUNCE_MS 150

#define ENCODER_DEBOUNCE_MS 5
#define KNOCK_DEBOUNCE_MS 250
#define KNOCK_IGNORE_WINDOW_MS 50
#define KNOCK_TIMEOUT_MS 3000

#define AUTH_TIMEOUT_MS 30000
#define LOCKOUT_DURATION_MS 10000
#define MAX_FAILED_ATTEMPTS 3

// ============================================================================
// LCD SETUP
// ============================================================================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ============================================================================
// ENUMS & STRUCTURES
// ============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_JOYSTICK_AUTH,
  STATE_ENCODER_AUTH,
  STATE_KNOCK_AUTH,
  STATE_UNLOCKED,
  STATE_LOCKED_OUT
};

enum JoystickDirection {
  DIR_CENTER,
  DIR_UP,
  DIR_DOWN,
  DIR_LEFT,
  DIR_RIGHT
};

enum CalibrationMode {
  CAL_NONE,
  CAL_MENU,
  CAL_JOYSTICK,
  CAL_ENCODER,
  CAL_KNOCK
};

struct KnockEvent {
  unsigned long timestamp;
};

struct Credentials {
  std::vector<JoystickDirection> joystickSequence;
  std::vector<int> encoderSequence;
  std::vector<unsigned long> knockPattern;
  bool joystickSet;
  bool encoderSet;
  bool knockSet;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
SystemState currentState = STATE_IDLE;
CalibrationMode calibrationMode = CAL_NONE;
int calibrationMenuIndex = 0;
const char* calibrationMenuItems[] = {"Joystick", "Encoder", "Knock Pattern"};
const int calibrationMenuSize = 3;

unsigned long stateStartTime = 0;
unsigned long lastStateActivity = 0;
uint8_t failedAttempts = 0;
unsigned long lockoutStartTime = 0;

Credentials masterCredential;
std::vector<JoystickDirection> joystickAttempt;
std::vector<int> encoderAttempt;
std::vector<unsigned long> knockTimestamps;

// ============================================================================
// LCD HELPER FUNCTIONS
// ============================================================================
void lcdClear() {
  lcd.clear();
}

void lcdPrint(int col, int row, const char* text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdPrintCentered(int row, const char* text) {
  int len = strlen(text);
  int col = (16 - len) / 2;
  if (col < 0) col = 0;
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdShowStatus(const char* line1, const char* line2 = "") {
  lcdClear();
  lcdPrintCentered(0, line1);
  if (strlen(line2) > 0) {
    lcdPrintCentered(1, line2);
  }
}

void lcdShowProgress(const char* title, int current, int total) {
  lcdClear();
  lcdPrintCentered(0, title);
  
  char progress[17];
  snprintf(progress, sizeof(progress), "%d / %d", current, total);
  lcdPrintCentered(1, progress);
}

// ============================================================================
// JOYSTICK CLASS
// ============================================================================
class Joystick {
private:
  int vrxPin, vryPin, swPin;
  unsigned long lastDirectionChange;
  JoystickDirection lastDirection;
  bool buttonPressed;
  unsigned long lastButtonPress;

public:
  Joystick(int vx, int vy, int sw) : vrxPin(vx), vryPin(vy), swPin(sw) {
    pinMode(swPin, INPUT_PULLUP);
    lastDirectionChange = 0;
    lastDirection = DIR_CENTER;
    buttonPressed = false;
    lastButtonPress = 0;
  }

  void begin() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
  }

  int readX() {
    return analogRead(vrxPin);
  }

  int readY() {
    return analogRead(vryPin);
  }

  int getMappedX() {
    int raw = analogRead(vrxPin);
    return map(raw, 0, 4095, -10, 10);
  }

  int getMappedY() {
    int raw = analogRead(vryPin);
    return map(raw, 0, 4095, -10, 10);
  }

  JoystickDirection readDirection() {
    int x = analogRead(vrxPin);
    int y = analogRead(vryPin);
    
    JoystickDirection newDir = DIR_CENTER;
    
    int mappedX = map(x, 0, 4095, -10, 10);
    int mappedY = map(y, 0, 4095, -10, 10);
    
    if (mappedY < -3) {
      newDir = DIR_UP;
    } else if (mappedY > 3) {
      newDir = DIR_DOWN;
    } else if (mappedX < -3) {
      newDir = DIR_LEFT;
    } else if (mappedX > 3) {
      newDir = DIR_RIGHT;
    }
    
    if (newDir != lastDirection && newDir != DIR_CENTER) {
      if (millis() - lastDirectionChange > JOYSTICK_DEBOUNCE_MS) {
        lastDirection = newDir;
        lastDirectionChange = millis();
        return newDir;
      }
    }
    
    if (newDir == DIR_CENTER && lastDirection != DIR_CENTER) {
      lastDirection = DIR_CENTER;
    }
    
    return DIR_CENTER;
  }

  bool isButtonPressed() {
    bool current = (digitalRead(swPin) == LOW);
    if (current && !buttonPressed && (millis() - lastButtonPress > 200)) {
      buttonPressed = true;
      lastButtonPress = millis();
      return true;
    }
    if (!current) buttonPressed = false;
    return false;
  }
};

// ============================================================================
// ROTARY ENCODER CLASS (Polling-based, matches working code)
// ============================================================================
class RotaryEncoder {
private:
  int clkPin, dtPin, swPin;
  int position;
  int lastStateCLK;
  bool lastStateSW;
  unsigned long lastButtonPress;

public:
  RotaryEncoder(int clk, int dt, int sw) : clkPin(clk), dtPin(dt), swPin(sw) {
    position = 0;
    lastStateCLK = HIGH;
    lastStateSW = HIGH;
    lastButtonPress = 0;
  }

  void begin() {
    pinMode(clkPin, INPUT);
    pinMode(dtPin, INPUT);
    pinMode(swPin, INPUT_PULLUP);
    lastStateCLK = digitalRead(clkPin);
    lastStateSW = digitalRead(swPin);
  }

  // Call this frequently in loop() - EXACTLY like working code
  void update() {
    int currentStateCLK = digitalRead(clkPin);
    
    // Check for rotation
    if (currentStateCLK != lastStateCLK) {
      if (digitalRead(dtPin) != currentStateCLK) {
        position++;  // Clockwise
      } else {
        position--;  // Counter-clockwise
      }
    }
    lastStateCLK = currentStateCLK;
  }

  int getPosition() { 
    return position; 
  }
  
  void resetPosition() { 
    position = 0; 
  }

  bool isButtonPressed() {
    bool currentStateSW = (digitalRead(swPin) == LOW);
    
    // Detect falling edge (HIGH to LOW transition)
    if (currentStateSW && !lastStateSW) {
      // Debounce
      if (millis() - lastButtonPress > 200) {
        lastStateSW = currentStateSW;
        lastButtonPress = millis();
        return true;
      }
    }
    
    lastStateSW = currentStateSW;
    return false;
  }
};

// ============================================================================
// KNOCK SENSOR CLASS
// ============================================================================
class KnockSensor {
private:
  int doPin;
  unsigned long lastKnockTime;
  unsigned long lastTriggerTime;
  std::vector<KnockEvent> knockBuffer;
  int ignoredTriggers;

public:
  KnockSensor(int dPin) : doPin(dPin) {
    lastKnockTime = 0;
    lastTriggerTime = 0;
    ignoredTriggers = 0;
  }

  void begin() {
    pinMode(doPin, INPUT);
  }

  bool detectKnock() {
    bool triggered = digitalRead(doPin) == HIGH;
    unsigned long now = millis();
    
    if (triggered) {
      if (now - lastKnockTime < KNOCK_IGNORE_WINDOW_MS) {
        if (now - lastTriggerTime > 10) {
          ignoredTriggers++;
          Serial.print("  [ignored trigger #");
          Serial.print(ignoredTriggers);
          Serial.println("]");
        }
        lastTriggerTime = now;
        return false;
      }
      
      if (now - lastKnockTime < KNOCK_DEBOUNCE_MS) {
        lastTriggerTime = now;
        return false;
      }
      
      lastKnockTime = now;
      lastTriggerTime = now;
      ignoredTriggers = 0;
      
      knockBuffer.push_back({now});
      
      return true;
    }
    return false;
  }
  
  void cleanOldKnocks() {
    unsigned long now = millis();
    while (!knockBuffer.empty() && 
           (now - knockBuffer[0].timestamp > KNOCK_TIMEOUT_MS)) {
      knockBuffer.erase(knockBuffer.begin());
    }
  }

  std::vector<unsigned long> getKnockIntervals() {
    std::vector<unsigned long> intervals;
    for (size_t i = 1; i < knockBuffer.size(); i++) {
      intervals.push_back(knockBuffer[i].timestamp - knockBuffer[i-1].timestamp);
    }
    return intervals;
  }

  void clearKnockBuffer() {
    knockBuffer.clear();
    ignoredTriggers = 0;
  }

  int getKnockCount() {
    return knockBuffer.size();
  }
};

// ============================================================================
// GLOBAL SENSOR INSTANCES
// ============================================================================
Joystick joystick(JOY_VRX_PIN, JOY_VRY_PIN, JOY_SW_PIN);
RotaryEncoder encoder(ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);
KnockSensor knockSensor(KNOCK_PIN);

// ============================================================================
// CALIBRATION MENU
// ============================================================================
void enterCalibrationMenu() {
  calibrationMode = CAL_MENU;
  calibrationMenuIndex = 0;
  encoder.resetPosition();
  
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║   CALIBRATION MENU                    ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println("Rotate encoder to select, press to confirm");
  Serial.println("Current encoder position: 0");
  
  lcdClear();
  lcdPrint(0, 0, "> Joystick");
  lcdPrint(2, 1, "Encoder");
}

void handleCalibrationMenu() {
  static int lastMenuIndex = -1;
  static int lastEncoderPos = 0;
  
  // Update encoder state
  encoder.update();
  
  // Check if encoder position changed
  int currentPos = encoder.getPosition();
  if (currentPos != lastEncoderPos) {
    lastEncoderPos = currentPos;
    calibrationMenuIndex = ((currentPos % calibrationMenuSize) + calibrationMenuSize) % calibrationMenuSize;
    
    Serial.print("Encoder position: ");
    Serial.print(currentPos);
    Serial.print(" -> Menu index: ");
    Serial.println(calibrationMenuIndex);
  }
  
  // Redraw menu if selection changed
  if (calibrationMenuIndex != lastMenuIndex) {
    lastMenuIndex = calibrationMenuIndex;
    
    // Display menu
    lcdClear();
    
    // Show current item on line 0 with arrow
    lcdPrint(0, 0, ">");
    lcdPrint(2, 0, calibrationMenuItems[calibrationMenuIndex]);
    
    // Show next item on line 1 (if exists)
    int nextIdx = (calibrationMenuIndex + 1) % calibrationMenuSize;
    lcdPrint(0, 1, " ");
    lcdPrint(2, 1, calibrationMenuItems[nextIdx]);
    
    Serial.print("Menu: ");
    Serial.println(calibrationMenuItems[calibrationMenuIndex]);
  }
  
  // Select menu item
  if (encoder.isButtonPressed()) {
    Serial.print("Selected: ");
    Serial.println(calibrationMenuItems[calibrationMenuIndex]);
    
    switch(calibrationMenuIndex) {
      case 0:
        calibrationMode = CAL_JOYSTICK;
        joystickAttempt.clear();
        lcdShowStatus("JOYSTICK CAL", "Move & press");
        Serial.println("Move joystick in sequence, press button when done");
        break;
      case 1:
        calibrationMode = CAL_ENCODER;
        encoderAttempt.clear();
        encoder.resetPosition();
        
        Serial.println("\n╔═══════════════════════════════════════╗");
        Serial.println("║   ENCODER CALIBRATION                 ║");
        Serial.println("╚═══════════════════════════════════════╝");
        Serial.println("1. Rotate encoder to desired position");
        Serial.println("2. Press ENCODER button to record it");
        Serial.println("3. Repeat for each position (min 3)");
        Serial.println("4. Press JOYSTICK button when done\n");
        
        lcdShowStatus("ENCODER CAL", "Rotate & press");
        
        // Clear button states by reading them
        joystick.isButtonPressed(); // Consume any pending press
        encoder.isButtonPressed();  // Consume any pending press
        
        delay(1000); // Give time for all buttons to be released
        break;
      case 2:
        calibrationMode = CAL_KNOCK;
        knockSensor.clearKnockBuffer();
        lcdShowStatus("KNOCK CAL", "Knock pattern...");
        Serial.println("Knock your pattern, wait 3 seconds");
        break;
    }
  }
}

// ============================================================================
// JOYSTICK CALIBRATION
// ============================================================================
const char* directionToString(JoystickDirection dir) {
  switch(dir) {
    case DIR_UP: return "UP";
    case DIR_DOWN: return "DOWN";
    case DIR_LEFT: return "LEFT";
    case DIR_RIGHT: return "RIGHT";
    default: return "CENTER";
  }
}

void handleJoystickCalibration() {
  JoystickDirection dir = joystick.readDirection();
  
  if (dir != DIR_CENTER) {
    joystickAttempt.push_back(dir);
    
    Serial.print("✓ Direction ");
    Serial.print(joystickAttempt.size());
    Serial.print(": ");
    Serial.println(directionToString(dir));
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%d: %s", 
             joystickAttempt.size(),
             directionToString(dir));
    lcdShowStatus("Recording...", buffer);
  }
  
  if (joystick.isButtonPressed()) {
    if (joystickAttempt.size() >= 3) {
      masterCredential.joystickSequence = joystickAttempt;
      masterCredential.joystickSet = true;
      
      Serial.println("\n=== NEW JOYSTICK SEQUENCE SAVED ===");
      Serial.print("Moves: ");
      Serial.println(joystickAttempt.size());
      Serial.print("Sequence: ");
      for (size_t i = 0; i < joystickAttempt.size(); i++) {
        Serial.print(directionToString(joystickAttempt[i]));
        if (i < joystickAttempt.size() - 1) Serial.print(" → ");
      }
      Serial.println("\n");
      
      lcdShowStatus("Saved!", "Back to LOCKED");
      delay(2000);
      
      calibrationMode = CAL_NONE;
      changeState(STATE_IDLE);
    } else {
      Serial.print("ERROR: Only ");
      Serial.print(joystickAttempt.size());
      Serial.println(" moves!");
      Serial.println("Need at least 3 moves");
      
      lcdShowStatus("Too short!", "Need 3+ moves");
      delay(2000);
      joystickAttempt.clear();
      lcdShowStatus("JOYSTICK CAL", "Move & press");
    }
  }
}

// ============================================================================
// ENCODER CALIBRATION
// ============================================================================
void handleEncoderCalibration() {
  static unsigned long calibrationStartTime = 0;
  static int lastDisplayedPos = -999;
  
  if (calibrationStartTime == 0) {
    calibrationStartTime = millis();
    Serial.println(">>> ENCODER CALIBRATION ACTIVE <<<");
    Serial.println("Waiting for input...");
  }
  
  encoder.update();
  
  int currentPos = encoder.getPosition();
  
  // Show current position when it changes
  if (currentPos != lastDisplayedPos) {
    lastDisplayedPos = currentPos;
    Serial.print("Encoder position: ");
    Serial.println(currentPos);
    
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "Saved: %d", encoderAttempt.size());
    snprintf(line2, sizeof(line2), "Pos: %d", currentPos);
    
    lcdClear();
    lcdPrintCentered(0, line1);
    lcdPrintCentered(1, line2);
  }
  
  // Press encoder button to record current position
  if (encoder.isButtonPressed()) {
    // Ignore presses in first 1500ms (debounce from menu selection)
    if (millis() - calibrationStartTime < 1500) {
      Serial.println("  [ENCODER BUTTON ignored - too soon after entering calibration]");
      return;
    }
    
    encoderAttempt.push_back(currentPos);
    
    Serial.print("✓ RECORDED Position ");
    Serial.print(encoderAttempt.size());
    Serial.print(": ");
    Serial.println(currentPos);
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "Saved: %d", currentPos);
    lcdShowStatus("Position OK!", buffer);
    delay(800);
    
    // Reset position for next entry
    encoder.resetPosition();
    lastDisplayedPos = -999;
  }
  
  // Press joystick button to finish and save
  if (joystick.isButtonPressed()) {
    // Need at least 2000ms to have passed
    if (millis() - calibrationStartTime < 2000) {
      Serial.println("  [JOYSTICK BUTTON ignored - too soon, wait and record positions first]");
      return;
    }
    
    Serial.println(">>> JOYSTICK BUTTON PRESSED - Attempting to save <<<");
    
    if (encoderAttempt.size() >= 3) {
      masterCredential.encoderSequence = encoderAttempt;
      masterCredential.encoderSet = true;
      
      Serial.println("\n=== NEW ENCODER SEQUENCE SAVED ===");
      Serial.print("Positions: ");
      for (size_t i = 0; i < encoderAttempt.size(); i++) {
        Serial.print(encoderAttempt[i]);
        if (i < encoderAttempt.size() - 1) Serial.print(" → ");
      }
      Serial.println("\n");
      
      lcdShowStatus("Saved!", "Back to LOCKED");
      delay(2000);
      
      calibrationMode = CAL_NONE;
      calibrationStartTime = 0;
      encoder.resetPosition();
      changeState(STATE_IDLE);
    } else {
      Serial.print("ERROR: Only ");
      Serial.print(encoderAttempt.size());
      Serial.println(" positions recorded!");
      Serial.println("Need at least 3 positions. Keep recording.");
      
      lcdShowStatus("Too short!", "Need 3+ pos");
      delay(1500);
    }
  }
}

// ============================================================================
// KNOCK CALIBRATION
// ============================================================================
void handleKnockCalibration() {
  static unsigned long lastKnockTime = 0;
  static bool recording = false;
  
  if (knockSensor.detectKnock()) {
    lastKnockTime = millis();
    recording = true;
    
    int count = knockSensor.getKnockCount();
    Serial.print("✓ Knock ");
    Serial.print(count);
    Serial.print(" at ");
    Serial.print(millis());
    Serial.println("ms");
    
    lcdShowProgress("Recording...", count, 99);
  }
  
  if (recording && (millis() - lastKnockTime > 3000)) {
    std::vector<unsigned long> pattern = knockSensor.getKnockIntervals();
    
    Serial.println("\n=== CALIBRATION COMPLETE ===");
    Serial.print("Total knocks: ");
    Serial.println(knockSensor.getKnockCount());
    Serial.print("Intervals: ");
    Serial.println(pattern.size());
    
    if (pattern.size() >= 2) {
      masterCredential.knockPattern = pattern;
      masterCredential.knockSet = true;
      
      Serial.println("\n=== NEW PATTERN SAVED ===");
      Serial.print("Total knocks: ");
      Serial.println(pattern.size() + 1);
      Serial.print("Intervals: ");
      for (size_t i = 0; i < pattern.size(); i++) {
        Serial.print(pattern[i]);
        Serial.print("ms");
        if (i < pattern.size() - 1) Serial.print(", ");
      }
      Serial.println("\n");
      
      lcdShowStatus("Saved!", "Back to LOCKED");
      delay(2000);
      
      calibrationMode = CAL_NONE;
      encoder.resetPosition();
      changeState(STATE_IDLE);
    } else {
      Serial.print("ERROR: Only ");
      Serial.print(knockSensor.getKnockCount());
      Serial.println(" knocks!");
      Serial.println("Need at least 3 knocks");
      
      lcdShowStatus("Too short!", "Try again...");
      delay(2000);
      knockSensor.clearKnockBuffer();
      recording = false;
      lcdShowStatus("KNOCK CAL", "Knock pattern...");
    }
  }
}

// ============================================================================
// DTW IMPLEMENTATION
// ============================================================================
float calculateDTW(const std::vector<unsigned long>& seq1, 
                   const std::vector<unsigned long>& seq2) {
  int n = seq1.size();
  int m = seq2.size();
  
  if (n == 0 || m == 0) return INFINITY;
  
  float** dtw = new float*[n + 1];
  for (int i = 0; i <= n; i++) {
    dtw[i] = new float[m + 1];
  }
  
  for (int i = 0; i <= n; i++) {
    for (int j = 0; j <= m; j++) {
      dtw[i][j] = INFINITY;
    }
  }
  dtw[0][0] = 0;
  
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= m; j++) {
      float cost = abs((int)seq1[i-1] - (int)seq2[j-1]);
      float minPrev = min(min(dtw[i-1][j], dtw[i][j-1]), dtw[i-1][j-1]);
      dtw[i][j] = cost + minPrev;
    }
  }
  
  float result = dtw[n][m];
  result = result / (n + m);
  
  for (int i = 0; i <= n; i++) {
    delete[] dtw[i];
  }
  delete[] dtw;
  
  return result;
}

std::vector<unsigned long> normalizeSequence(const std::vector<unsigned long>& seq) {
  if (seq.empty()) return seq;
  
  unsigned long sum = 0;
  for (unsigned long val : seq) {
    sum += val;
  }
  float avg = (float)sum / seq.size();
  
  std::vector<unsigned long> normalized;
  for (unsigned long val : seq) {
    normalized.push_back((unsigned long)((val / avg) * 100));
  }
  
  return normalized;
}

bool verifyKnockPattern(const std::vector<unsigned long>& attempt,
                       const std::vector<unsigned long>& master,
                       float threshold = 30.0) {
  
  if (attempt.size() != master.size()) {
    Serial.print("Wrong count. Expected: ");
    Serial.print(master.size() + 1);
    Serial.print(", Got: ");
    Serial.println(attempt.size() + 1);
    return false;
  }
  
  std::vector<unsigned long> normAttempt = normalizeSequence(attempt);
  std::vector<unsigned long> normMaster = normalizeSequence(master);
  
  float distance = calculateDTW(normAttempt, normMaster);
  
  Serial.println("\n=== DTW Analysis ===");
  Serial.print("DTW Distance: ");
  Serial.print(distance);
  Serial.print(" (threshold: ");
  Serial.print(threshold);
  Serial.println(")");
  
  if (distance <= threshold) {
    Serial.println("✓ PATTERN MATCH!");
    return true;
  } else {
    Serial.println("✗ Pattern mismatch");
    return false;
  }
}

// ============================================================================
// JOYSTICK AUTHENTICATION
// ============================================================================
void handleJoystickAuth() {
  static unsigned long lastUpdate = 0;
  
  JoystickDirection dir = joystick.readDirection();
  
  if (dir != DIR_CENTER) {
    joystickAttempt.push_back(dir);
    lastStateActivity = millis();
    
    Serial.print("Joystick: ");
    Serial.print(directionToString(dir));
    Serial.print(" (");
    Serial.print(joystickAttempt.size());
    Serial.print("/");
    Serial.print(masterCredential.joystickSequence.size());
    Serial.println(")");
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%d/%d %s", 
             joystickAttempt.size(), 
             masterCredential.joystickSequence.size(),
             directionToString(dir));
    lcdShowStatus("Joystick Path", buffer);
    
    if (joystickAttempt.size() >= masterCredential.joystickSequence.size()) {
      delay(500);
      
      bool match = true;
      for (size_t i = 0; i < masterCredential.joystickSequence.size(); i++) {
        if (joystickAttempt[i] != masterCredential.joystickSequence[i]) {
          match = false;
          Serial.print("Mismatch at position ");
          Serial.print(i);
          Serial.print(": Expected ");
          Serial.print(directionToString(masterCredential.joystickSequence[i]));
          Serial.print(", Got ");
          Serial.println(directionToString(joystickAttempt[i]));
          break;
        }
      }
      
      if (match) {
        Serial.println("✓ Joystick sequence correct!");
        lcdShowStatus("Joystick OK!", "Next: Encoder");
        delay(1500);
        changeState(STATE_ENCODER_AUTH);
      } else {
        Serial.println("✗ Joystick sequence wrong!");
        lcdShowStatus("Wrong Path!", "Try again");
        delay(2000);
        handleFailedAuth();
      }
    }
  }
  
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    if (joystickAttempt.size() == 0) {
      char buffer[17];
      snprintf(buffer, sizeof(buffer), "Need %d moves", masterCredential.joystickSequence.size());
      lcdShowStatus("Joystick Path", buffer);
    }
  }
}

// ============================================================================
// ENCODER AUTHENTICATION
// ============================================================================
void handleEncoderAuth() {
  static unsigned long lastUpdate = 0;
  static int tolerance = 3;  // Allow ±3 position error
  
  encoder.update();
  
  int currentPos = encoder.getPosition();
  
  // Show current position and instructions
  if (millis() - lastUpdate > 300) {
    lastUpdate = millis();
    
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "Enc %d/%d", 
             encoderAttempt.size() + 1,
             masterCredential.encoderSequence.size());
    snprintf(line2, sizeof(line2), "Pos: %d", currentPos);
    
    lcdClear();
    lcdPrintCentered(0, line1);
    lcdPrintCentered(1, line2);
  }
  
  // Press encoder button to confirm position
  if (encoder.isButtonPressed()) {
    encoderAttempt.push_back(currentPos);
    lastStateActivity = millis();
    
    Serial.print("Encoder: Position ");
    Serial.print(currentPos);
    Serial.print(" recorded (");
    Serial.print(encoderAttempt.size());
    Serial.print("/");
    Serial.print(masterCredential.encoderSequence.size());
    Serial.println(")");
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "Set: %d", currentPos);
    lcdShowStatus("Position OK", buffer);
    delay(500);
    
    // Reset for next position
    encoder.resetPosition();
    lastUpdate = 0;
    
    // Check if sequence is complete
    if (encoderAttempt.size() >= masterCredential.encoderSequence.size()) {
      delay(300);
      
      // Verify the sequence with tolerance
      bool match = true;
      Serial.println("\n=== Encoder Verification ===");
      for (size_t i = 0; i < masterCredential.encoderSequence.size(); i++) {
        int expected = masterCredential.encoderSequence[i];
        int actual = encoderAttempt[i];
        int diff = abs(expected - actual);
        
        Serial.print("Position ");
        Serial.print(i + 1);
        Serial.print(": Expected ");
        Serial.print(expected);
        Serial.print(", Got ");
        Serial.print(actual);
        Serial.print(", Diff ");
        Serial.println(diff);
        
        if (diff > tolerance) {
          match = false;
          break;
        }
      }
      
      if (match) {
        Serial.println("✓ Encoder sequence correct!");
        lcdShowStatus("Encoder OK!", "Next: Knock");
        delay(1500);
        changeState(STATE_KNOCK_AUTH);
      } else {
        Serial.println("✗ Encoder sequence wrong!");
        lcdShowStatus("Wrong Combo!", "Try again");
        delay(2000);
        handleFailedAuth();
      }
    }
  }
}

// ============================================================================
// KNOCK AUTHENTICATION
// ============================================================================
void handleKnockAuth() {
  static unsigned long lastKnockCheck = 0;
  static bool waitingForCompletion = false;
  static unsigned long lastKnockDetected = 0;
  
  if (!waitingForCompletion && knockSensor.getKnockCount() == 0) {
    knockSensor.cleanOldKnocks();
  }
  
  if (knockSensor.detectKnock()) {
    lastKnockDetected = millis();
    lastStateActivity = millis();
    waitingForCompletion = true;
    
    int count = knockSensor.getKnockCount();
    Serial.print("Knock ");
    Serial.println(count);
    
    lcdShowProgress("Knocking...", count, masterCredential.knockPattern.size() + 1);
  }
  
  if (waitingForCompletion && (millis() - lastKnockDetected > KNOCK_TIMEOUT_MS)) {
    waitingForCompletion = false;
    
    std::vector<unsigned long> intervals = knockSensor.getKnockIntervals();
    
    Serial.println("\n=== AUTHENTICATION ATTEMPT ===");
    Serial.print("Knocks detected: ");
    Serial.println(knockSensor.getKnockCount());
    Serial.print("Intervals calculated: ");
    Serial.println(intervals.size());
    
    if (intervals.size() > 0) {
      lcdShowStatus("Verifying...", "");
      delay(500);
      
      if (verifyKnockPattern(intervals, masterCredential.knockPattern)) {
        Serial.println("\n*** AUTHENTICATION SUCCESS ***\n");
        lcdShowStatus("ACCESS", "GRANTED!");
        delay(2000);
        changeState(STATE_UNLOCKED);
      } else {
        Serial.println("\n*** AUTHENTICATION FAILED ***\n");
        lcdShowStatus("ACCESS", "DENIED!");
        delay(2000);
        handleFailedAuth();
      }
    } else {
      Serial.println("ERROR: No intervals!");
      lcdShowStatus("No pattern!", "Try again");
      delay(2000);
      handleFailedAuth();
    }
  }
  
  if (millis() - lastKnockCheck > 2000) {
    lastKnockCheck = millis();
    if (knockSensor.getKnockCount() == 0) {
      char buffer[17];
      snprintf(buffer, sizeof(buffer), "%d knocks", masterCredential.knockPattern.size() + 1);
      lcdShowStatus("Knock Pattern", buffer);
    }
  }
}

// ============================================================================
// STATE MACHINE
// ============================================================================
void changeState(SystemState newState) {
  currentState = newState;
  stateStartTime = millis();
  lastStateActivity = millis();
  
  Serial.print(">>> STATE CHANGED TO: ");
  Serial.print(newState);
  Serial.println(" <<<");
  
  if (newState == STATE_JOYSTICK_AUTH) {
    joystickAttempt.clear();
    Serial.println("\n=== JOYSTICK AUTHENTICATION ===");
    Serial.print("Required moves: ");
    Serial.println(masterCredential.joystickSequence.size());
    Serial.print("Sequence: ");
    for (size_t i = 0; i < masterCredential.joystickSequence.size(); i++) {
      Serial.print(directionToString(masterCredential.joystickSequence[i]));
      if (i < masterCredential.joystickSequence.size() - 1) Serial.print(" → ");
    }
    Serial.println();
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "Need %d moves", masterCredential.joystickSequence.size());
    lcdShowStatus("Joystick", buffer);
    
  } else if (newState == STATE_ENCODER_AUTH) {
    encoderAttempt.clear();
    encoder.resetPosition();
    
    Serial.println("\n=== ENCODER AUTHENTICATION ===");
    Serial.print("Required positions: ");
    Serial.println(masterCredential.encoderSequence.size());
    Serial.print("Positions: ");
    for (size_t i = 0; i < masterCredential.encoderSequence.size(); i++) {
      Serial.print(masterCredential.encoderSequence[i]);
      if (i < masterCredential.encoderSequence.size() - 1) Serial.print(" → ");
    }
    Serial.println();
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "Need %d pos", masterCredential.encoderSequence.size());
    lcdShowStatus("Encoder Combo", buffer);
    
  } else if (newState == STATE_KNOCK_AUTH) {
    knockTimestamps.clear();
    knockSensor.clearKnockBuffer();
    
    Serial.println("\n=== KNOCK AUTHENTICATION ===");
    Serial.print("Required knocks: ");
    Serial.println(masterCredential.knockPattern.size() + 1);
    
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%d knocks", masterCredential.knockPattern.size() + 1);
    lcdShowStatus("Knock Pattern", buffer);
    
  } else if (newState == STATE_IDLE) {
    lcdShowStatus("LOCKED", "Press joy btn");
    
    Serial.println("\n════════════════════════════════════════");
    Serial.println("SYSTEM LOCKED");
    Serial.println("════════════════════════════════════════");
    Serial.print("Joystick set: ");
    Serial.print(masterCredential.joystickSet ? "YES (" : "NO (");
    Serial.print(masterCredential.joystickSequence.size());
    Serial.println(" moves)");
    Serial.print("Encoder set: ");
    Serial.print(masterCredential.encoderSet ? "YES (" : "NO (");
    Serial.print(masterCredential.encoderSequence.size());
    Serial.println(" pos)");
    Serial.print("Knock set: ");
    Serial.print(masterCredential.knockSet ? "YES (" : "NO (");
    Serial.print(masterCredential.knockPattern.size() + 1);
    Serial.println(" knocks)");
    Serial.println("════════════════════════════════════════\n");
    
  } else if (newState == STATE_UNLOCKED) {
    lcdShowStatus("UNLOCKED", "Press enc btn");
  } else if (newState == STATE_LOCKED_OUT) {
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%d attempts!", MAX_FAILED_ATTEMPTS);
    lcdShowStatus("LOCKED OUT", buffer);
  }
}

bool checkTimeout() {
  if (currentState == STATE_IDLE || currentState == STATE_UNLOCKED) {
    return false;
  }
  
  if (millis() - lastStateActivity > AUTH_TIMEOUT_MS) {
    Serial.println("Timeout!");
    failedAttempts++;
    
    if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
      lockoutStartTime = millis();
      changeState(STATE_LOCKED_OUT);
    } else {
      changeState(STATE_IDLE);
    }
    return true;
  }
  return false;
}

void handleFailedAuth() {
  failedAttempts++;
  Serial.print("Failed: ");
  Serial.println(failedAttempts);
  
  if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
    lockoutStartTime = millis();
    changeState(STATE_LOCKED_OUT);
  } else {
    changeState(STATE_IDLE);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(LCD_SDA, LCD_SCL);
  
  lcd.init();
  lcd.backlight();
  
  joystick.begin();
  encoder.begin();
  knockSensor.begin();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Update encoder in every loop iteration (polling mode)
  encoder.update();
  
  // DEBUG: Show current mode
  static CalibrationMode lastMode = CAL_NONE;
  static SystemState lastState = STATE_IDLE;
  
  if (calibrationMode != lastMode) {
    Serial.print(">>> CALIBRATION MODE CHANGED: ");
    Serial.print(lastMode);
    Serial.print(" -> ");
    Serial.println(calibrationMode);
    lastMode = calibrationMode;
  }
  
  if (currentState != lastState) {
    Serial.print(">>> SYSTEM STATE CHANGED: ");
    Serial.print(lastState);
    Serial.print(" -> ");
    Serial.println(currentState);
    lastState = currentState;
  }
  
  // Handle calibration modes
  if (calibrationMode == CAL_MENU) {
    handleCalibrationMenu();
    delay(10);
    return;
  } else if (calibrationMode == CAL_JOYSTICK) {
    handleJoystickCalibration();
    delay(10);
    return;
  } else if (calibrationMode == CAL_ENCODER) {
    handleEncoderCalibration();
    delay(10);
    return;
  } else if (calibrationMode == CAL_KNOCK) {
    handleKnockCalibration();
    delay(10);
    return;
  }
  
  // Calibration menu trigger (hold encoder button for 3 seconds)
  static unsigned long encButtonHoldStart = 0;
  if (digitalRead(ENC_SW_PIN) == LOW) {
    if (encButtonHoldStart == 0) {
      encButtonHoldStart = millis();
    } else if (millis() - encButtonHoldStart > 3000 && currentState == STATE_IDLE) {
      enterCalibrationMenu();
      encButtonHoldStart = 0;
      return;
    }
  } else {
    encButtonHoldStart = 0;
  }
  
  // Lockout countdown
  if (currentState == STATE_LOCKED_OUT) {
    unsigned long remaining = (LOCKOUT_DURATION_MS - (millis() - lockoutStartTime)) / 1000;
    if (remaining > 0) {
      char buffer[17];
      snprintf(buffer, sizeof(buffer), "Wait %lus", remaining);
      lcdShowStatus("LOCKED OUT", buffer);
    } else {
      failedAttempts = 0;
      changeState(STATE_IDLE);
      Serial.println("Lockout expired");
    }
    delay(1000);
    return;
  }
  
  checkTimeout();
  
  switch (currentState) {
    case STATE_IDLE:
      // ONLY respond to button press - stay in IDLE otherwise
      if (joystick.isButtonPressed()) {
        Serial.println("\n>>> AUTHENTICATION STARTED <<<\n");
        changeState(STATE_JOYSTICK_AUTH);
      }
      // Don't do anything else - just wait
      break;
      
    case STATE_JOYSTICK_AUTH:
      handleJoystickAuth();
      break;
      
    case STATE_ENCODER_AUTH:
      handleEncoderAuth();
      break;
      
    case STATE_KNOCK_AUTH:
      handleKnockAuth();
      break;
      
    case STATE_UNLOCKED:
      if (encoder.isButtonPressed()) {
        failedAttempts = 0;
        changeState(STATE_IDLE);
        Serial.println("Locked");
      }
      break;
  }
  
  delay(10);
}