// Mega Pin Layout
// ===============
// Analogs (8 channels, 0-255 mapped, center 128 for joysticks):
//   A0: Left joy Y (forward/back)
//   A1: Left joy X (left/right)
//   A2: Left joy Z (twist - yaw whole body and legs)
//   A3: Right joy Y (up/down - pitch)
//   A4: Right joy X (left/right - roll)
//   A5: Right joy Z (twist - center body control only, legs stay in place)
//   A6: Body height pot
//   A7: Gait speed pot

// Toggle switches (6, INPUT_PULLUP):
//   D2: HexOn
//   D3: Pi/Controller switch
//   D4: Mode Select (Translate/Walk)
//   D5: Balance Mode
//   D6: Double Travel
//   D7: Double Leg Lift Height

// Momentary buttons (5, LOW = pressed = 255, same wiring):
//   D8:  Toggle between Walk method 1 & Walk method 2
//   D9:  S-Leg Mode/Enter Menu (button press shows menu, select leg 1-6 with rotary encoder)
//   D10: S-Leg Hold/Freeze position
//   D11: Gait preset (button press shows menu, select gait preset with rotary encoder)
//   D12: GPPlayer Mode (button press shows menu, select sequence with rotary encoder)

// Rotary encoder:
//   D20: DT (SCL, interrupt-capable)
//   D21: CLK (SDA, interrupt-capable)
//   D24: SW (push button for select/confirm)

// TFT SPI 240x320 LCD (ILI9341 or similar):
//   TFT VCC     → 5V
//   TFT LED     → 5V
//   TFT GND     → GND

//   TFT DC      → D47
//   TFT RESET   → D48
//   SD CS       → D49
//   TFT/SD MISO → D50
//   TFT/SD MOSI → D51
//   TFT/SD SCK  → D52
//   TFT CS      → D53
//=====================

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include <Rotary.h>
#include <SD.h>
#include <math.h>  // For cos, sin, PI in joystick arcs

const int analogPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // 8 analogs
const int switchPins[6] = {2, 3, 4, 5, 6, 7};  // toggle switches
const int buttonPins[6] = {8, 9, 10, 11, 12, 24};  // momentary buttons + encoder SW
const int encoderDT = 20;
const int encoderCLK = 21;

const int TFT_DC = 47;
const int TFT_RST = 48;
const int SD_CS = 49;
const int TFT_CS = 53;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

Rotary encoder = Rotary(encoderCLK, encoderDT);
volatile int deltaAccum = 0;

// Menu state
enum MenuState { NO_MENU, SLEG_MENU, GAIT_MENU, GP_MENU };
MenuState currentMenu = NO_MENU;
int menuValue = 0;

// Gait names
const char* gaitNames[8] = {"Ripple 6", "Ripple 12", "Quadripple 9", "Tripod 4", "Tripod 6", "Tripod 8", "Wave 12", "Wave 18"};

// SD card sequences
String seqFiles[10];  // Up to 10 CSV files
int numSeqFiles = 0;

// Globals
byte g_fHexOn = 0;
byte PiControllerSwitch = 0;  // 0 = Pi, 255 = Controller
byte ModeSelect = 0;  // 0 = Translate, 255 = Walk
byte BalanceMode = 0;
byte DoubleTravel = 0;
byte DoubleLegLiftHeight = 0;
byte WalkMethod = 0;
byte SLHold = 0;
byte GaitPreset = 0;  // 0-7
byte SelectedLeg = 255;  // 255 = off, 0-5 = selected
byte GPSequence = 0;  // 0 = off, 1-N = sequence index +1

// Joystick/pot values (0-255)
byte leftFwdBack = 128;
byte leftLeftRight = 128;
byte leftTwist = 128;
byte rightUpDown = 128;
byte rightLeftRight = 128;
byte rightTwist = 128;
byte bodyHeight = 0;
byte gaitSpeed = 0;

// Previous values for update optimization
byte prevLeftFwdBack = 128;
byte prevLeftLeftRight = 128;
byte prevLeftTwist = 128;
byte prevRightUpDown = 128;
byte prevRightLeftRight = 128;
byte prevRightTwist = 128;
byte prevBodyHeight = 0;
byte prevGaitSpeed = 0;
byte prevSwitches[6] = {0,0,0,0,0,0};
byte prevWalkMethod = 0;

// Button debouncing/tracking
unsigned long lastButtonTime[6] = {0, 0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;  // ms
const unsigned long longPressDelay = 500;  // ms for long press
bool longPressed[6] = {false, false, false, false, false, false};

// Function to draw outlined text (white with black outline)
void drawOutlinedText(const char* text, int x, int y, uint16_t color, uint16_t outlineColor) {
  tft.setTextColor(outlineColor);
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      tft.setCursor(x + dx, y + dy);
      tft.print(text);
    }
  }
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(text);
}

// Function to center outlined text
void printCenteredOutlined(const char* text, int y, uint16_t color = ILI9341_WHITE, uint16_t outlineColor = ILI9341_BLACK) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (tft.width() - w) / 2;
  drawOutlinedText(text, x, y, color, outlineColor);
}

// Draw top and bottom status bars
void drawStatusBars(byte states[6]) {
  const int barHeight = 25;
  int blockWidth = tft.width() / 3;

  // Top bar at y=0
  tft.fillRect(0, 0, tft.width(), barHeight, ILI9341_GREEN);

  tft.setTextSize(1);

  // Top row: dynamic labels
  byte topIndices[3] = {0, 1, 2};  // HexOn, PiController, ModeSelect
  const char* topLabels[3][2] = {{"Off", "On"}, {"Pi", "Joystick"}, {"Translate", "Walk"}};  // 0: off/0, on/255; 1: Pi/0, Joystick/255; 2: Translate/0, Walk/255

  for (int i = 0; i < 3; i++) {
    int x = i * blockWidth;
    byte idx = topIndices[i];
    byte state = states[idx];
    uint16_t bgColor = (state == 255) ? ILI9341_WHITE : ILI9341_GREEN;
    tft.fillRect(x, 0, blockWidth, barHeight, bgColor);
    const char* label = topLabels[i][state == 255 ? 1 : 0];
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    int tx = x + (blockWidth - w) / 2;
    int ty = (barHeight - h) / 2;
    drawOutlinedText(label, tx, ty, ILI9341_WHITE, ILI9341_BLACK);
    if (i < 2) {
      tft.drawLine(x + blockWidth, 0, x + blockWidth, barHeight, ILI9341_BLACK);
    }
  }

  // Bottom bar at y=240 - barHeight = 215
  tft.fillRect(0, 215, tft.width(), barHeight, ILI9341_GREEN);

  // Bottom row: static labels, color change
  byte bottomIndices[3] = {3, 4, 5};  // BalanceMode, DoubleTravel, DoubleLegLiftHeight
  const char* bottomLabels[3] = {"Balance", "Dbl Travel", "Dbl Height"};

  for (int i = 0; i < 3; i++) {
    int x = i * blockWidth;
    byte idx = bottomIndices[i];
    byte state = states[idx];
    uint16_t bgColor = (state == 255) ? ILI9341_WHITE : ILI9341_GREEN;
    tft.fillRect(x, 215, blockWidth, barHeight, bgColor);
    const char* label = bottomLabels[i];
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    int tx = x + (blockWidth - w) / 2;
    int ty = 215 + (barHeight - h) / 2;
    drawOutlinedText(label, tx, ty, ILI9341_WHITE, ILI9341_BLACK);
    if (i < 2) {
      tft.drawLine(x + blockWidth, 215, x + blockWidth, 215 + barHeight, ILI9341_BLACK);
    }
  }

  tft.setTextSize(2);  // Reset
}

// Draw walk method text
void drawWalkMethod(byte method) {
  tft.fillRect(0, 190, 320, 25, ILI9341_BLACK);  // Clear area above bottom bar
  const char* label = (method == 0) ? "Method 1" : "Method 2";
  printCenteredOutlined(label, 190, ILI9341_WHITE, ILI9341_BLACK);
}

// Draw joystick circle and arc
void drawJoystickBase() {
  // Left
  tft.drawCircle(80, 120, 40, ILI9341_LIGHTGREY);  // 80px diameter
  // Semi-arc above, thin orange
  // Approximate with short lines
  for (int angle = 180; angle <= 360; angle += 5) {  // Top semi
    int x1 = 80 + 30 * cos(angle * PI / 180);
    int y1 = 70 + 30 * sin(angle * PI / 180);  // y=120-50=70 center
    int x2 = 80 + 29 * cos(angle * PI / 180);  // Thin
    int y2 = 70 + 29 * sin(angle * PI / 180);
    tft.drawLine(x1, y1, x2, y2, ILI9341_ORANGE);
  }

  // Right
  tft.drawCircle(240, 120, 40, ILI9341_LIGHTGREY);
  for (int angle = 180; angle <= 360; angle += 5) {
    int x1 = 240 + 30 * cos(angle * PI / 180);
    int y1 = 70 + 30 * sin(angle * PI / 180);
    int x2 = 240 + 29 * cos(angle * PI / 180);
    int y2 = 70 + 29 * sin(angle * PI / 180);
    tft.drawLine(x1, y1, x2, y2, ILI9341_ORANGE);
  }
}

// Update left joystick (erase old, draw new)
void updateLeftJoystick(byte fwdBack, byte leftRight, byte twist, byte prevFwdBack, byte prevLeftRight, byte prevTwist) {
  // Erase old
  tft.fillCircle(80 + map(prevLeftRight, 0, 255, -40, 40), 120 + map(prevFwdBack, 0, 255, 40, -40), 5, ILI9341_BLACK);
  // Draw new
  tft.fillCircle(80 + map(leftRight, 0, 255, -40, 40), 120 + map(fwdBack, 0, 255, 40, -40), 5, ILI9341_WHITE);
  // Twist arc - update if changed
  if (twist != prevTwist) {
    // Erase old arc
    for (int angle = 180; angle <= 360; angle += 5) {
      int r = map(prevTwist, 0, 255, 0, 30);
      int x1 = 80 + r * cos(angle * PI / 180);
      int y1 = 70 + r * sin(angle * PI / 180);
      tft.drawPixel(x1, y1, ILI9341_BLACK);
    }
    // Draw new arc
    for (int angle = 180; angle <= 360; angle += 5) {
      int r = map(twist, 0, 255, 0, 30);
      int x1 = 80 + r * cos(angle * PI / 180);
      int y1 = 70 + r * sin(angle * PI / 180);
      tft.drawPixel(x1, y1, ILI9341_ORANGE);
    }
  }
}

// Update right joystick (similar)
void updateRightJoystick(byte upDown, byte leftRight, byte twist, byte prevUpDown, byte prevLeftRight, byte prevTwist) {
  // Erase old
  tft.fillCircle(240 + map(leftRight, 0, 255, -40, 40), 120 + map(upDown, 0, 255, 40, -40), 5, ILI9341_BLACK);
  // Draw new
  tft.fillCircle(240 + map(leftRight, 0, 255, -40, 40), 120 + map(upDown, 0, 255, 40, -40), 5, ILI9341_WHITE);
  // Twist arc
  if (twist != prevTwist) {
    // Erase old
    for (int angle = 180; angle <= 360; angle += 5) {
      int r = map(prevTwist, 0, 255, 0, 30);
      int x1 = 240 + r * cos(angle * PI / 180);
      int y1 = 70 + r * sin(angle * PI / 180);
      tft.drawPixel(x1, y1, ILI9341_BLACK);
    }
    // Draw new
    for (int angle = 180; angle <= 360; angle += 5) {
      int r = map(twist, 0, 255, 0, 30);
      int x1 = 240 + r * cos(angle * PI / 180);
      int y1 = 70 + r * sin(angle * PI / 180);
      tft.drawPixel(x1, y1, ILI9341_ORANGE);
    }
  }
}

// Draw sliders base
void drawSlidersBase() {
  tft.drawRect(10, 25, 20, 165, ILI9341_LIGHTGREY);  // Left slider track
  tft.drawRect(290, 25, 20, 165, ILI9341_LIGHTGREY); // Right
}

// Update slider (left=true for body, false for gait)
void updateSlider(bool isLeft, byte value, byte prevValue) {
  int y = map(prevValue, 0, 255, 190, 25);  // Invert: 0 at bottom, 255 at top
  tft.fillRect(isLeft ? 10 : 290, y - 5, 20, 10, ILI9341_BLACK); // Erase old
  y = map(value, 0, 255, 190, 25);
  tft.fillRect(isLeft ? 10 : 290, y - 5, 20, 10, ILI9341_WHITE); // Draw new
}

// Draw thick line (for legs)
void drawThickLine(int x1, int y1, int x2, int y2, uint16_t color) {
  tft.drawLine(x1, y1, x2, y2, color);
  tft.drawLine(x1+1, y1, x2+1, y2, color);
  tft.drawLine(x1, y1+1, x2, y2+1, color);
}

// Draw hexapod
void drawHexapod(int selected = -1) {
  tft.fillRect(0, 50, 320, 165, ILI9341_BLACK);  // Clear larger area for hexapod

  // Body: tall dark-gray rounded rect
  int bodyX = 160, bodyY = 110, bodyW = 90, bodyH = 60;  // Larger body, centered higher
  tft.fillRoundRect(bodyX - bodyW/2, bodyY - bodyH/2, bodyW, bodyH, 10, ILI9341_CYAN);
  tft.drawRoundRect(bodyX - bodyW/2, bodyY - bodyH/2, bodyW, bodyH, 10, ILI9341_WHITE);

  
  // Need to reorder this part: 1 RR, 2 RM, 3 RF, 4 LR, 5 LM, 6 LF
  // Legs: 1 RF, 2 RM, 3 RR, 4 LF, 5 LM, 6 LR
  // Each leg: body -> coxa joint -> femur joint -> tibia end -> foot triangle
  // Adjusted positions to spread out more and reduce overlap
  struct LegJoint {
    int cx, cy, fx, fy, tx, ty;  // coxa, femur, tibia ends relative to bodyX,Y
  };
  LegJoint legs[6] = {
    {45, -25, 80, -40, 115, -20},  // RF forward
    {45, 0, 80, 0, 115, 20},       // RM
    {45, 25, 80, 40, 115, 60},     // RR backward
    {-45, -25, -80, -40, -115, -20}, // LF
    {-45, 0, -80, 0, -115, 20},      // LM
    {-45, 25, -80, 40, -115, 60}     // LR
  };

  for (int i = 0; i < 6; i++) {
    uint16_t color = (i + 1 == selected) ? ILI9341_RED : ILI9341_WHITE;
    // Body to coxa
    drawThickLine(bodyX, bodyY + legs[i].cy / 2, bodyX + legs[i].cx, bodyY + legs[i].cy, color);
    // Coxa to femur
    drawThickLine(bodyX + legs[i].cx, bodyY + legs[i].cy, bodyX + legs[i].fx, bodyY + legs[i].fy, color);
    // Femur to tibia
    drawThickLine(bodyX + legs[i].fx, bodyY + legs[i].fy, bodyX + legs[i].tx, bodyY + legs[i].ty, color);

    // Joint dots
    tft.fillCircle(bodyX + legs[i].cx, bodyY + legs[i].cy, 4, ILI9341_YELLOW);
    tft.fillCircle(bodyX + legs[i].fx, bodyY + legs[i].fy, 4, ILI9341_YELLOW);

    // Foot arrow pointing down
    int fx = bodyX + legs[i].tx, fy = bodyY + legs[i].ty;
    tft.fillTriangle(fx - 5, fy, fx + 5, fy, fx, fy + 10, color);

    // Label below arrow, larger size
    tft.setTextSize(2);
    tft.setTextColor(color);
    tft.setCursor(fx - 8, fy + 12);  // Adjusted for size 2
    tft.print(i + 1);
    tft.setTextSize(2);  // Reset if needed
  }
}

// ISR for encoder
void rotaryISR() {
  unsigned char result = encoder.process();
  if (result == DIR_CW) deltaAccum++;
  else if (result == DIR_CCW) deltaAccum--;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Pins
  for (int p : analogPins) pinMode(p, INPUT);
  for (int p : switchPins) pinMode(p, INPUT_PULLUP);
  for (int p : buttonPins) pinMode(p, INPUT_PULLUP);

  // Encoder
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(encoderCLK), rotaryISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderDT), rotaryISR, CHANGE);

  // TFT
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.begin();
  delay(500);  // Added delay for init
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  printCenteredOutlined("Hexapod Controller", 0);
  printCenteredOutlined("Initializing...", 20);

  // Test TFT
  if (tft.width() == 0 || tft.height() == 0) {  // Init fail check
    Serial.println("TFT Fail");
    tft.fillScreen(ILI9341_RED);  // Red screen if fail
  } else {
    Serial.println("TFT OK");
  }

  // SD init
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);  // Ensure TFT is deselected
  if (!SD.begin(SD_CS)) {
    printCenteredOutlined("SD Fail", 40, ILI9341_RED);
    Serial.println("SD Fail");
    delay(2000);
  } else {
    Serial.println("SD OK");
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      String name = entry.name();
      name.toUpperCase();  // Convert to uppercase for case-insensitive check
      if (name.endsWith(".CSV") && numSeqFiles < 10) {
        seqFiles[numSeqFiles++] = entry.name();  // Store original name
      }
      entry.close();
    }
    root.close();
  }

  // Defaults
  g_fHexOn = 0;
  PiControllerSwitch = 0;
  ModeSelect = 0;
  BalanceMode = 0;
  DoubleTravel = 0;
  DoubleLegLiftHeight = 0;
  WalkMethod = 0;
  SLHold = 0;
  GaitPreset = 0;
  SelectedLeg = 255;
  GPSequence = 0;

  delay(1000);
  tft.fillScreen(ILI9341_BLACK);

  // Draw main UI
  byte initialStates[6] = {g_fHexOn, PiControllerSwitch, ModeSelect, BalanceMode, DoubleTravel, DoubleLegLiftHeight};
  drawStatusBars(initialStates);
  drawWalkMethod(WalkMethod);
  drawJoystickBase();
  drawSlidersBase();
  updateLeftJoystick(leftFwdBack, leftLeftRight, leftTwist, 128, 128, 128);
  updateRightJoystick(rightUpDown, rightLeftRight, rightTwist, 128, 128, 128);
  updateSlider(true, bodyHeight, 0);
  updateSlider(false, gaitSpeed, 0);
}

void loop() {
  // Read analogs
  leftFwdBack = map(analogRead(analogPins[0]), 0, 1023, 0, 255);
  leftLeftRight = map(analogRead(analogPins[1]), 0, 1023, 0, 255);
  leftTwist = map(analogRead(analogPins[2]), 0, 1023, 0, 255);
  rightUpDown = map(analogRead(analogPins[3]), 0, 1023, 0, 255);
  rightLeftRight = map(analogRead(analogPins[4]), 0, 1023, 0, 255);
  rightTwist = map(analogRead(analogPins[5]), 0, 1023, 0, 255);
  bodyHeight = map(analogRead(analogPins[6]), 0, 1023, 0, 255);
  gaitSpeed = map(analogRead(analogPins[7]), 0, 1023, 0, 255);

  // Read switches
  byte currentSwitches[6];
  currentSwitches[0] = g_fHexOn = (digitalRead(switchPins[0]) == LOW) ? 255 : 0;
  currentSwitches[1] = PiControllerSwitch = (digitalRead(switchPins[1]) == LOW) ? 255 : 0;
  currentSwitches[2] = ModeSelect = (digitalRead(switchPins[2]) == LOW) ? 255 : 0;
  currentSwitches[3] = BalanceMode = (digitalRead(switchPins[3]) == LOW) ? 255 : 0;
  currentSwitches[4] = DoubleTravel = (digitalRead(switchPins[4]) == LOW) ? 255 : 0;
  currentSwitches[5] = DoubleLegLiftHeight = (digitalRead(switchPins[5]) == LOW) ? 255 : 0;

  // Check if any switch changed
  bool switchesChanged = false;
  for (int i = 0; i < 6; i++) {
    if (currentSwitches[i] != prevSwitches[i]) {
      switchesChanged = true;
      prevSwitches[i] = currentSwitches[i];
    }
  }
  if (switchesChanged) {
    drawStatusBars(currentSwitches);
  }

  // Buttons press detection
  bool buttonPressed[6] = {false};
  for (int i = 0; i < 6; i++) {
    bool reading = (digitalRead(buttonPins[i]) == LOW);
    if (reading) {
      unsigned long currentTime = millis();
      if (currentTime - lastButtonTime[i] > debounceDelay) {
        if (lastButtonTime[i] == 0) {  // First press
          buttonPressed[i] = true;
          lastButtonTime[i] = currentTime;
        }
      }
    } else {
      lastButtonTime[i] = 0;
      longPressed[i] = false;
    }
  }

  // Long press detection for menu buttons
  int menuButtons[3] = {1, 3, 4};  // indices for SLEG, GAIT, GP
  MenuState menuStates[3] = {SLEG_MENU, GAIT_MENU, GP_MENU};
  for (int j = 0; j < 3; j++) {
    int i = menuButtons[j];
    bool reading = (digitalRead(buttonPins[i]) == LOW);
    if (reading && lastButtonTime[i] > 0 && (millis() - lastButtonTime[i] > longPressDelay) && !longPressed[i]) {
      if (currentMenu == menuStates[j]) {
        currentMenu = NO_MENU;
        tft.fillRect(0, 25, 320, 190, ILI9341_BLACK);
        drawJoystickBase();
        drawSlidersBase();
        updateLeftJoystick(leftFwdBack, leftLeftRight, leftTwist, leftFwdBack, leftLeftRight, leftTwist);
        updateRightJoystick(rightUpDown, rightLeftRight, rightTwist, rightUpDown, rightLeftRight, rightTwist);
        updateSlider(true, bodyHeight, bodyHeight);
        updateSlider(false, gaitSpeed, gaitSpeed);
        drawWalkMethod(WalkMethod);  // Redraw method text
        longPressed[i] = true;
        if (currentMenu == SLEG_MENU) {
          SelectedLeg = 255; // Reset single leg on exit
        }
        // No reset for gait or GP on exit - keep last selected
      }
    }
  }

  // Encoder delta
  noInterrupts();
  int delta = deltaAccum;
  deltaAccum = 0;
  interrupts();

  // Handle buttons and menus
  if (currentMenu == NO_MENU) {
    // Update UI only if changed
    if (leftFwdBack != prevLeftFwdBack || leftLeftRight != prevLeftLeftRight || leftTwist != prevLeftTwist) {
      updateLeftJoystick(leftFwdBack, leftLeftRight, leftTwist, prevLeftFwdBack, prevLeftLeftRight, prevLeftTwist);
      prevLeftFwdBack = leftFwdBack;
      prevLeftLeftRight = leftLeftRight;
      prevLeftTwist = leftTwist;
    }
    if (rightUpDown != prevRightUpDown || rightLeftRight != prevRightLeftRight || rightTwist != prevRightTwist) {
      updateRightJoystick(rightUpDown, rightLeftRight, rightTwist, prevRightUpDown, prevRightLeftRight, prevRightTwist);
      prevRightUpDown = rightUpDown;
      prevRightLeftRight = rightLeftRight;
      prevRightTwist = rightTwist;
    }
    if (bodyHeight != prevBodyHeight) {
      updateSlider(true, bodyHeight, prevBodyHeight);
      prevBodyHeight = bodyHeight;
    }
    if (gaitSpeed != prevGaitSpeed) {
      updateSlider(false, gaitSpeed, prevGaitSpeed);
      prevGaitSpeed = gaitSpeed;
    }
  } else {
    // Handle encoder in menus - live update variables
    if (delta != 0) {
      menuValue += delta;
      if (currentMenu == SLEG_MENU) {
        menuValue = constrain(menuValue, 1, 6); // 1-6 legs
        SelectedLeg = menuValue - 1; // Live update
        char buf[20];
        sprintf(buf, "Select Leg: %d", menuValue);
        tft.fillRect(0, 30, 320, 20, ILI9341_BLACK); // Clear text
        printCenteredOutlined(buf, 30);
        drawHexapod(menuValue); // Comment this out if flickering/mess up
      } else if (currentMenu == GAIT_MENU) {
        menuValue = constrain(menuValue, 0, 7); // 0-7 gaits
        GaitPreset = menuValue; // Live update
        tft.fillRect(0, 60, 320, 20, ILI9341_BLACK); // Clear name
        printCenteredOutlined(gaitNames[menuValue], 60);
      } else if (currentMenu == GP_MENU) {
        menuValue = constrain(menuValue, 0, numSeqFiles); // 0 off, 1-N seq
        GPSequence = menuValue; // Live update
        char buf[30];
        if (menuValue == 0) {
          sprintf(buf, "Off");
        } else {
          sprintf(buf, "%s", seqFiles[menuValue - 1].c_str());
        }
        tft.fillRect(0, 60, 320, 20, ILI9341_BLACK);
        printCenteredOutlined(buf, 60);
      }
    }
  }

  // Button 0: Walk method toggle
  if (buttonPressed[0]) {
    WalkMethod = !WalkMethod;
  }

  if (WalkMethod != prevWalkMethod) {
    drawWalkMethod(WalkMethod);
    prevWalkMethod = WalkMethod;
  }

  // Button 1: S-Leg menu
  if (buttonPressed[1] && currentMenu == NO_MENU) {
    currentMenu = SLEG_MENU;
    menuValue = 1;
    SelectedLeg = 0; // Instantly select first leg
    tft.fillRect(0, 25, 320, 190, ILI9341_BLACK);  // Clear between bars
    tft.setTextSize(2);
    char buf[20];
    sprintf(buf, "Select Leg: %d", menuValue);
    printCenteredOutlined(buf, 30);
    drawHexapod(menuValue);
  }

  // Button 2: SL Hold toggle
  if (buttonPressed[2]) {
    SLHold = !SLHold;
    // Optional: flash message
  }

  // Button 3: Gait menu
  if (buttonPressed[3] && currentMenu == NO_MENU) {
    currentMenu = GAIT_MENU;
    menuValue = 0;
    GaitPreset = 0; // Instantly select first gait
    tft.fillRect(0, 25, 320, 190, ILI9341_BLACK);
    printCenteredOutlined("Select Gait:", 40);
    printCenteredOutlined(gaitNames[menuValue], 60);
  }

  // Button 4: GP menu
  if (buttonPressed[4] && currentMenu == NO_MENU) {
    currentMenu = GP_MENU;
    menuValue = 0;
    GPSequence = 0; // Instantly off
    tft.fillRect(0, 25, 320, 190, ILI9341_BLACK);
    printCenteredOutlined("Select Sequence:", 40);
    printCenteredOutlined("Off", 60); // Default off
  }

  // Removed encoder SW confirm - now live updates on encoder turn

  // Send string to Serial1 (ESP-TX) every loop
  char buf[100];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          leftFwdBack, leftLeftRight, leftTwist, rightUpDown, rightLeftRight, rightTwist,
          bodyHeight, gaitSpeed, g_fHexOn, PiControllerSwitch, ModeSelect, BalanceMode,
          DoubleTravel, DoubleLegLiftHeight, WalkMethod, SLHold, GaitPreset, SelectedLeg, GPSequence);
  Serial1.print(buf);
}
