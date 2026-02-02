#ifndef INPUT_WIRELESS_H
#define INPUT_WIRELESS_H

#include <Arduino.h>
#include "Hex_Globals.h"
#include <SerialPIO.h>
SerialPIO WirelessSerial(8, 9);

#define MAX_BODY_Y 100  // Max body height in mm
#define NUM_GAITS 8     // 0-7 gaits
#define WALKMODE 0
#define TRANSLATEMODE 1
#define SINGLELEGMODE 2
#define GPPLAYERMODE 3

#define cTravelDeadZone 20 // Deadzone for sticks

// Global - Local to this file only
static unsigned g_BodyYOffset; 
static byte ControlMode;
static bool DoubleTravelOn;
static bool WalkMethod;

// Init Controller
void InitController(void) {
  WirelessSerial.begin(115200);  // Wireless from ESP-RX was Serial3 on Mega
  Serial2.begin(100000);  // Match Pi baud rate
  Serial2.setTimeout(10); // Low timeout to avoid blocking

  g_fHexOn = 0;
  GaitType = 0;
  LegLiftHeight = 50;
  SpeedControl = 100;
  TravelLengthX = 0;
  TravelLengthZ = 0;
  TravelRotationY = 0;
  DoubleTravelOn = false;
  WalkMethod = 0;
  g_BodyYOffset = 0;
  SelectedLeg = 255;
  fSLHold = false;
  BalanceMode = false;
  GaitSelect();
  ControlMode = TRANSLATEMODE;
}

// Helper function to count commas in buffer
int countCommas(const char* buf) {
  int count = 0;
  while (*buf) {
    if (*buf == ',') count++;
    buf++;
  }
  return count;
}

// Helper function to parse buffer into byte array (19 values)
bool parseBufferToValues(const char* buf, byte* values) {
  char tempBuf[150];  // Larger for 19 values
  strcpy(tempBuf, buf);
  char* token = strtok(tempBuf, ",");
  int idx = 0;
  while (token && idx < 19) {
    values[idx] = (byte)atoi(token);
    token = strtok(NULL, ",");
    idx++;
  }
  return (idx == 19);  // Ensure exactly 19 values parsed
}

// Control Input
void ControlInput(void) {
  char inBuf[150] = {0};
  byte values[19]; // Bumped to 19
  bool gotInput = false;

  // Try wireless (Serial3) first
  if (WirelessSerial.available() > 20) {  // Lowered threshold
    char wirelessBuf[150] = {0};
    int bytesRead = WirelessSerial.readBytesUntil('\n', wirelessBuf, sizeof(wirelessBuf) - 1);
    if (bytesRead >= 38) {  // Min for 19 vals (~38 chars: "0,0,...\n")
      // Trim whitespace
      char* ptr = wirelessBuf;
      while (*ptr == ' ' || *ptr == '\r' || *ptr == '\n') ptr++;
      int commaCount = countCommas(ptr);
      if (commaCount == 18 && parseBufferToValues(ptr, values)) {
        byte switch_val = values[9];  // ch10: piControllerSwitch
        if (switch_val > 127) {  // >127 for high (was 128, now >127 for tolerance)
          // Use wireless
          gotInput = true;
        } else {
          // Switch low, try Pi
          if (Serial2.available() > 20) {
            char piBuf[150] = {0};
            int piBytes = Serial2.readBytesUntil('\n', piBuf, sizeof(piBuf) - 1);
            if (piBytes >= 38) {  // Added garbage check
              char* piPtr = piBuf;
              while (*piPtr == ' ' || *piPtr == '\r' || *piPtr == '\n') piPtr++;
              int piCommaCount = countCommas(piPtr);
              if (piCommaCount == 18 && parseBufferToValues(piPtr, values)) {
                gotInput = true;
              }
            }
          }
        }
      }
    }
  } else {
    // No wireless, default to Pi if available and switch low (but since no wireless avail, check switch from last parse or default)
    if (Serial2.available() > 20) {
      char piBuf[150] = {0};
      int piBytes = Serial2.readBytesUntil('\n', piBuf, sizeof(piBuf) - 1);
      if (piBytes >= 38) {
        char* piPtr = piBuf;
        while (*piPtr == ' ' || *piPtr == '\r' || *piPtr == '\n') piPtr++;
        int piCommaCount = countCommas(piPtr);
        if (piCommaCount == 18 && parseBufferToValues(piPtr, values)) {
          gotInput = true;
        } 
      }
    }
  }

  if (!gotInput) {
    return; // No input, skip processing
  }

  // Use the pre-parsed values[] (rest same as before)
  short ch1 = values[0] - 128;      //ch1  Left forward/back
  short ch2 = values[1] - 128;      //ch2  Left left/right
  short ch3 = values[2] - 128;      //ch3  Left twist
  short ch4 = values[3] - 128;      //ch4  Right up/down (pitch)
  short ch5 = values[4] - 128;      //ch5  Right left/right (roll)
  short ch6 = values[5] - 128;      //ch6  Right twist (yaw)
  byte bodyheightpot = values[6];   //ch7  Body Height
  byte gaitspeedpot = values[7];    //ch8  Gait Speed
  byte hexon = values[8];           //ch9  HexOn
  //ch10 Ignored here, now used for Pi/Controller switch)
  byte modeselect = values[10];     //ch11 Mode Select Translate/Walk
  byte balancemode = values[11];    //ch12 Balance Mode
  byte doubletravel = values[12];   //ch13 Double Travel
  byte doubleleglift = values[13];  //ch14 Double Leg Lift Height
  byte walkmeth = values[14];       //ch15 Walk Method 1/2
  byte slhold = values[15];         //ch16 SingleLeg Hold
  byte gaitpreset = values[16];     //ch17 Gait Preset (0-7)
  byte selectedleg = values[17];    //ch18 Selected Leg (255 off, 0-5 legs)
  byte gpsequence = values[18];     //ch19 GP Sequence (0 off, 1+ sequence)

  // Hex on/off
  bool fNewHexOn = (hexon > 128);
  if (fNewHexOn != g_fHexOn) {
    if (fNewHexOn) {
      g_fHexOn = 1;
    } else {
      BodyPosX = BodyPosY = BodyPosZ = 0;
      BodyRotX1 = BodyRotY1 = BodyRotZ1 = 0;
      TravelLengthX = TravelLengthZ = TravelRotationY = 0;
      g_BodyYOffset = 0;
      SelectedLeg = 255;
      g_fHexOn = 0;
    }
  }

  if (g_fHexOn) {
    BalanceMode = (balancemode > 128);
    g_BodyYOffset = map(bodyheightpot, 0, 255, 0, MAX_BODY_Y);
    SpeedControl = map(gaitspeedpot, 0, 255, 2000, 100); // Reversed: 0=slowest (2000), 255=fastest (100)
    NomGaitSpeed = map(gaitspeedpot, 0, 255, 50, 200);  // adjust range to taste
    WalkMethod = (walkmeth > 128);
    DoubleTravelOn = (doubletravel > 128);
    LegLiftHeight = (doubleleglift > 128) ? 80 : 50;
    GaitType = gaitpreset;
    GaitSelect();
    fSLHold = (slhold > 128);
    SelectedLeg = selectedleg;  // Assume 255 off, 0-5 legs

    // Deadzone
    if (abs(ch1) < cTravelDeadZone) ch1 = 0;
    if (abs(ch2) < cTravelDeadZone) ch2 = 0;
    if (abs(ch3) < cTravelDeadZone) ch3 = 0;
    if (abs(ch4) < cTravelDeadZone) ch4 = 0;
    if (abs(ch5) < cTravelDeadZone) ch5 = 0;
    if (abs(ch6) < cTravelDeadZone) ch6 = 0;

    // Mode selection
    if (gpsequence != 0) {
      ControlMode = GPPLAYERMODE;
      // GPSeq = gpsequence - 1;  // Uncomment if main uses GPSeq var (0-based)
    } else if (SelectedLeg != 255) {
      ControlMode = SINGLELEGMODE;
    } else if (modeselect > 128) {
      ControlMode = WALKMODE;
    } else {
      ControlMode = TRANSLATEMODE;
    }

    // Body rotations – always active
    if (ControlMode != SINGLELEGMODE) {
      BodyRotX1 = ch4;      // pitch
      BodyRotZ1 = ch5;      // roll
      BodyRotY1 = ch6 * 2;  // yaw
    } else {
      BodyRotX1 = BodyRotY1 = BodyRotZ1 = 0;
    }

    // === Walk mode ===
    if (ControlMode == WALKMODE) {
      TravelLengthX = -ch2;
      TravelLengthZ = ch1;
      TravelRotationY = -ch3 / 4;

      if (DoubleTravelOn) {
        TravelLengthX /= 2;
        TravelLengthZ /= 2;
        TravelRotationY /= 2;
      }

      BodyPosX = 0;
      BodyPosZ = 0;

      InputTimeDelay = 128 - max(max(abs(ch1), abs(ch2)), abs(ch3));
    }

    // === Translate mode ===
    else if (ControlMode == TRANSLATEMODE) {
      BodyPosX = -ch2 / 2;
      BodyPosZ = ch1 / 3;
      TravelLengthX = TravelLengthZ = TravelRotationY = 0;

      InputTimeDelay = 128 - max(max(abs(ch1), abs(ch2)), max(abs(ch4), max(abs(ch5), abs(ch6))));
    }

    // === Single leg mode ===
    else if (ControlMode == SINGLELEGMODE) {
      // Left stick up/down: leg height (Y)
      SLLegY = -ch1 / 5;
      // Right stick left/right: side to side (X)
      SLLegX = -ch5 / 2;
      // Right stick fwd/back: extend out (Z)
      SLLegZ = -ch4 / 2;

      // Reset hold ONLY on first entry to single-leg (not every loop)
      static byte prev_SelectedLeg = 255;
      if (SelectedLeg != prev_SelectedLeg) {
        fSLHold = false;  // default off when switching legs or entering
        prev_SelectedLeg = SelectedLeg;
      }

      TravelLengthX = TravelLengthZ = TravelRotationY = 0;
      InputTimeDelay = 128 - max(max(abs(ch1), abs(ch4)), abs(ch5));
    }
  }

  BodyPosY = max(g_BodyYOffset, 0);
}

void AllowControllerInterrupts(boolean fAllow) {
}

#endif
