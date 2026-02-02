#define DBGSerial         Serial
#define SSCSerial         Serial1


//[SERIAL CONNECTIONS]
#define cSSC_OUT         0     //Pico TX GPIO0
#define cSSC_IN          1     //PICO RX GPIO1
#define cSSC_BAUD        115200     //SSC32 BAUD rate
#define cSSC_BINARYMODE 0     // Define if your SSC-32 card supports binary mode.

//--------------------------------------------------------------------
//[SSC PIN NUMBERS]
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal
#define cRRFemurPin     1   //Rear Right leg Hip Vertical
#define cRRTibiaPin     2   //Rear Right leg Knee

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal
#define cRMFemurPin     5   //Middle Right leg Hip Vertical
#define cRMTibiaPin     6   //Middle Right leg Knee

#define cRFCoxaPin      8   //Front Right leg Hip Horizontal
#define cRFFemurPin     9   //Front Right leg Hip Vertical
#define cRFTibiaPin     10   //Front Right leg Knee

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal
#define cLRFemurPin     17   //Rear Left leg Hip Vertical
#define cLRTibiaPin     18   //Rear Left leg Knee

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal
#define cLMFemurPin     21   //Middle Left leg Hip Vertical
#define cLMTibiaPin     22   //Middle Left leg Knee

#define cLFCoxaPin      24   //Front Left leg Hip Horizontal
#define cLFFemurPin     25   //Front Left leg Hip Vertical
#define cLFTibiaPin     26   //Front Left leg Knee
//--------------------------------------------------------------------
//[Which Servos to Invert]
#define cRRFemurInv  1
#define cRMFemurInv  1
#define cRFFemurInv  1
#define cLRFemurInv  1
#define cLMFemurInv  1
#define cLFFemurInv  1
#define cLFCoxaInv  -1
#define cLMCoxaInv  -1
#define cLRCoxaInv  -1
#define cRRCoxaInv  -1
#define cRMCoxaInv  -1
#define cRFCoxaInv  -1
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1    -550      //Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     550
#define cRRFemurMin1   -900
#define cRRFemurMax1    550
#define cRRTibiaMin1   -300
#define cRRTibiaMax1    650

#define cRMCoxaMin1    -550      //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1     550
#define cRMFemurMin1   -900
#define cRMFemurMax1    550
#define cRMTibiaMin1   -300
#define cRMTibiaMax1    650

#define cRFCoxaMin1    -550      //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     550
#define cRFFemurMin1   -900
#define cRFFemurMax1    550
#define cRFTibiaMin1   -300
#define cRFTibiaMax1    650

#define cLRCoxaMin1    -550      //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     550
#define cLRFemurMin1   -900
#define cLRFemurMax1    550
#define cLRTibiaMin1   -300
#define cLRTibiaMax1    650

#define cLMCoxaMin1    -550      //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1     550
#define cLMFemurMin1   -900
#define cLMFemurMax1    550
#define cLMTibiaMin1   -300
#define cLMTibiaMax1    650

#define cLFCoxaMin1    -550      //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     550
#define cLFFemurMin1   -900
#define cLFFemurMax1    550
#define cLFTibiaMin1   -300
#define cLFTibiaMax1    650
//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cCoxaLength      34      //1.14" = 29mm (1.14 * 25.4)
#define cFemurLength     92      //2.25" = 57mm (2.25 * 25.4)
#define cTibiaLength     150     //5.55" = 141mm (5.55 * 25.4)

#define cRRCoxaAngle1   -600    //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    600    //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1   -600    //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    600    //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -69      //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ       119     //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX      -138     //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ       0       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX      -69      //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -119     //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX       69      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ       119     //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX       138     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ       0       //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX       69      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -119     //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ       90   
#define CHexInitXZCos60  45
#define CHexInitXZSin60  78
#define CHexInitY        60  // Decreased from 80 to 60 to lower rest pose by ~20mm (fixes 15mm lift)


#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ           //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ    -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ           //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ    -CHexInitXZSin60
//--------------------------------------------------------------------
