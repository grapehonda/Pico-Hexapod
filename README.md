 -WIP- pico port with extras
 Pi zero 2w takes control automatically when controller is off or switched to pi via controller toggle.
 Flask server talks to LLM- sentiment analysis triggers different poses/animations.
 Random idle triggered animations
 Trying to update the GPPlayer mode to select sequences from controller's menu
 Needs a way to read the csv files stored on controller's SD card and use them to play sequences


- UART0 - SSC-32
- UART1 - Pi Zero 2W
- SerialPIO - ESP-NOW rx
- Hanus.py needs updated to support new channel mapping, plus additional channels.
- Once GPPlayer works as intended, Hanus.py will use sequences and for animations instead of preset strings of fake controller inputs
- TFT Controller runs on Arduino Mega and outputs to ESP-NOW tx on UART1.
- ESP-NOW just works as a dumb bridge- nothing else.
 

 
 
 
 The original:
- Project Lynxmotion Phoenix
- Description: Phoenix software
- Software version: V2.0
- Date: 29-10-2009
- Programmer: 
- Jeroen Janssen [aka Xan]
- Kurt Eckhardt(KurtE) converted to C and Arduino
- Kåre Halvorsen aka Zenta - Makes everything work correctly!
  
