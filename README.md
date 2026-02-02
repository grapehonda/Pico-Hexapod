 -WIP- Porting to Pico 2.

- UART0 - SSC-32
- UART1 - Pi Zero 2W
- SerialPIO - ESP-NOW rx

- Hanus.py needs updated to support new channel mapping, plus additional channels.
- TFT Controller runs on Arduino Mega and outputs to ESP-NOW on UART1. ESP-NOW just works as a dumb bridge- nothing else.
