# Nucleo L432KC OBD2 Scanner for Corolla Verso 2007 D4D
## General Notes
The Corolla Verso (2007, diesel) uses k-line for OBD2 data link layer; the k-line uses pin 7 from the OBD2 adapter. It is readily interfaced to a UART peripheral.

## Hardware
- Nucleo L432KC development board
- Murata OKI-78SR-5/1.5-W36-C voltage converter 
- ST L9637 - ISO9141 interface
- DB9 male/female connectors
- OBD2 male connector
- 128x64 (or 132x64) OLED display

## Software
The software runs a cooperative scheduler and a couple of protocol parsers - one for commands coming from the VCP (Virtual Com Port), and one for messages coming from the k-line. See comments in source for details. A detailed explanation will follow soon; the device is under development.

## Disclaimer
Please do read about k-line, OBD2 and voltage levels in cars before reading the code. Totalling/setting your car on fire is a probable outcome of messing around with car electronics. The author is NOT responsable for any damages resulting from proper or improper usage of this code.