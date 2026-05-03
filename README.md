# MS43-to-E90-CAN-Translator-ESP32
A CAN bus translator bridging BMW MS43 ECU messages to the E90 platform CAN protocol for gauge cluster, ABS, and DSC integration.

This repo is based on [pstrzoda/MS43-to-E90-CanBus-Translator](https://github.com/pstrzoda/MS43-to-E90-CanBus-Translator) but modified for ESP32 using the [TechOverflow/esp32_can](https://github.com/TechOverflow/esp32_can) library and a [CANipulator](https://www.tindie.com/products/fusion/canipulator-automotive-dual-can-esp32-interface/) board.

---
## Install
- Download [collin80/can_common](https://github.com/collin80/can_common) and place in your Arduino libraries folder
- Download [TechOverflow/esp32_can](https://github.com/TechOverflow/esp32_can) and place in your Arduino libraries folder
- Download [TechOverflow/SmartLeds](https://github.com/TechOverflow/SmartLeds) and place in your Arduino libraries folder
- Download this repo and place the sketch folder into your Arduino folder
- On line 703 check `CAN0.begin()` speed : 100000 for K-CAN (Body/Kombi) or 500000 for PT-CAN
- Upload to the board
