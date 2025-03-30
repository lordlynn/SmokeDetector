# Smoke Detector
Uses a Bosch BME690 air qualirty sensor to detect VOC concentrations in the air. When levels exceed the calibration values, it indicates that there is an excessive concentration of VOCs or smoke in the air. At this point, LEDs begin flashing to provide a visual indicator, a piezo buzzer is activated to provide an auditory indicator, and a warning message is sent over Bluetooth and serial connections. A history of high VOC events are also recorded to an EEPROM.

This project required the development of a custom surface mount PCB that was designed in Altium. The PCB was assembled by hand at GVSU.
![PCBImage](https://github.com/user-attachments/assets/67bca363-61c2-4f12-9f74-aefc112d7333)

The system was designed to run off of a coin cell battery and had an optional solar panel and USB charging inputs. The battery/charging circuit was a separate PCB as it was used as our first experience assembly surface mount boards. So to simply the PCB design, screw terminals were included to connect the two PCBs.
