Arduino-PID-Controller
============
This project is based on the [Adafruit Sous-Vide controller](https://github.com/adafruit/Sous_Viduino) by Bill Earl with the following changes:

* cheap 1602 LCD Panel instead of the Adafruit RGB LCD Shield
* encoder with push button instead of a keypad
* robust Hardware Design (not described in this Repo) with Schuko socket
* navigation Menu
* and other minor changes

## Pictures

![Front](/photos/front.jpg)
![Back](/photos/back.jpg)
![Electronics](/photos/electronic.jpg)

## Todo
- [] get higher resolution temperature readings
- [] add socket for different sensor probes (e.g. PT1000) and sensor selection menu or auto deterction
- [] add sensor calibration mode
- [] add memory function to store and load the settings for different environment this controller is used in (P,I,D, direction, sensor, sensor calibration)
- [] add Calibration Mode for temperature sensor
- [] add Error message if autotune fails
- [] normalize noisy temperature readings
- [] make digits of setpoint and sensor reading beneath one another with trailing zeros
- [] define special characters for the lcd as array
- [] match speed of controll loop to frequency of main power, use optocoupler to detect zero crossing


## Libraries

This code uses third party libraries 

Libraries for PID Controll
* [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)
* [Arduino-PID-AutoTune-Library](https://github.com/br3ttb/Arduino-PID-AutoTune-Library)
Libraries for the LCD
* [Arduino-LiquidCrystal-I2C-library](https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library)

Libraries for the DS18B20 Temperature Sensor
* [OneWire](https://github.com/PaulStoffregen/OneWire)
* [Arduino-Temperature-Control-Library](
https://github.com/milesburton/Arduino-Temperature-Control-Library)

Library for the Encoder
* [RotaryEncoder](https://github.com/mathertel/RotaryEncoder)
