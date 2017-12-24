# Arduino based Bluetooth Remote Control Car

This is the code for a Bluetooth connected RC car based on the ["Sunfounder Smart Video Car Kit for Raspberry Pi"](https://www.sunfounder.com/rpi-car.html). The RasPi was replaced with an Arduino Nano and the WiFi and Camera removed.

Overall this is just a very simple example of using some common ICs:

* L298N H-bridge motor driver
* PCA9685 16 channel PWM driver
* HC-06 Bluetooth/Serial module

The L298N motor direction is controlled directly via digital pins on the Arduino. Motor speed is controlled via PWM duty cycle from the PCA9685.

To drive the PCA9685 the [Adafruit library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) for [their breakout board](https://www.adafruit.com/product/815) is used. Thanks Adafruit!

The HC-06 Bluetooth module does not require any code. It is just connected to the hardware serial pins of the Arduino. But some older commits contain code to configure it via SoftwareSerial.

