// Arduino based RC car with bluetooth connection
//
// Base: Sunfounder Smart Video Car Kit for RasPi (w/o Smart Video and RasPi)
// 1x Servo for steering
// 2x DC Motor for speed
// 1x Bluetooth module for remote control (via bt<->serial adapter)

// JY-MCU - Bluetooth/Serial cross-wired
//   TXD <- Pin D10 (RX)
//   RXD <- Pin D11 (TX)

// L298N - DC Motor driver:
//   IN1 <- Pin D2
//   IN2 <- Pin D3
//   IN3 <- Pin D4
//   IN4 <- Pin D5

// PCA9685 - Servo controller PWM driver:
//   SDA <- Pin A4
//   SCL <- Pin A5
//   CH0 -> steering servo
//   CH4 -> right wheel duty cycle
//   CH5 -> left wheel duty cycle

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

SoftwareSerial btSerial(10, 11); // RX, TX
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

enum Pins {
  M1_FWD = 2,
  M1_REV = 3,
  M2_FWD = 4,
  M2_REV = 5
};

enum Constants {
  NEUTRAL      = 2300, // Direction: Straight (requires calibration)
  PWM_HZ       = 400, // PWM duty cycle frequency (40 Hz - 1kHz)
  CH_DIRECTION = 0,    // Steering PWM channel
  CH_M1_SPEED  = 4,    // Motor 1 PWM channel
  CH_M2_SPEED  = 5,    // Motor 2 PWM channel
  BT_BAUD      = 57600 // Baud Rate of Bluetooth Serial
};

bool forward;
int16_t speed;
int16_t direction;

// 12 bit PWM speed control, affects both motors
// stop = 0
// full = 4095
void set_speed()
{
  pwm.setPWM(CH_M1_SPEED, 0, speed);
  pwm.setPWM(CH_M2_SPEED, 0, speed);
}

// 12 bit PWM for direction
// left = 0
// straight = NEUTRAL (requires calibration)
// right = 4095
void set_direction()
{
  pwm.setPWM(CH_DIRECTION, 0, direction);
}

// Full STOP
void full_stop()
{
  digitalWrite(M1_FWD, LOW);
  digitalWrite(M2_FWD, LOW);
  digitalWrite(M1_REV, LOW);
  digitalWrite(M2_REV, LOW);

  speed = 0;
  set_speed();

  direction = NEUTRAL;
  set_direction();

  forward = false;
}

// Reverse the direction
void reverse()
{
  forward ^= 0x1;

  if (forward) {
    digitalWrite(M1_REV, LOW);
    digitalWrite(M2_REV, LOW);
    digitalWrite(M1_FWD, HIGH);
    digitalWrite(M2_FWD, HIGH);
  } else {
    digitalWrite(M1_FWD, LOW);
    digitalWrite(M2_FWD, LOW);
    digitalWrite(M1_REV, HIGH);
    digitalWrite(M2_REV, HIGH);
  }
}

/*
void waitForResponse() {
    delay(1000);
    while (btSerial.available()) {
      Serial.write(btSerial.read());
    }
    Serial.write("\n");
}

void configureBtSerial() {
  Serial.println("Starting config");
  btSerial.begin(BT_BAUD);
  delay(1000);

  // Should respond with OK
  btSerial.print("AT");
  waitForResponse();

  // Should respond with its version
  btSerial.print("AT+VERSION");
  waitForResponse();

  // Set pin to 0000
  btSerial.print("AT+PIN0000");
  waitForResponse();

  // Set the name
  btSerial.print("AT+NAME");
  btSerial.print("SerialRobot");
  waitForResponse();

  // Set baudrate to 57600
  btSerial.print("AT+BAUD7");
  waitForResponse();

  Serial.println("Done!");
}
*/

void setup()
{
  //Serial.begin(115200);

  btSerial.begin(BT_BAUD);
  // configureBtSerial();

  forward = false;

  pinMode(M1_FWD, OUTPUT);
  pinMode(M1_REV, OUTPUT);
  pinMode(M2_FWD, OUTPUT);
  pinMode(M2_REV, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(PWM_HZ);
  full_stop();
}

void loop()
{
  // Read input
  int cmd = 0;
  if (btSerial.available()) {
    cmd = btSerial.read();
  }

  // Adapt movement
  switch (cmd) {
  case 'j':
    direction = max(0, direction - 0xff);
    btSerial.print("Go Left: ");
    btSerial.println(direction);
    set_direction();
    break;
  case 'k':
    speed = min(0x0fff, speed + 0xff);
    btSerial.print("Go Faster: ");
    btSerial.println(speed);
    set_speed();
    break;
  case 'l':
    speed = max(0, speed - 0xff);
    btSerial.print("Go Slower: ");
    btSerial.println(speed);
    set_speed();
    break;
  case ';':
    direction = min(0x0fff, direction + 0xff);
    btSerial.print("Go Right: ");
    btSerial.println(direction);
    set_direction();
    break;
  case 'n':
    direction = NEUTRAL;
    btSerial.print("Go Straight: ");
    btSerial.println(direction);
    set_direction();
    break;
  case 's':
    speed = 0;
    btSerial.println("Full STOP!");
    full_stop();
    break;
  case ' ':
    reverse();
    btSerial.print("Reverse! Going ");
    btSerial.println(forward ? "FORWARD" : "BACKWARDS");
  default:
    break;
  }
}

