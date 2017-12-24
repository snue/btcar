// Arduino based RC car with bluetooth connection
//
// Base: Sunfounder Smart Video Car Kit for RasPi (w/o Smart Video and RasPi)
// 1x Arduino Nano MCU
// 1x Servo for steering
// 2x DC Motor for speed
// 1x Bluetooth<->Serial module for remote control

// JY-MCU - Bluetooth/Serial module (cross-wired):
//   TXD <- Pin 1 (RX)
//   RXD <- Pin 0 (TX)

// L298N - H-Bridge DC Motor driver:
//   IN1 <- Pin D2 (Motor 1 Fwd)
//   IN2 <- Pin D3 (Motor 1 Reverse)
//   IN3 <- Pin D4 (Motor 2 Fwd)
//   IN4 <- Pin D5 (Motor 2 Reverse)

// PCA9685 - 16 channel PWM Servo driver:
//   SDA <- Pin A4
//   SCL <- Pin A5
//   CH0 -> steering servo
//   CH4 -> right wheel duty cycle
//   CH5 -> left wheel duty cycle

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

enum Pins {
  M1_FWD = 2,
  M1_REV = 3,
  M2_FWD = 4,
  M2_REV = 5
};

enum Constants {
  NEUTRAL      = 2300, // Direction: Straight (requires calibration)
  PWM_HZ       = 400,  // PWM duty cycle frequency (40 Hz - 1kHz)
  CH_DIRECTION = 0,    // Steering PWM channel
  CH_M1_SPEED  = 4,    // Motor 1 PWM channel
  CH_M2_SPEED  = 5,    // Motor 2 PWM channel
  BT_BAUD      = 57600 // Baud Rate of Bluetooth Serial
};

// Just keep current movement settings in global variables
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
// straight = NEUTRAL
// right = 4095
void set_direction()
{
  // TODO: use map() function to calibrate
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

  // CAUTION: Always pull one output to LOW first! Never set both directions HIGH!
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

void setup()
{
  Serial.begin(BT_BAUD);

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
  if (Serial.available()) {
    cmd = Serial.read();
  }

  // Adapt movement
  switch (cmd) {
  case 'j':
    direction = max(0, direction - 0xff);
    Serial.print("Go Left: ");
    Serial.println(direction);
    set_direction();
    break;
  case 'k':
    speed = min(0x0fff, speed + 0xff);
    Serial.print("Go Faster: ");
    Serial.println(speed);
    set_speed();
    break;
  case 'l':
    speed = max(0, speed - 0xff);
    Serial.print("Go Slower: ");
    Serial.println(speed);
    set_speed();
    break;
  case ';':
    direction = min(0x0fff, direction + 0xff);
    Serial.print("Go Right: ");
    Serial.println(direction);
    set_direction();
    break;
  case 'n':
    direction = NEUTRAL;
    Serial.print("Go Straight: ");
    Serial.println(direction);
    set_direction();
    break;
  case 's':
    speed = 0;
    Serial.println("Full STOP!");
    full_stop();
    break;
  case ' ':
    reverse();
    Serial.print("Engage! Going ");
    Serial.println(forward ? "FORWARD" : "BACKWARDS");
  default:
    break;
  }
}

