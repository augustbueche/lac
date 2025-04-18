#include <Wire.h>
#include <MPU6050_tockn.h>
#include <QMC5883LCompass.h>  // MPrograms library
#include <math.h>

// Teensy 4.1 Pin Locations - Motors 1 and 2
int motor1Pin1 = 2;
int motor1Pin2 = 3;
int enable1Pin = 4;
int motor2Pin1 = 5;
int motor2Pin2 = 6;
int enable2Pin = 7;

// Ultrasonic Sensors
const int trigPinA = 8;
const int echoPinA = 9;
const int trigPinB = 10;
const int echoPinB = 11;

// IR Encoders
const int leftEncoderPin = 12;
const int rightEncoderPin = 13;

// Sensor objects
MPU6050 mpu6050(Wire);
QMC5883LCompass compass;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

int leftMotorDirection = 1;   // 1 = forward, -1 = backward
int rightMotorDirection = 1;
int speed = 300;  // PWM speed

unsigned long previousMillis = 0;
const long interval = 300;  // interval in ms

// Encoder interrupts
void leftEncoderISR()  { leftEncoderCount  += leftMotorDirection; }
void rightEncoderISR() { rightEncoderCount += rightMotorDirection; }

// Calibration centers from your latest min/max test
// X: min –4660, max –4188 → center = –4424
// Y: min –4123, max –3620 → center ≈ –3871.5
const float centerX = -4424.0;
const float centerY = -3871.5;

void setup() {
    // Motors
    pinMode(motor1Pin1, OUTPUT); pinMode(motor1Pin2, OUTPUT); pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT); pinMode(motor2Pin2, OUTPUT); pinMode(enable2Pin, OUTPUT);

    // Ultrasonics
    pinMode(trigPinA, OUTPUT); pinMode(echoPinA, INPUT);
    pinMode(trigPinB, OUTPUT); pinMode(echoPinB, INPUT);

    // Encoders
    pinMode(leftEncoderPin, INPUT);
    pinMode(rightEncoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(leftEncoderPin),  leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

    // Serial & I2C
    Serial.begin(115200);
    Wire.begin();  // Teensy default SDA=18, SCL=19

    // MPU6050
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // Compass
    compass.init();
    compass.setCalibrationOffsets(-2228.00, -2173.00, -339.00);
    compass.setCalibrationScales(   0.71,    0.73,    4.66);
}

float getDistanceA() {
    digitalWrite(trigPinA, LOW);  delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH); delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    long d = pulseIn(echoPinA, HIGH);
    return (d * 0.034) / 2;
}

float getDistanceB() {
    digitalWrite(trigPinB, LOW);  delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH); delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);
    long d = pulseIn(echoPinB, HIGH);
    return (d * 0.034) / 2;
}

void StopMotors() {
    digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, 0);     analogWrite(enable2Pin, 0);
    leftMotorDirection  = 0;
    rightMotorDirection = 0;
}

void ReverseMotors() {
    digitalWrite(motor1Pin1, LOW);  digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);  digitalWrite(motor2Pin2, HIGH);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
    leftMotorDirection  = -1;
    rightMotorDirection = -1;
}

void ForwardMotors() {
    digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
    leftMotorDirection  = 1;
    rightMotorDirection = 1;
}

void TurnLeft() {
    digitalWrite(motor1Pin1, LOW);  digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
}

void TurnRight() {
    digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);  digitalWrite(motor2Pin2, HIGH);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
}

void MinorTurnLeft() {
    digitalWrite(motor1Pin1, LOW);  digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
}

void MinorTurnRight() {
    digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);  digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed); analogWrite(enable2Pin, speed);
}

void loop() {
    // Read sensors
    float distanceA = getDistanceA();
    float distanceB = getDistanceB();
    mpu6050.update();
    compass.read();

    // Zero‑centered compass axes
    float rawX = compass.getX() - centerX;
    float rawY = compass.getY() - centerY;

    // Compute raw heading (0° when +Y axis points north)
    float rad = atan2(rawX, rawY);
    float deg = rad * 180.0 / PI;
    if (deg < 0) deg += 360;

    // Apply 180° offset, wrap into [0,360)
    deg = fmod(deg + 180.0, 360.0);

    int compassHeading = int(deg + 0.5);  // round

    // Serial output
    unsigned long now = millis();
    if (now - previousMillis >= interval) {
        previousMillis = now;

        Serial.print("A:"); Serial.print(distanceA);
        Serial.print(",B:"); Serial.print(distanceB);
        Serial.print(",Left Encoder:");  Serial.print(leftEncoderCount);
        Serial.print(",Right Encoder:"); Serial.print(rightEncoderCount);

        Serial.print(",MPU AccX:");  Serial.print(mpu6050.getAccX());
        Serial.print(",MPU AccY:");  Serial.print(mpu6050.getAccY());
        Serial.print(",MPU AccZ:");  Serial.print(mpu6050.getAccZ());
        Serial.print(",MPU GyroX:"); Serial.print(mpu6050.getGyroX());
        Serial.print(",MPU GyroY:"); Serial.print(mpu6050.getGyroY());
        Serial.print(",MPU GyroZ:"); Serial.print(mpu6050.getGyroZ());

        Serial.print(",Compass:"); Serial.println(compassHeading);
    }

    // Handle serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'S': StopMotors();     break;
            case 'B': ReverseMotors();  break;
            case 'L': TurnLeft();       break;
            case 'R': TurnRight();      break;
            case 'F': ForwardMotors();  break;
            case 'm': MinorTurnLeft();  break;
            case 'n': MinorTurnRight(); break;
        }
    }
}
