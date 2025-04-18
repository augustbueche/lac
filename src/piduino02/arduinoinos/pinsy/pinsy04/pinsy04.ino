#include <Wire.h>
#include <MPU6050_tockn.h>
#include <QMC5883LCompass.h>  // MPrograms library

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

// Encoder interrupt service routines
void leftEncoderISR() { leftEncoderCount += leftMotorDirection; }
void rightEncoderISR() { rightEncoderCount += rightMotorDirection; }

// Calibration parameters from your min/max test for the compass axes
// For X: Min = -4660, Max = -4188  -> Center = -4424
// For Y: Min = -4123, Max = -3620  -> Center ≈ -3871.5
const float centerX = -4424.0;
const float centerY = -3871.5;

void setup() {
    // Setup motor pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    // Setup ultrasonic sensor pins
    pinMode(trigPinA, OUTPUT);
    pinMode(echoPinA, INPUT);
    pinMode(trigPinB, OUTPUT);
    pinMode(echoPinB, INPUT);

    // Setup encoder pins
    pinMode(leftEncoderPin, INPUT);
    pinMode(rightEncoderPin, INPUT);

    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

    // Start Serial communication
    Serial.begin(115200);
    Wire.begin();  // Uses default I2C pins (18=SDA, 19=SCL on Teensy 4.1)

    // Initialize MPU6050
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // Initialize compass and set your calibration parameters
    compass.init();  // Required for QMC5883LCompass (MPrograms)
    compass.setCalibrationOffsets(-2228.00, -2173.00, -339.00);
    compass.setCalibrationScales(0.71, 0.73, 4.66);
}

float getDistanceA() {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    long durationA = pulseIn(echoPinA, HIGH);
    return (durationA * 0.034) / 2;  // cm
}

float getDistanceB() {
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);
    long durationB = pulseIn(echoPinB, HIGH);
    return (durationB * 0.034) / 2;  // cm
}

void StopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, 0);
    analogWrite(enable2Pin, 0);
    leftMotorDirection = 0;
    rightMotorDirection = 0;
}

void ReverseMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
    leftMotorDirection = -1;
    rightMotorDirection = -1;
}

void ForwardMotors() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
    leftMotorDirection = 1;
    rightMotorDirection = 1;
}

void TurnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
}

void TurnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
}

void MinorTurnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
}

void MinorTurnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, speed);
    analogWrite(enable2Pin, speed);
}

void loop() {
    // Get ultrasonic distances
    float distanceA = getDistanceA();
    float distanceB = getDistanceB();

    // Update MPU6050 sensor data
    mpu6050.update();

    // Read compass raw calibrated values (after internal calibration)
    compass.read();
    float rawX = compass.getX();
    float rawY = compass.getY();
    
    // Calibrate the raw values by subtracting the center
    float calX = rawX - centerX;
    float calY = rawY - centerY;
    
    // Compute heading using atan2 with the calibrated values:
    // Using: heading = atan2(calX, calY) * 180/PI yields 0° when calX is ~0 and calY positive.
    float heading = atan2(calX, calY) * 180.0 / PI;
    if (heading < 0)
        heading += 360;
        
    // Now offset by 180 degrees
    heading += 180;
    if (heading >= 360)
        heading -= 360;
        
    int compassHeading = (int)(heading + 0.5);  // Rounded integer value (0-359)

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.print("A:"); Serial.print(distanceA);
        Serial.print(",B:"); Serial.print(distanceB);
        Serial.print(",Left Encoder:"); Serial.print(leftEncoderCount);
        Serial.print(",Right Encoder:"); Serial.print(rightEncoderCount);
        Serial.print(",MPU AccX:"); Serial.print(mpu6050.getAccX());
        Serial.print(",MPU AccY:"); Serial.print(mpu6050.getAccY());
        Serial.print(",MPU AccZ:"); Serial.print(mpu6050.getAccZ());
        Serial.print(",MPU GyroX:"); Serial.print(mpu6050.getGyroX());
        Serial.print(",MPU GyroY:"); Serial.print(mpu6050.getGyroY());
        Serial.print(",MPU GyroZ:"); Serial.print(mpu6050.getGyroZ());
        Serial.print(",Compass:"); Serial.println(compassHeading);
    }

    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'S': StopMotors(); break;
            case 'B': ReverseMotors(); break;
            case 'L': TurnLeft(); break;
            case 'R': TurnRight(); break;
            case 'F': ForwardMotors(); break;
            case 'm': MinorTurnLeft(); break;
            case 'n': MinorTurnRight(); break;
        }
    }
}
