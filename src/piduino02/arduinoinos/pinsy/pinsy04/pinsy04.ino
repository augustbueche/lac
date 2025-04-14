#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SparkFun_QMC5883L.h>  // Library for the QMC5883L compass

// Teensy 4.1 Pin Locations - Motors 1 and 2
int motor1Pin1 = 2;
int motor1Pin2 = 3;
int enable1Pin = 4;
int motor2Pin1 = 5;
int motor2Pin2 = 6;
int enable2Pin = 7;

// Teensy 4.1 Pin Locations - Ultrasonic Sensors
const int trigPinA = 8;
const int echoPinA = 9;
const int trigPinB = 10;
const int echoPinB = 11;

// Teensy 4.1 Pin Locations - IR Encoders
const int leftEncoderPin = 12;
const int rightEncoderPin = 13;

// Teensy 4.1 Pin Locations - MPU6050 (I2C)
const int mpuSdaPin = 18; // SDA
const int mpuSclPin = 19; // SCL

// Compass (QMC5883L) uses the same I2C bus (SDA & SCL)
  // VCC -> 3.3V, GND -> GND, SDA -> 18, SCL -> 19, INT (optional) -> not connected

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Motor direction variables
int leftMotorDirection = 1;  // 1 for forward, -1 for backward
int rightMotorDirection = 1;

// PWM speed
int speed = 300;

// Timer for non-blocking serial output
unsigned long previousMillis = 0;
const long interval = 300; // 300ms

// Sensor objects
MPU6050 mpu6050(Wire);
QMC5883L compass;  // Compass sensor object

// Interrupt Service Routines for the encoders
void leftEncoderISR() { leftEncoderCount += leftMotorDirection; }
void rightEncoderISR() { rightEncoderCount += rightMotorDirection; }

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

    // Initialize I2C on designated pins (used by both MPU6050 and compass)
    Wire.begin(mpuSdaPin, mpuSclPin);

    // Initialize MPU6050
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    
    // Initialize Compass
    if (!compass.begin()) {
        Serial.println("Compass not detected!");
        while(1); // Hang if the compass isn't found
    }
}

float getDistanceA() {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    long durationA = pulseIn(echoPinA, HIGH);
    return (durationA * 0.034) / 2; // distance in cm
}

float getDistanceB() {
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);
    long durationB = pulseIn(echoPinB, HIGH);
    return (durationB * 0.034) / 2; // distance in cm
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
    float distanceA = getDistanceA();
    float distanceB = getDistanceB();

    // Update MPU6050 data
    mpu6050.update();

    // Read compass data via I2C
    compass.read();
    float compHeading = compass.getHeading(); // Heading in degrees

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.print("A:");
        Serial.print(distanceA);
        Serial.print(",B:");
        Serial.print(distanceB);
        Serial.print(",Left Encoder:");
        Serial.print(leftEncoderCount);
        Serial.print(",Right Encoder:");
        Serial.print(rightEncoderCount);
        Serial.print(",MPU AccX:");
        Serial.print(mpu6050.getAccX());
        Serial.print(",MPU AccY:");
        Serial.print(mpu6050.getAccY());
        Serial.print(",MPU AccZ:");
        Serial.print(mpu6050.getAccZ());
        Serial.print(",MPU GyroX:");
        Serial.print(mpu6050.getGyroX());
        Serial.print(",MPU GyroY:");
        Serial.print(mpu6050.getGyroY());
        Serial.print(",MPU GyroZ:");
        Serial.print(mpu6050.getGyroZ());
        Serial.print(",Compass:");
        Serial.println(compHeading);
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
