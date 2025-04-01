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

void leftEncoderISR() { leftEncoderCount += leftMotorDirection; }
void rightEncoderISR() { rightEncoderCount += rightMotorDirection; }

void setup() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    pinMode(trigPinA, OUTPUT);
    pinMode(echoPinA, INPUT);
    pinMode(trigPinB, OUTPUT);
    pinMode(echoPinB, INPUT);

    pinMode(leftEncoderPin, INPUT);
    pinMode(rightEncoderPin, INPUT);

    attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

    Serial.begin(115200);
}

float getDistanceA() {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    long durationA = pulseIn(echoPinA, HIGH);
    return (durationA * 0.034) / 2;
}

float getDistanceB() {
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);
    long durationB = pulseIn(echoPinB, HIGH);
    return (durationB * 0.034) / 2;
}

void StopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, 0);
    analogWrite(enable2Pin, 0);

    // Reset motor directions
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

    // Update motor directions
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

    // Update motor directions
    leftMotorDirection = 1;
    rightMotorDirection = 1;
}

// Add missing function definitions
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

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Include motor direction in the encoder data
        Serial.print("A:");
        Serial.print(distanceA);
        Serial.print(",B:");
        Serial.print(distanceB);
        Serial.print(",Left Encoder:");
        Serial.print(leftEncoderCount);
        Serial.print(",Right Encoder:");
        Serial.println(rightEncoderCount);
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
