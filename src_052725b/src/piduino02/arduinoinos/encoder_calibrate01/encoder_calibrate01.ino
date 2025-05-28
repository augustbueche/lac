// Teensy 4.1 Pin Locations - Motors 1 and 2
int motor1Pin1 = 2;
int motor1Pin2 = 3;
int enable1Pin = 4;
int motor2Pin1 = 5;
int motor2Pin2 = 6;
int enable2Pin = 7;

// Teensy 4.1 Pin Locations - IR Encoders
const int leftEncoderPin = 12;
const int rightEncoderPin = 13;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Motor speed (base speed for calibration)
int baseSpeed = 460;

// Correction factor for the right motor
float correctionFactor = 0.91;  

void leftEncoderISR() { leftEncoderCount++; }
void rightEncoderISR() { rightEncoderCount++; }

void setup() {
    // Motor pins setup
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    // Encoder pins setup
    pinMode(leftEncoderPin, INPUT);
    pinMode(rightEncoderPin, INPUT);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

    // Start serial communication
    Serial.begin(115200);

    // Initialize encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    Serial.println("Starting encoder calibration with correction factor...");
}

void loop() {
    // Drive motors forward with correction factor applied
    Serial.println("Driving motors forward...");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);

    // Apply correction factor to the right motor's speed
    analogWrite(enable1Pin, baseSpeed);  // Left motor at base speed
    analogWrite(enable2Pin, baseSpeed * correctionFactor);  // Right motor adjusted

    // Wait for 5 seconds
    delay(5000);

    // Stop motors
    Serial.println("Stopping motors...");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, 0);
    analogWrite(enable2Pin, 0);

    // Print encoder counts
    Serial.print("Left Encoder Count: ");
    Serial.println(leftEncoderCount);
    Serial.print("Right Encoder Count: ");
    Serial.println(rightEncoderCount);

    // Halt the program
    while (true) {
        // Do nothing
    }
}
