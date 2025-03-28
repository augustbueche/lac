// ESP32 Pin Locations - Motors 1 and 2
int motor1Pin1 = 33; 
int motor1Pin2 = 32; 
int enable1Pin = 4; 
int motor2Pin1 = 26; 
int motor2Pin2 = 25; 
int enable2Pin = 5; 

// ESP32 Pin Locations - Ultrasonic Sensors
const int trigPinA = 12;
const int echoPinA = 13;
const int trigPinB = 19;
const int echoPinB = 18;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int speed = 200;

// Timer for non-blocking serial output
unsigned long previousMillis = 0;
const long interval = 300; // 300ms between serial messages

// Function: Stop Motors
void StopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
}


// Function: Move Backward
void ReverseMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Function: Turn Right
void TurnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Function: Turn Left
void TurnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Function: Slight Right Turn
void MinorTurnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Function: Slight Left Turn
void MinorTurnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Function: Move Forward
void ForwardMotors() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);
}

// Setup Function
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

    ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
    ledcAttachChannel(enable2Pin, freq, resolution, pwmChannel);

    Serial.begin(115200);
}

// Function: Get Distance from Ultrasonic Sensor A
float getDistanceA() {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);
    long durationA = pulseIn(echoPinA, HIGH);
    return (durationA * 0.034) / 2; // Convert to cm
}

// Function: Get Distance from Ultrasonic Sensor B
float getDistanceB() {
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);
    long durationB = pulseIn(echoPinB, HIGH);
    return (durationB * 0.034) / 2; // Convert to cm
}

// Main Loop
void loop() {
    float distanceA = getDistanceA();
    float distanceB = getDistanceB();

    // Motor control logic
    if ((distanceA > 0 && distanceA < 22) || (distanceB > 0 && distanceB < 22)) {
        StopMotors();
        ReverseMotors();
        if (distanceA < distanceB) {
            TurnLeft();
        } else {
            TurnRight();
        }
    } 
    else if ((distanceA >= 22 && distanceA < 30) || (distanceB >= 22 && distanceB < 30)) {
        if (distanceA < distanceB) {
            MinorTurnLeft();
        } else {
            MinorTurnRight();
        }
        ForwardMotors();
    } 
    else {
        ForwardMotors();
    }
    

    // Non-blocking Serial Transmission
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.print("A:");
        Serial.print(distanceA);
        Serial.print(",B:");
        Serial.println(distanceB);
    }
}
 