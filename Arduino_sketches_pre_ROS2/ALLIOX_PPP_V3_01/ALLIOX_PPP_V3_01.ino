// If boot mode error - change encoder pin from D2
// ESP32 Pin Locations - Motors 1 and 2
int motor1Pin1 = 33; 
int motor1Pin2 = 32; 
int enable1Pin = 4; 
int motor2Pin1 = 26; 
int motor2Pin2 = 25; 
int enable2Pin = 5; 

// ESP32 Pin Locations - Ultrasonic Sensor
const int trigPinA = 12;
const int echoPinA = 13;
const int trigPinB = 19;
const int echoPinB = 18;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int speed = 200;

// Motor Functions (called later)

// Stop
void ReverseMotors() {

    Serial.println("Motor stopped");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    
}

// Back Up
void StopMotors() {

    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);  
  
}

// Turn Right
void TurnRight() {

    Serial.println("Turning Right");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);  
  
}

// Turn Left
void TurnLeft() {

    Serial.println("Turning Left");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);  
  
}

// Turn Right Slightly while
void MinorTurnRight() {

    Serial.println("Turning Right");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);  
  
}

// Turn Left Slightly while moving
void MinorTurnLeft() {

    Serial.println("Turning Left");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed);  
  
}

// Proceed Forward
void ForwardMotors() {

    Serial.println("Moving Forward");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);   
    ledcWrite(enable2Pin, speed); 
  
}

void setup() {
  
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

//Ultrasonic
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);

//pwm
  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  ledcAttachChannel(enable2Pin, freq, resolution, pwmChannel);


  Serial.begin(115200);


}

float getDistanceA() {
  digitalWrite(trigPinA, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinA, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinA, LOW);

  // Calculate distance
  long durationA = pulseIn(echoPinA, HIGH);
  float distanceA = (durationA * 0.034) / 2; // Convert to cm
  return distanceA;
}

float getDistanceB() {
  digitalWrite(trigPinB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinB, LOW);

  // Calculate distance
  long durationB = pulseIn(echoPinB, HIGH);
  float distanceB = (durationB * 0.034) / 2; // Convert to cm
  return distanceB;
}

// Driving Program

void loop() {
  float distanceA = getDistanceA();
  float distanceB = getDistanceB();
  Serial.println("Right Ultrasonic Distance:");
  Serial.println(distanceA);
   Serial.println("Left Ultrasonic Distance:");
  Serial.println(distanceB);

if ((distanceA > 0 && distanceA < 22) || (distanceB > 0 && distanceB < 22)) {
    Serial.println("Obstacle (Stop and Turn)");
    
    StopMotors();
    delay(300);
    ReverseMotors();
    delay(300);

    // Decide turn direction
    if (distanceA < distanceB) {
        TurnLeft();
    } else {
        TurnRight();
    }
    delay(300);
} 
else if ((distanceA >= 22 && distanceA < 30) || (distanceB >= 22 && distanceB < 30)) {
    Serial.println("Obstacle (Adjust Direction)");
    
    // Decide turn direction but don't stop
    if (distanceA < distanceB) {
        MinorTurnLeft();
    } else {
        MinorTurnRight();
    }
    delay(300); // Adjust direction while moving
    
    ForwardMotors(); // Continue forward without stopping
} 
else {
    // No obstacle, move forward
    ForwardMotors();
    delay(300);
}



}
