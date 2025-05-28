// === Motor Driver Pins (DBH-12V) ===
const int leftMotorIn1  = 3;  // PWM pin for CW
const int leftMotorIn2  = 2;  // PWM pin for CCW
const int rightMotorIn1 = 5;  // PWM pin for CW
const int rightMotorIn2 = 4;  // PWM pin for CCW

// === Encoder Pins ===
const int leftEncoderPinA  = 12;
const int leftEncoderPinB  = 7;
const int rightEncoderPinA = 13;
const int rightEncoderPinB = 14;

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// === Ultrasonic Sensor Pins ===
const int trigPinA = 16;
const int echoPinA = 17;
const int trigPinB = 20;
const int echoPinB = 21;
const int trigPinC = 23;
const int echoPinC = 22;

// === Timing ===
unsigned long last_enc_send = 0;
const unsigned long SEND_INTERVAL = 100;  // ms
unsigned long last_cmd_time = 0;

// TESTING CHANGING 500 TO 100 TO MATCH TIMEOUT IN ROS
//const unsigned long CMD_TIMEOUT = 500;  // ms
const unsigned long CMD_TIMEOUT = 100;  // ms

int last_left_pwm = 0;
int last_right_pwm = 0;
bool motors_were_active = false;

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(leftMotorIn1, OUTPUT);
  pinMode(leftMotorIn2, OUTPUT);
  pinMode(rightMotorIn1, OUTPUT);
  pinMode(rightMotorIn2, OUTPUT);

  // Encoder setup
  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);
  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);

  // Ultrasonic pin setup
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  pinMode(trigPinC, OUTPUT);
  pinMode(echoPinC, INPUT);

  Serial.println("Teensy ready — motors, encoders, and ultrasonics.");
}

void loop() {
  // 1. Read velocity command from ROS
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      last_left_pwm  = cmd.substring(0, comma).toInt();
      last_right_pwm = cmd.substring(comma + 1).toInt();
      last_cmd_time = millis();
      motors_were_active = true;
    }
  }

  // 2. Timeout protection
  if (millis() - last_cmd_time > CMD_TIMEOUT) {
    if (motors_were_active) {
      Serial.println("Command timeout — stopping motors.");
      motors_were_active = false;
    }
    last_left_pwm = 0;
    last_right_pwm = 0;
  }

  // 3. Drive motors
  setMotor(last_left_pwm, leftMotorIn1, leftMotorIn2);
  setMotor(last_right_pwm, rightMotorIn1, rightMotorIn2);

  // 4. Send encoder and ultrasonic data to ROS
  if (millis() - last_enc_send >= SEND_INTERVAL) {
    last_enc_send = millis();

    Serial.print("Left Encoder:");
    Serial.print(left_ticks);
    Serial.print(" Right Encoder:");
    Serial.println(right_ticks);

    printUltrasonicData();  // New sensor line
  }
}

// === Motor control ===
void setMotor(int speed, int in1, int in2) {
  int pwm = constrain(abs(speed), 0, 255);

  if (speed == 0) {         //logic fix to ensure robot stops when no velocity commands are given
    analogWrite(in1, 0);
    digitalWrite(in1, LOW);
    analogWrite(in2, 0);
    digitalWrite(in2, LOW);
    
  }else if (speed > 0){
    analogWrite(in1, pwm);  // Forward
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwm);  // Reverse
  }
}

// === Encoder interrupt routines ===
void leftEncoderISR() {
  bool A = digitalRead(leftEncoderPinA);
  bool B = digitalRead(leftEncoderPinB);
  if (A == B) left_ticks++;
  else        left_ticks--;
}

void rightEncoderISR() {
  bool A = digitalRead(rightEncoderPinA);
  bool B = digitalRead(rightEncoderPinB);
  if (A == B) right_ticks++;
  else        right_ticks--;
}

// === Helper to get distance in cm from HC-SR04 ===
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 50000);  // timeout at 50ms
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;   //in cm
}

// === Send ultrasonic sensor readings ===
void printUltrasonicData() {
  float distA = getDistance(trigPinA, echoPinA);
  float distB = getDistance(trigPinB, echoPinB);
  float distC = getDistance(trigPinC, echoPinC);

  Serial.print("UltraA:");
  Serial.print(distA);
  Serial.print(" UltraB:");
  Serial.print(distB);
  Serial.print(" UltraC:");
  Serial.println(distC);
}
