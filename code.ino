#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define MOTOR_SPEED 180
#define OBSTACLE_DISTANCE 5

// Right motor
int enableRightMotor = 6;
int rightMotorPin1   = 7;
int rightMotorPin2   = 8;

// Left motor
int enableLeftMotor  = 5;
int leftMotorPin1    = 9;
int leftMotorPin2    = 10;

// Ultrasonic sensor
#define TRIG_PIN 13
#define ECHO_PIN 4

// HW067 Color Sensor (Updated Pins)
#define S0        A0
#define S1        A1
#define S2        A2
#define S3        A3
#define SENSOR_OUT A4

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor,  OUTPUT);
  pinMode(leftMotorPin1,    OUTPUT);
  pinMode(leftMotorPin2,    OUTPUT);

  // Line‐following IR sensors
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT,  INPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  digitalWrite(S0, HIGH);  // Set frequency scaling for color sensor
  digitalWrite(S1, LOW);

  // Ensure motors are stopped
  rotateMotor(0, 0);
}

void loop() {
  int rightIR = digitalRead(IR_SENSOR_RIGHT);
  int leftIR  = digitalRead(IR_SENSOR_LEFT);
  long distance    = getDistance();
  int  detectedColor = detectColor();

  // Obstacle avoidance
  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    Serial.println("Obstacle too close—stopping.");
    rotateMotor(0, 0);
    return;
  }

  // Color‐based speed control
  if (detectedColor == 0) {
    // Red → stop
    Serial.println("Red detected—stop");
    rotateMotor(0, 0);  
    return;
  } else if (detectedColor == 1) {
    // Blue → slow
    Serial.println("Blue detected—slow down");
    rotateMotor(MOTOR_SPEED/2, MOTOR_SPEED/2);
  } else if (detectedColor == 2) {
    // Green → speed up
    Serial.println("Green detected—speed up");
    rotateMotor(MOTOR_SPEED+50, MOTOR_SPEED+50);
  } // else -1: no valid color, continue line logic

  // Line following logic
  if (leftIR == HIGH && rightIR == HIGH) {
    Serial.println("Both sensors see line—forward");
    rotateMotor(-MOTOR_SPEED, -MOTOR_SPEED);
  }
  else if (leftIR == HIGH && rightIR == LOW) {
    Serial.println("Left sees line—turn LEFT");
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (leftIR == LOW && rightIR == HIGH) {
    Serial.println("Right sees line—turn RIGHT");
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  else {
    Serial.println("No line—stop");
    rotateMotor(0, 0);
  }
}

void rotateMotor(int rightSpeed, int leftSpeed) {
  // Right motor direction
  if (rightSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // Left motor direction
  if (leftSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  // Apply speed
  analogWrite(enableRightMotor, abs(rightSpeed));
  analogWrite(enableLeftMotor,  abs(leftSpeed));
}

// Non‐blocking ultrasonic with timeout
long getDistance() {
  // Trigger a 10µs pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo up to 25 ms
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) {
    // No echo → assume no obstacle / very far
    return 999;
  }
  long dist = duration * 0.034 / 2;
  Serial.print("Ultrasonic: "); Serial.print(dist); Serial.println(" cm");
  return dist;
}

// Color detection with per‐channel timeouts
int detectColor() {
  const unsigned long timeout = 20000UL; // 20 ms max per read
  int red, green, blue;

  // Red
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  red = pulseIn(SENSOR_OUT, LOW, timeout);

  // Green
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  green = pulseIn(SENSOR_OUT, LOW, timeout);

  // Blue
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blue = pulseIn(SENSOR_OUT, LOW, timeout);

  // If any channel timed out, skip
  if (red == 0 || green == 0 || blue == 0) {
    Serial.println("Color read timeout");
    return -1;
  }

  float sum = float(red + green + blue);
  float rRatio = red   / sum;
  float bRatio = blue  / sum;

  Serial.print("Ratios—R: "); Serial.print(rRatio, 2);
  Serial.print("  B: "); Serial.println(bRatio, 2);

  if (rRatio > 0.5) {
    return 0;  // Red
  } else if (bRatio > 0.3) {
    return 1;  // Blue
  } else {
    return 2;  // Green (or default)
  }
}