#include <Servo.h>
#include <NewPing.h>

#define SERVO_PIN A0
#define ULTRASONIC_SENSOR_TRIG 12
#define ULTRASONIC_SENSOR_ECHO 13
#define MAX_REGULAR_MOTOR_SPEED 120
#define MAX_MOTOR_ADJUST_SPEED 150
#define DISTANCE_TO_CHECK 30

int distance = 0;

//Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 3;
int rightMotorPin2 = 4;

//Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 2;
int leftMotorPin2 = 7;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Attach servo to pin A0
  myServo.attach(A0);
  delay(3000);
  myServo.write(100);
  
  rotateMotor(0, 0);
}

void loop() {

  distance = mySensor.ping_cm();
  // distance = 50;

  //If distance is within 30 cm then adjust motor direction as below
  if (distance > 0 && distance < DISTANCE_TO_CHECK) {
    //Stop motors
    rotateMotor(0, 0);
    delay(500);

    //Rotate servo to left
    myServo.write(180);
    delay(500);

    //Read left side distance using ultrasonic sensor
    int distanceLeft = mySensor.ping_cm();

    //Rotate servo to right
    myServo.write(0);
    delay(500);

    //Read right side distance using ultrasonic sensor
    int distanceRight = mySensor.ping_cm();

    //Bring servo to center
    myServo.write(100);
    delay(500);

    // if no space in the left, turn right
    if (distanceLeft == 0) {
      rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
      delay(700);
    } 
    // if no space in the right, turn left
    else if (distanceRight == 0) {
      rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
      delay(700);
    } 
    // If space on left is more than right, turn left
    else if (distanceLeft >= distanceRight) {
      rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
      delay(700);
    } 
    // If space on right is more than left, turn right
    else {
      rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
      delay(700);
    }
    rotateMotor(0, 0);
    delay(200);
  } 
  // Keep moving forward
  else {
    rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
    delay(200);
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed >= 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed >= 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}