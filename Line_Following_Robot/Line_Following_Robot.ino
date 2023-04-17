#define IR_SENSOR_LEFT_L 8
#define IR_SENSOR_LEFT_R 9
#define IR_SENSOR_RIGHT_L 10
#define IR_SENSOR_RIGHT_R 11


#define MOTOR_SPEED 120

//Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 3;
int rightMotorPin2 = 4;

//Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 2;
int leftMotorPin2 = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT_R, INPUT);
  pinMode(IR_SENSOR_LEFT_R, INPUT);
  pinMode(IR_SENSOR_RIGHT_L, INPUT);
  pinMode(IR_SENSOR_LEFT_L, INPUT);
  rotateMotor(0, 0);
  Serial.begin(9600);
}


void loop() {
  // Read the status of the four sensors
  int leftIRSensorValue_R = digitalRead(IR_SENSOR_LEFT_R);
  int rightIRSensorValue_L = digitalRead(IR_SENSOR_RIGHT_L);
  int leftIRSensorValue_L = digitalRead(IR_SENSOR_LEFT_L);
  int rightIRSensorValue_R = digitalRead(IR_SENSOR_RIGHT_R);

  // If both of the middle sensors detcts black line, go straight
  if(rightIRSensorValue_L == 1 && leftIRSensorValue_R == 1) {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }

  //If left most sensor or if only left middle sensor detects black line, then turn left 
  else if (rightIRSensorValue_L == LOW && leftIRSensorValue_R == HIGH) {
    turnLeft();
  }
  //If right most sensor or if only right middle sensor detects black line, then turn right
  else if (rightIRSensorValue_L == HIGH && leftIRSensorValue_R == LOW) {
    turnRight();
  }
  //If none of the sensors detect black line, then stop
  else if (rightIRSensorValue_L == LOW && leftIRSensorValue_R == LOW) {
    stop();
  }
  else{
    stop();
  }
}

// Function to rotate the motors
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {

  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

// Turn car left direction
void turnLeft(){
  rotateMotor(0, MOTOR_SPEED);
}

// Turn car right direction
void turnRight(){
  rotateMotor(MOTOR_SPEED, 0);
}

// Move car forward
void moveForward(){
  rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
}

// Stop car
void stop(){
  rotateMotor(0, 0);
}