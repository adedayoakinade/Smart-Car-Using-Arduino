#define MAX_REGULAR_MOTOR_SPEED 120
#define MAX_MOTOR_ADJUST_SPEED 150


//Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 3;
int rightMotorPin2 = 4;

//Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 2;
int leftMotorPin2 = 7;

int t;

void setup() {
  // put your setup code here, to run once:
  // Configure the necessary pins as output and input
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  Serial.begin(9600);

  rotateMotor(0, 0);
}



void loop() {
  //Read the lsignal from the bluetooth module
  if (Serial.available()) {
    t = Serial.read();
    Serial.println(t);
  }

  if (t == 49) {  //move forward(all motors rotate in forward direction)
    rotateMotor(MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
  }

  else if (t == 50) {  //move reverse (all motors rotate in reverse direction)
    rotateMotor(-MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
  }

  else if (t == 51) {  //turn right (left side motors rotate in forward direction, right side motors doesn't rotate)
    rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
  }

  else if (t == 52) {  //turn left (right side motors rotate in forward direction, left side motors doesn't rotate)
    rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
  }

  else if (t == 53) {  //STOP (all motors stop)
    rotateMotor(0, 0);
  }
  delay(100);
}

//Function to rotate the motors in the appropriate direction
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