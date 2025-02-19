#include <Wire.h>

#define pi 3.14159265358979323846264338327950288
#define MOTOR_COUNT 2
#define MY_ADDR 8

// RIGHT IS 0
// LEFT IS 1

// Motor pin configuration
const int motorDirPins[MOTOR_COUNT] = {7, 8};
const int motorPwmPins[MOTOR_COUNT] = {9, 10};
const int motorEncoderA[MOTOR_COUNT] = {3, 2};
const int motorEncoderB[MOTOR_COUNT] = {6, 5};
// enables motor driver
int en = 4;

// velocity control variables
float desiredSpeed[MOTOR_COUNT] = {0.0, 0.0};
float motorVoltage[MOTOR_COUNT] = {0.0, 0.0};
float velocityError[MOTOR_COUNT] = {0.0, 0.0};
unsigned int motorPWM[MOTOR_COUNT] = {0, 0};
volatile float motorRad[MOTOR_COUNT] = {0.0, 0.0};
float motorVelocity[MOTOR_COUNT] = {0.0, 0.0};
float motorPriorPos[MOTOR_COUNT] = {0.0, 0.0};
// parameters for velocity control
float batteryVoltage = 7.0;
float kp = 3.5;

// position control variables
float radError[MOTOR_COUNT] = {0.0, 0.0};
float desiredRad[MOTOR_COUNT] = {0.0, 0.0};
float integralError[MOTOR_COUNT] = {0.0, 0.0};
float kpPos = 37.5;
float kiPos = 19;
float maxSpeed = 6;

// for input from the pi
int posIn[2] = {0, 0};

// Time variables
long lastTimeMs;
long startTimeMs;
long desiredTsMs = 10;
float vDelta = 0;

// Position tracking
float xpos = 0;
float ypos = 0;
float phi = 0;

void setup() {
  // Initialize pins and interrupts
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motorDirPins[i], OUTPUT);
    pinMode(motorPwmPins[i], OUTPUT);
    pinMode(motorEncoderA[i], INPUT_PULLUP);
    pinMode(motorEncoderB[i], INPUT_PULLUP);
  }

  // enables motor driver
  pinMode(en, OUTPUT);
  digitalWrite(en, HIGH);

  // interupts for encoders
  attachInterrupt(digitalPinToInterrupt(motorEncoderA[0]), []() { readEncoder(0); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorEncoderA[1]), []() { readEncoder(1); }, CHANGE);

  // initalizes serial and time
  Serial.begin(115200);
  startTimeMs = millis();
  lastTimeMs = startTimeMs;
  Serial.println("Ready!");
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

  // for taking data from pi
}

void loop() {



  // decides time between loops
  while (millis() < lastTimeMs + desiredTsMs) {
    // Wait until the desired time passes
  }
  lastTimeMs = millis();


  // output data to serial for debugging
  printData();

  // calculates position and velocity
  calcPosAndVelocity();

  // control position
  positionControl();
}


// this takes in what will be from the pi. 
void receive() {
  int offset = Wire.read();
  while(Wire.available()) {
    int incomingByte = Wire.read();

    // Check if the input is within valid range (0-3)
    if (incomingByte >= 0 && incomingByte <= 3) {
        // Update the posIn array based on the binary representation of the incomingByte
        desiredRad[1] = pi * ((incomingByte >> 1) & 1);  // Extract the first bit
        desiredRad[0] = pi * (incomingByte & 1);         // Extract the second bit
  }
  }

}



void printData() {
  Serial.print(float(millis()) / 1000.0);  // Print current time in seconds
  Serial.print("\t");
  Serial.print(motorVoltage[1]);  // Right motor voltage
  Serial.print("\t");
  Serial.print(motorVoltage[0]);  // Left motor voltage
  Serial.print("\t");
  Serial.print(motorVelocity[1], 4);  // Right motor velocity (4 decimal places)
  Serial.print("\t");
  Serial.print(motorVelocity[0], 4);  // Left motor velocity (4 decimal places)
  Serial.print("\t");
  Serial.print(motorRad[1], 4);  // Right motor velocity (4 decimal places)
  Serial.print("\t");
  Serial.print(motorRad[0], 4);  // Left motor velocity (4 decimal places)
  Serial.println();
}

// controls the position of the motors,
// calculates errors, desired speed, then calls velocity control to set that speed
void positionControl() {
  for(int i=0;i<2;i++) {
    radError[i] = desiredRad[i] - motorRad[i];
    integralError[i] = integralError[i] + radError[i]*((float)desiredTsMs /1000);
    desiredSpeed[i] = (kpPos * radError[i] + kiPos * integralError[i]);
    velocityControl();
}
}

// calculates velocity errors and uses them to set the motor pwm
// calls setMotor to set the actual pwm of the motor
void velocityControl() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    velocityError[i] = desiredSpeed[i] - motorVelocity[i];
    motorVoltage[i] = kp * velocityError[i];
    motorPWM[i] = 255 * abs(motorVoltage[i]) / batteryVoltage;
  }
  setMotor();
}

// sets the motors accorditing to the desired motorVoltageS_
void setMotor() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    digitalWrite(motorDirPins[i], motorVoltage[i] < 0 ? LOW : HIGH);
    analogWrite(motorPwmPins[i], min(motorPWM[i], 255));
  }
}

void calcPosAndVelocity() {
  // Calculate motor velocities
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorVelocity[i] = (motorRad[i] - motorPriorPos[i]) / ((millis() - vDelta) / 1000.0);
    motorPriorPos[i] = motorRad[i];
  }
  vDelta = millis();

  // calculate postion
  float deltaLeft = motorRad[0] - motorPriorPos[0];
  float deltaRight = motorRad[1] - motorPriorPos[1];
  xpos += cos(phi) * (deltaLeft + deltaRight) / 2;
  ypos += sin(phi) * (deltaLeft + deltaRight) / 2;
  phi += (1 / 0.34) * (deltaLeft - deltaRight);
}

void readEncoder(int motorIndex) {
  // Read current encoder states
  int thisA = digitalRead(motorEncoderA[motorIndex]);
  int thisB = digitalRead(motorEncoderB[motorIndex]);

  // Ensure this and last states are tracked per motor
  static int lastA[MOTOR_COUNT] = {0};
  static int lastB[MOTOR_COUNT] = {0};
  static int count[MOTOR_COUNT] = {0};

  // Check for any change in encoder state
  if (thisA != lastA[motorIndex] || thisB != lastB[motorIndex]) {
    int lastState = (lastA[motorIndex] << 1) | lastB[motorIndex];
    int currentState = (thisA << 1) | thisB;

    // Positive transitions (clockwise direction)
    if ((lastState == 0 && currentState == 1) || (lastState == 1 && currentState == 3) ||
        (lastState == 2 && currentState == 0) || (lastState == 3 && currentState == 2)) {
      count[motorIndex]++;
    }
    // Double positive steps (if there's noise or fast transitions)
    else if ((lastState == 0 && currentState == 3) || (lastState == 3 && currentState == 0)) {
      count[motorIndex] += 2;
    }
    else if ((lastState == 2 && currentState == 1) || (lastState == 1 && currentState == 2)) {
      count[motorIndex] -= 2;
    }
    // Negative transitions (counterclockwise direction)
    else {
      count[motorIndex]--;
    }

    // Save current states for the next comparison
    lastA[motorIndex] = thisA;
    lastB[motorIndex] = thisB;

    // Update the motor's angular position in radians
    motorRad[motorIndex] = 2 * pi * (float)count[motorIndex] / 3200;
  }
}

