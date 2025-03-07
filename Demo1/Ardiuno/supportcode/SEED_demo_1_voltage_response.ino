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
float desiredMotorVoltage[MOTOR_COUNT] = {0.0, 0.0};
float motorVoltage[MOTOR_COUNT] = {0.0, 0.0};
unsigned int motorPWM[MOTOR_COUNT] = {0, 0};
volatile float motorRad[MOTOR_COUNT] = {0.0, 0.0};
float motorVelocity[MOTOR_COUNT] = {0.0, 0.0};
float motorPriorPos[MOTOR_COUNT] = {0.0, 0.0};
float batteryVoltage = 7.0;
float totalVelocity = 0.0;
float totalAngle = 0.0;


// Time variables
long lastTimeMs;
long startTimeMs;
long desiredTsMs = 10;
float vDelta = 0;


// Vechile dynamics variables
float sumVoltage = 0;
float difVoltage = 0;




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



  // for taking data from pi
}

void loop() {


  if (millis() >= startTimeMs + 1000) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      desiredMotorVoltage[i] = 3;
    }
  }
  if (millis() >= startTimeMs + 3000) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      desiredMotorVoltage[i] = 0;
      Serial.println("Finished");
    }
  }


  // decides time between loops
  while (millis() < lastTimeMs + desiredTsMs) {
    // Wait until the desired time passes
  }
  lastTimeMs = millis();


  // output data to serial for debugging
  printData();

  velocityControl();
  calcVelocityAndVolt();
}



void printData() {
  Serial.print(float(millis()) / 1000.0);  // Print current time in seconds
  Serial.print("\t");
  Serial.print(sumVoltage);  // sum of voltage
  Serial.print("\t");
  Serial.print(totalVelocity);  // total velocity change to totalAngle to see angular response
  Serial.println();
}



// calculates velocity errors and uses them to set the motor pwm
// calls setMotor to set the actual pwm of the motor
void velocityControl() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorVoltage[i] = desiredMotorVoltage[i];
    motorPWM[i] = 255 * abs(motorVoltage[i]);
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

void calcVelocityAndVolt() {
  // Calculate motor velocities
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorVelocity[i] = (motorRad[i] - motorPriorPos[i]) / ((millis() - vDelta) / 1000.0);
    motorPriorPos[i] = motorRad[i];
  }
  vDelta = millis();
  totalVelocity = (.075 / 2) * (motorVelocity[1] + motorVelocity[0]);
  totalAngle = (.075/.362) * (motorRad[0] - motorRad[1]);

  // calculate voltage dif and sum
  sumVoltage = motorVoltage[0] + motorVoltage[1];
  difVoltage = motorVoltage[0] - motorVoltage[1];

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

