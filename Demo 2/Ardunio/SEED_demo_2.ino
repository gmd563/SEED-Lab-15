#include <Wire.h>

#define pi 3.14159265358979323846264338327950288
#define MOTOR_COUNT 2
#define I2C_ADDRESS 8
#define MAX_BYTES 32   // Maximum bytes per I2C transaction

uint8_t receivedData[MAX_BYTES]; // Store as unsigned bytes
int receivedCount = 0;



// RIGHT IS 0
// LEFT IS 1

// Motor pin configuration
const int motorDirPins[MOTOR_COUNT] = {7, 8};
const int motorPwmPins[MOTOR_COUNT] = {9, 10};
const int motorEncoderA[MOTOR_COUNT] = {3, 2};
const int motorEncoderB[MOTOR_COUNT] = {6, 5};
// enables motor driver
int en = 4;

// individual motor parameters
float motorVoltage[MOTOR_COUNT] = {0.0, 0.0};
unsigned int motorPWM[MOTOR_COUNT] = {0, 0};
volatile float motorRad[MOTOR_COUNT] = {0.0, 0.0};
float motorVelocity[MOTOR_COUNT] = {0.0, 0.0};
float motorPriorPos[MOTOR_COUNT] = {0.0, 0.0};

// velocity control variables
float desiredVelocity = 0.0;
float tempDesiredVelocity = 0.0;
float totalVelocity = 0.0;
float velocityError = 0.0;
float totalMotorVoltage = 0.0;
float batteryVoltage = 7.0;
float kpVelocity = 75;

// position control variables
float maxVelocity = .3;  // Example: 1 m/s
float maxAccel = 0.0000005;  // Max acceleration in m/s²
float posError = 0.0;
float prevPosError = 0.0;
float desiredPos = 0.0;
float totalPos = 0.0;
float posIntegralError = 0.0;
float tempPosIntegralError = 0.0;
float kpPos = 18;
float kiPos = 4.5;

// anglular velocity control variables 
float desiredAngularVelocity = 0.0;
float tempDesiredAngularVelocity = 0.0;
float totalAngularVelocity = 0.0;
float angularVelocityError = 0.0;
float deltaTotalMotorVoltage = 0.0;
float kpAngularVelocity = 13;

// angle control variables
float maxAngularVelocity = .6;  // Example: rad/s²
float maxAngularAccel = 0.005;  // Max acceleration in rad/s²
float angleError = 0.0;
float prevAngleError = 0.0;
float desiredAngle = 0.0;
float totalAngle = 0.0;
float angleIntegralError = 0.0;
float tempAngleIntegralError = 0.0;
// float kpAngularPos = 23;
// float kiAngularPos = 7.15;
float kpAngularPos = 5;
float kiAngularPos = 0;

// Time variables
long lastTimeMs;
long startTimeMs;
long desiredTsMs = 10;
float vDelta = 0;


// Vechile dynamics variables
float sumVoltage = 0;
float difVoltage = 0;

// input from pi
volatile float detected = 0.0;
volatile float angle = 0.0;
volatile float distance = 0.0;

enum {search, move, moveToSearch};
unsigned char state = search;  // What the door is doing at any given moment.




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
  Wire.begin(I2C_ADDRESS);            // Initialize as I2C slave
  Wire.onReceive(receiveEvent);       // Register receive event


  
}

void loop() {
  if (receivedCount > 1) { // Ensure there's more than just the offset byte
    bool once;
    switch (state) {
      case search:
        if (detected != 0) {
          state = move;
        }
        desiredAngle = totalAngle + 10000000;
        break;
      
      case move:
        desiredAngle = totalAngle + (angle * (3.14159/180)); // desired angle in degrees
        desiredPos = totalPos + (distance *.3048); // replace 1 with desired pos in ft
        if (detected == 0) {
          once = 1;
          state = moveToSearch;
        }
        break;

      case moveToSearch:
        if (once == 1) {
          desiredAngle = totalAngle + (angle * (3.14159/180));
          once = 0;
        }
        if (angle == 0) {
          state = search;
        }
        break;
    }
    receivedCount = 0; // Reset for the next transaction
  }
  // decides time between loops
  while (millis() < lastTimeMs + desiredTsMs) {
    // Wait until the desired time passes
  }
  lastTimeMs = millis();




  printData();
  calcVals();
  positionControl();
  processData();

}
void processData() {
  if (receivedCount > 1) { // Ensure there's more than just the offset byte

    // Process the received data, ignoring the first byte (offset)
    for (int i = 1; i < receivedCount; i += 4) {
      if (i + 3 >= receivedCount) break;  // Avoid reading incomplete floats

      if (i < 5) memcpy(&detected, &receivedData[i], sizeof(float));
      if (i < 9) memcpy(&angle, &receivedData[i], sizeof(float));
      if (i < 13) memcpy(&distance, &receivedData[i], sizeof(float));
    }
    // receivedCount = 0; // Reset for the next transaction (done in main loop)


  }
}

// this takes in what will be from the pi. 
void receiveEvent(int numBytes) {
  receivedCount = min(numBytes, MAX_BYTES); // Prevent overflow

  for (int i = 0; i < receivedCount; i++) {
    receivedData[i] = (uint8_t) Wire.read(); // Read as unsigned bytes
  }
}



void printData() {
  Serial.print(state);  // Print current time in seconds
  Serial.print("\t");
  Serial.print(detected);  // total velocity
  Serial.print("\t");
  Serial.print(angle);  // sum of voltage
  Serial.print("\t");
  Serial.print(desiredAngle);  // sum of voltage
  Serial.print("\t");
  Serial.print(totalAngle);  // sum of voltage
  Serial.print("\t");
  Serial.print(distance);  // total velocity
  Serial.print("\t");
  Serial.print(desiredPos);  // total velocity
  Serial.print("\t");
  Serial.print(totalPos);  // total velocity
  Serial.println();
}



// controls the position of the motors,
// calculates errors, desired speed, then calls velocity control to set that speed
void positionControl() {
  posError = desiredPos - totalPos;
  angleError = desiredAngle - totalAngle;


  // Compute integral terms 
  tempPosIntegralError = posIntegralError + posError * ((float)desiredTsMs / 1000);
  tempAngleIntegralError = angleIntegralError + angleError * ((float)desiredTsMs / 1000);

  // Compute desired velocities (before limiting acceleration)
  tempDesiredVelocity = (kpPos * posError + kiPos * tempPosIntegralError);
  tempDesiredAngularVelocity = (kpAngularPos * angleError + kiAngularPos * tempAngleIntegralError);

  // Clamp max velocity
  tempDesiredVelocity = constrain(tempDesiredVelocity, -maxVelocity, maxVelocity);
  tempDesiredAngularVelocity = constrain(tempDesiredAngularVelocity, -maxAngularVelocity, maxAngularVelocity);

  // Apply rate limiting to avoid sudden acceleration
  float deltaT = millis() - lastTimeMs / 1000;  // Convert ms to seconds
  float maxVelocityChange = maxAccel * deltaT;
  float maxAngularVelocityChange = maxAngularAccel * deltaT;


  // Smooth velocity transition
  if (tempDesiredVelocity > desiredVelocity + maxVelocityChange) {
    desiredVelocity += maxVelocityChange;
  }
  else if (tempDesiredVelocity < desiredVelocity - maxVelocityChange) {
    desiredVelocity -= maxVelocityChange;
  }
  else {
    desiredVelocity = tempDesiredVelocity;
  }

  // Smooth angular velocity transition
  if (tempDesiredAngularVelocity > desiredAngularVelocity + maxAngularVelocityChange) {
    desiredAngularVelocity += maxAngularVelocityChange;
  }
  else if (tempDesiredAngularVelocity < desiredAngularVelocity - maxAngularVelocityChange) {
    desiredAngularVelocity -= maxAngularVelocityChange;
  }
  else {
    desiredAngularVelocity = tempDesiredAngularVelocity;
  }

  // Reset integral when error changes sign (prevents overshooting)**
  if ((posError > 0 && prevPosError < 0) || (posError < 0 && prevPosError > 0)) {
    posIntegralError = 0;
  }
  if ((angleError > 0 && prevAngleError < 0) || (angleError < 0 && prevAngleError > 0)) {
    angleIntegralError = 0;
  }

  prevPosError = posError;
  prevAngleError = angleError;


  velocityControl();
}

// calculates velocity errors and uses them to set the motor pwm
// calls setMotor to set the actual pwm of the motor
void velocityControl() {
  velocityError = desiredVelocity - totalVelocity;
  angularVelocityError = desiredAngularVelocity - totalAngularVelocity;
  totalMotorVoltage = kpVelocity * velocityError;
  deltaTotalMotorVoltage = kpAngularVelocity * angularVelocityError;
  motorVoltage[0] = (totalMotorVoltage + deltaTotalMotorVoltage)/2;
  motorVoltage[1] = (totalMotorVoltage - deltaTotalMotorVoltage)/2;
  for (int i = 0; i < MOTOR_COUNT; i++) {
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

void calcVals() {
  // Calculate motor velocities
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorVelocity[i] = (motorRad[i] - motorPriorPos[i]) / ((millis() - vDelta) / 1000.0);
    motorPriorPos[i] = motorRad[i];
  }
  vDelta = millis();

  // calculates the total velocity for the velocity proportional controller
  totalVelocity = (.075 / 2) * (motorVelocity[1] + motorVelocity[0]);
  totalAngularVelocity = (.075/.362) * (motorVelocity[0] - motorVelocity[1]);

  totalPos = (.07524 / 2) * (motorRad[1] + motorRad[0]);
  totalAngle = (.07524/.362) * (motorRad[0] - motorRad[1]);

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
  static long long count[MOTOR_COUNT] = {0};


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

