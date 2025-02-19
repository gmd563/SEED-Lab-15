// SEED_outputs_velocity
// Ian Keeffe and Yoon Seok Lee

// motor on power switch side of the device called right and other side left
// make the motor go forward make 
// 7 (right) HIGH and pwm 9
// 8 (left) HIGH and pwm 10
// Motor control Arduino code with porportional controller and velocity output 
// output is in tab seperated value format

# define pi 3.14159265358979323846264338327950288


// this is used to read the position of the left motor
int leftthisA;
int leftthisB;
int leftlastA;
int leftlastB;
int leftlast;
int leftcurrent;
long leftcount;
// volatile needed so that when interupt updates leftrad it gets updated in main code
volatile float leftrad = 0;

// this is used to read the position of the right motor
int rightthisA;
int rightthisB;
int rightlastA;
int rightlastB;
int rightlast;
int rightcurrent;
long rightcount;
// volatile needed so that when interupt updates rightrad it gets updated in main code
volatile float rightrad = 0;

// enables motor driver
int en = 4;

// this is the pin numbers for the left motor
int leftdir = 8;
int leftpwmpin = 10;
int lefta = 2;
int leftb = 5;

// this is the pin numbers for the right motor
int rightdir = 7;
int rightpwmpin = 9;
int righta = 3;
int rightb = 6;

// this is a constant for calculating the pid control max avail voltage
float battery_voltage = 7.0;

// this is for calculating the velocity and change in position of the motors
float rightpriorpos = 0;
float leftpriorpos = 0;
// these had to be initalized in the main loop for some reason
float rightv = 0;
float leftv = 0;
// for time step calculating velocity
float vDelta = 0;

// this is for controlling the amount of time it waits in the loop and other time control
// found that with desired_Ts_ms < 5 it can be unpredictable
long last_time_ms;
long start_time_ms;
long desired_Ts_ms = 10;

// this is our porpotional controller gain
float kp = 3.5;

// these are used for the left porpotional controller
float lefterror;
float leftdesired_speed = 0;
float leftvoltage;
unsigned int leftpwm;

// these are used for the right porpotional controller
float righterror;
float rightdesired_speed = 0;
float rightvoltage;
unsigned int rightpwm;

// these store the position of the robot
float xpos = 0;
float ypos = 0;
float phi = 0;

void setup() {
  // initalizes pins

  pinMode(en, OUTPUT);

  pinMode(leftdir, OUTPUT);
  pinMode(leftpwm, OUTPUT);
  pinMode(lefta, INPUT_PULLUP);
  pinMode(leftb, INPUT_PULLUP);
  // attaches interrupt to lefta pin for reading position
  attachInterrupt(digitalPinToInterrupt(lefta), leftpos, CHANGE);

  pinMode(rightdir, OUTPUT);
  pinMode(rightpwm, OUTPUT);
  pinMode(righta, INPUT_PULLUP);
  pinMode(rightb, INPUT_PULLUP);
  // attaches interrupt to righta pin for reading position
  attachInterrupt(digitalPinToInterrupt(righta), rightpos, CHANGE);

  // Enable pin
  digitalWrite(en, HIGH);
  // Make left side go forward
  digitalWrite(leftdir, HIGH);
  // Make right side go forward
  digitalWrite(rightdir, HIGH);


  // initalizes serial connection and time
  Serial.begin(115200); // Set the baud rate fast so that we can display the results
  start_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("Ready!");

}








// main loop
void loop() {
  // sets the motors to spin at 1 rad/s from time 1 to 3
  if (millis() >= start_time_ms + 1000) {
    leftdesired_speed = 5;
    rightdesired_speed = 5;
  }
  if (millis() >= start_time_ms + 3000) {
    leftdesired_speed = 0;
    rightdesired_speed = 0;
  }

  // waits a set amount of time
  while (millis()<last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();


  // calculates the x, y and phi 
  // these are the current location of the robot
  // equation found in assignement 2 handout
  xpos = xpos + cos(phi)*((leftrad - leftpriorpos)+(rightrad - rightpriorpos))/2;
  ypos = ypos + sin(phi)*((leftrad - leftpriorpos)+(rightrad - rightpriorpos))/2;
  phi = phi + (1/.34)*((leftrad - leftpriorpos)-(rightrad - rightpriorpos));


  // calculates the velocity of the right and left motors
  rightv = (float)(rightrad - rightpriorpos) / ((float)(millis() - vDelta) / 1000.0);
  leftv = (float)(leftrad - leftpriorpos) / ((float)(millis() - vDelta) / 1000.0);
  // resets variables used in calculation
  vDelta = millis();
  rightpriorpos = rightrad;
  leftpriorpos = leftrad;

  // outputs the right and left velocities.
  // get rid of comments on if and else statement to send data to matlab
  // put comments on if and else statements but not prints to aid debugging
  if (millis() <= start_time_ms + 10000) {
    Serial.print(String(float(millis())/1000));
    Serial.print("\t");
    Serial.print(xpos);
    Serial.print("\t");
    Serial.print(ypos);
    Serial.print("\t");
    Serial.print(phi);
    Serial.println("");
  }
  else {
    Serial.println("Finished");
  }
  
  // runs the velocity controller
  velocityControl(leftv, rightv);

}








// runs the velocity controller
void velocityControl(float lefv, float rightv) {
  // calculates the left porportional controller parameters 
  lefterror = leftdesired_speed - leftv;
  leftvoltage = kp*lefterror;


  // calculates the right porportional controller parameters
  righterror = rightdesired_speed - rightv;
  rightvoltage = kp*righterror;


  // calculate the requested voltage, up to the maximum available
  leftpwm = 255*abs(leftvoltage)/battery_voltage;
  rightpwm = 255*abs(rightvoltage)/battery_voltage;

  // run this to set the proper voltage
  setMotor();
}




// sets the proper voltage
void setMotor() {
  // check the sign of voltage and set the motor driver direction pins as appropriate
  if (leftvoltage<0) {
    digitalWrite(leftdir,LOW);
  } else {
    digitalWrite(leftdir,HIGH);
  }
  if (rightvoltage<0) {
    digitalWrite(rightdir,LOW);
  } else {
    digitalWrite(rightdir,HIGH);
  }

  // Set a PWM signal with a duty cycle of 50% (200 out of 255) on pin 9
  analogWrite(leftpwmpin, min(leftpwm,255));

  // Set a PWM signal with a duty cycle of 50% (200 out of 255) on pin 10
  analogWrite(rightpwmpin, min(rightpwm,255));
}




void leftpos() {
  // from the assignment doc. 
  // having this and last ensures that the encoder has actually changed
  // and it saves it old state to reference in last
  leftthisA = digitalRead(lefta);
  leftthisB = digitalRead(leftb);
  if (leftlastA != leftthisA || leftlastB != leftthisB) {
    // this concotonates the last state into one number for eaiser processing.
    leftlast = (leftlastA << 1) | leftlastB;
    leftcurrent = (leftthisA << 1) | leftthisB;

    // every single possiblity of turning in the positive direction one time
    if (leftlast == 0 && leftcurrent == 1) {
      leftcount++;
    } else if (leftlast == 1 && leftcurrent == 3) {
      leftcount++;
    } else if (leftlast == 2 && leftcurrent == 0) {
      leftcount++;
    } else if (leftlast == 3 && leftcurrent == 2) {
      leftcount++;
    // every single possiblity of turning twice
    } else if (leftlast == 0 && leftcurrent == 3) {
      leftcount += 2;
    } else if (leftlast == 3 && leftcurrent == 0) {
      leftcount += 2;
    } else if (leftlast == 2 && leftcurrent == 1) {
      leftcount -= 2;
    } else if (leftlast == 1 && leftcurrent == 2) {
      leftcount -= 2;
    // all other times it implements as turning negative
    } else {
      leftcount--;
    }

    // saves the state as the last state
    leftlastA = leftthisA;
    leftlastB = leftthisB;

    // sets left rad to the position in radians
    leftrad= 2*pi*(float)leftcount/3200;
  }
}

void rightpos() {
  // from the assignment doc. 
  // having this and last ensures that the encoder has actually changed
  // and it saves it old state to reference in last
  rightthisA = digitalRead(righta);
  rightthisB = digitalRead(rightb);
  if (rightlastA != rightthisA || rightlastB != rightthisB) {
    // this concotonates the last state into one number for eaiser processing.
    rightlast = (rightlastA << 1) | rightlastB;
    rightcurrent = (rightthisA << 1) | rightthisB;

    // every single possiblity of turning in the positive direction one time
    if (rightlast == 0 && rightcurrent == 1) {
      rightcount--;
    } else if (rightlast == 1 && rightcurrent == 3) {
      rightcount--;
    } else if (rightlast == 2 && rightcurrent == 0) {
      rightcount--;
    } else if (rightlast == 3 && rightcurrent == 2) {
      rightcount--;
    // every single possiblity of turning twice
    } else if (rightlast == 0 && rightcurrent == 3) {
      rightcount -= 2;
    } else if (rightlast == 3 && rightcurrent == 0) {
      rightcount -= 2;
    } else if (rightlast == 2 && rightcurrent == 1) {
      rightcount += 2;
    } else if (rightlast == 1 && rightcurrent == 2) {
      rightcount += 2;
    // all other times it implements as turning positive
    } else {
      rightcount++;
    }

    // saves the state as the last state
    rightlastA = rightthisA;
    rightlastB = rightthisB;

    // sets right rad to the position in radians
    rightrad= 2*pi*(float)rightcount/3200;
  }
}