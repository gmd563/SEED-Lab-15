// motor on power switch side of the device called right and other side left
// make the motor go forward make 
// 7 (right) high and pwm 9
// 8 (left) low and pwm 10
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
float leftrad;

// this is used to read the position of the right motor
int rightthisA;
int rightthisB;
int rightlastA;
int rightlastB;
int rightlast;
int rightcurrent;
long rightcount;
float rightrad;

// enables motor driver
int en = 4;

// this is the pin numbers for the left motor
int leftdir = 7;
int leftpwmpin = 9;
int lefta = 2;
int leftb = 5;

// this is the pin numbers for the right motor
int rightdir = 8;
int rightpwmpin = 10;
int righta = 3;
int rightb = 6;

// this is a constant for calculating the pid control max avail voltage
float battery_voltage = 7.0;

// this is for calculating the velocity and change in position of the motors
float rightpriorpos = 0;
float leftpriorpos = 0;
// these had to be initalized in the main loop for some reason
// float rightv = 0;
// float leftv = 0;

// this is for controlling the amount of time it waits in the loop and other time control
long last_time_ms;
long start_time_ms;
long desired_Ts_ms = 1;

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
  digitalWrite(leftdir, LOW);
  // Make right side go forward
  digitalWrite(rightdir, HIGH);


  // initalizes serial connection and time
  Serial.begin(115200); // Set the baud rate fast so that we can display the results
  start_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("Ready!");
}

void loop() {
  // sets the motors to spin at 1 rad/s from time 1 to 3
  if (millis() >= start_time_ms + 1000) {
    leftdesired_speed = 1;
    rightdesired_speed = 1;
  }
  if (millis() >= start_time_ms + 3000) {
    leftdesired_speed = 0;
    rightdesired_speed = 0;
  }



  // waits a set amount of time declared in desired_Ts_ms declaration
  while (millis()<last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }

  Serial.println(String((float)(rightrad), 6));
  

  // calculates the velocity of the right and left motors
  float rightv = ((float)(rightrad - rightpriorpos))/ (float)((float)(millis()-last_time_ms)/(float)1000);
  float leftv = ((float)(leftrad - leftpriorpos)) / (float)((float)(millis()-last_time_ms)/(float)1000);

  // outputs the right and left velocities.
  // get rid of comments on if and else statement to send data to matlab
  //if (millis() <= start_time_ms + 4000) {
    Serial.print(String(float(millis())/1000));
    Serial.print("\t");
    Serial.print(String(float(((rightvoltage*rightpwm)/255))));
    Serial.print("\t");
    Serial.print(String(float(((leftvoltage*leftpwm)/255))));
    Serial.print("\t");
    Serial.print(String(float(rightv), 4));
    Serial.print("\t");
    Serial.print(String(float(leftv), 4));
    Serial.println("");
  //}
  //else {
    //Serial.println("Finished");
  //}

  last_time_ms = millis();
  

  // sets priorpos to be equal to current position for next iteration of the loop
  rightpriorpos = rightrad;
  leftpriorpos = leftrad;

  // calculates the left porportional controller parameter
  lefterror = leftdesired_speed - leftv;
  leftvoltage = kp*lefterror;

  // calculates the right porportional controller parameter
  righterror = rightdesired_speed - rightv;
  rightvoltage = kp*righterror;

  

  // check the sign of voltage and set the motor driver direction pins as appropriate
  if (leftvoltage<0) {
    digitalWrite(leftdir,HIGH);
  } else {
    digitalWrite(leftdir,LOW);
  }
  if (rightvoltage<0) {
    digitalWrite(rightdir,LOW);
  } else {
    digitalWrite(rightdir,HIGH);
  }

  // Apply the requested voltage, up to the maximum available
  leftpwm = 255*abs(leftvoltage)/battery_voltage;
  rightpwm = 255*abs(rightvoltage)/battery_voltage;


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
  rightthisA = digitalRead(righta);
  rightthisB = digitalRead(rightb);
  if (rightlastA != rightthisA || rightlastB != rightthisB) {
    rightlast = (rightlastA << 1) | rightlastB;
    rightcurrent = (rightthisA << 1) | rightthisB;

    if (rightlast == 0 && rightcurrent == 1) {
      rightcount--;
    } else if (rightlast == 1 && rightcurrent == 3) {
      rightcount--;
    } else if (rightlast == 2 && rightcurrent == 0) {
      rightcount--;
    } else if (rightlast == 3 && rightcurrent == 2) {
      rightcount--;
    } else if (rightlast == 0 && rightcurrent == 3) {
      rightcount -= 2;
    } else if (rightlast == 3 && rightcurrent == 0) {
      rightcount -= 2;
    } else if (rightlast == 2 && rightcurrent == 1) {
      rightcount += 2;
    } else if (rightlast == 1 && rightcurrent == 2) {
      rightcount += 2;
    } else {
      rightcount++;
    }

    rightlastA = rightthisA;
    rightlastB = rightthisB;

    rightrad= 2*pi*(float)rightcount/3200;
  }
}
