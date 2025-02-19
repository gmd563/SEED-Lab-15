// motor on power switch side of the device called right and other side left
// make the motor go forward make 
// 7 (right) high and pwm 9
// 8 (left) low and pwm 10
// Motor control Arduino code

# define pi 3.14159265358979323846264338327950288

int leftthisA;
int leftthisB;
int leftlastA;
int leftlastB;
int leftlast;
int leftcurrent;
long leftcount;
float leftrad;

int rightthisA;
int rightthisB;
int rightlastA;
int rightlastB;
int rightlast;
int rightcurrent;
long rightcount;
float rightrad;

int en = 4;

int leftdir = 7;
int leftpwm = 9;
int lefta = 2;
int leftb = 5;

int rightdir = 8;
int rightpwm = 10;
int righta = 3;
int rightb = 6;

float pwm = 0;
float voltage = 7.5;
float rightpriorpos = 0;
float leftpriorpos = 0;
// float rightv = 0;
// float leftv = 0;

long last_time_ms;
long start_time_ms;
long desired_Ts_ms = 10;

void setup() {
  pinMode(en, OUTPUT);

  pinMode(leftdir, OUTPUT);
  pinMode(leftpwm, OUTPUT);
  pinMode(lefta, INPUT_PULLUP);
  pinMode(leftb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lefta), leftpos, CHANGE);

  pinMode(rightdir, OUTPUT);
  pinMode(rightpwm, OUTPUT);
  pinMode(righta, INPUT_PULLUP);
  pinMode(rightb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(righta), rightpos, CHANGE);

  // Enable pin
  digitalWrite(en, HIGH);
  // Make left side go forward
  digitalWrite(leftdir, HIGH);
  // Make right side go forward
  digitalWrite(rightdir, LOW);

  Serial.begin(115200); // Set the baud rate fast so that we can display the results
  start_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("Ready!");
}

void loop() {

  if (millis() >= start_time_ms + 1000) {
    pwm = 125;
  }
  if (millis() >= start_time_ms + 3000) {
    pwm = 0;
  }
  // Set a PWM signal with a duty cycle of 50% (200 out of 255) on pin 9
  analogWrite(leftpwm, pwm);

  // Set a PWM signal with a duty cycle of 50% (200 out of 255) on pin 10
  analogWrite(rightpwm, pwm);


  // waits a set amount of time
  while (millis()<last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
  float rightv = ((float)(rightrad - rightpriorpos))/ (float)((float)desired_Ts_ms/(float)1000);
  float leftv = ((float)(leftrad - leftpriorpos)) / (float)((float)desired_Ts_ms/(float)1000);
  if (millis() <= start_time_ms + 4000) {
    Serial.print(String(float(millis())/1000));
    Serial.print("\t");
    Serial.print(String(float(((voltage*pwm)/255))));
    Serial.print("\t");
    Serial.print(String(float(rightv), 4));
    Serial.print("\t");
    Serial.print(String(float(leftv), 4));
    Serial.println("");
  }
  else {
    Serial.println("Finished");
  }
  rightpriorpos = rightrad;
  leftpriorpos = leftrad;
}

void leftpos() {
  leftthisA = digitalRead(lefta);
  leftthisB = digitalRead(leftb);
  if (leftlastA != leftthisA || leftlastB != leftthisB) {
    leftlast = (leftlastA << 1) | leftlastB;
    leftcurrent = (leftthisA << 1) | leftthisB;

    if (leftlast == 0 && leftcurrent == 1) {
      leftcount--;
    } else if (leftlast == 1 && leftcurrent == 3) {
      leftcount--;
    } else if (leftlast == 2 && leftcurrent == 0) {
      leftcount--;
    } else if (leftlast == 3 && leftcurrent == 2) {
      leftcount--;
    } else if (leftlast == 0 && leftcurrent == 3) {
      leftcount -= 2;
    } else if (leftlast == 3 && leftcurrent == 0) {
      leftcount -= 2;
    } else if (leftlast == 2 && leftcurrent == 1) {
      leftcount += 2;
    } else if (leftlast == 1 && leftcurrent == 2) {
      leftcount += 2;
    } else {
      leftcount++;
    }

    leftlastA = leftthisA;
    leftlastB = leftthisB;

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
      rightcount++;
    } else if (rightlast == 1 && rightcurrent == 3) {
      rightcount++;
    } else if (rightlast == 2 && rightcurrent == 0) {
      rightcount++;
    } else if (rightlast == 3 && rightcurrent == 2) {
      rightcount++;
    } else if (rightlast == 0 && rightcurrent == 3) {
      rightcount += 2;
    } else if (rightlast == 3 && rightcurrent == 0) {
      rightcount += 2;
    } else if (rightlast == 2 && rightcurrent == 1) {
      rightcount -= 2;
    } else if (rightlast == 1 && rightcurrent == 2) {
      rightcount -= 2;
    } else {
      rightcount--;
    }

    rightlastA = rightthisA;
    rightlastB = rightthisB;

    rightrad= 2*pi*(float)rightcount/3200;
  }
}
