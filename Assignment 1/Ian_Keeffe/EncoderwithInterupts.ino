// encoder without interupts
// reads angular position from an encoder by using interupts
int pinThree = 3;
int pinTwo = 2;
int thisA;
int thisB;
int lastA;
int lastB;
int last;
int current;
int last_time_ms;
int desired_Ts_ms;
long count;

void setup() {


  // initalizes pins from encorder with a pullup
  pinMode(pinThree, INPUT_PULLUP);
  pinMode(pinTwo, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinTwo), turn, CHANGE);

  // begins serial communication from ardunio to computer
  Serial.begin(9600);
  Serial.print("\n");
}

void loop() {
  // doesnt do anything but could be used for anything else

}

void turn() {
  // from the assignment doc. 
  // having this and last ensures that the encoder has actually changed
  // and it saves it old state to reference in last
  thisA = digitalRead(pinThree);
  thisB = digitalRead(pinTwo);
  if (lastA != thisA || lastB != thisB) {
    // this concotonates the last state into one number for eaiser processing.
    last = (lastA << 1) | lastB;
    current = (thisA << 1) | thisB;
    
    // every single possiblity of turning in the negative direction on time
    if(last == 0 && current == 1) {
      count++;
    }
    else if(last == 1 && current == 3){
      count++;
    }
    else if(last == 2 && current == 0){
      count++;
    }
    else if(last == 3 && current == 2){
      count++;
    }
    // every single possiblity of turning twice
    else if(last == 0 && current == 3){
      count = count + 2;
    }
    else if(last == 3 && current == 0){
      count = count + 2;
    }
    else if(last == 2 && current == 1){
      count = count - 2;
    }
    else if(last == 2 && current == 1){
      count = count - 2;
    }
    // all other times it implements as turning positive
    else{
      count--;
    }


    // saves the state as the last state
    lastA = thisA;
    lastB = thisB;
    
    // prints the angular position
    Serial.println(count);
}

}
