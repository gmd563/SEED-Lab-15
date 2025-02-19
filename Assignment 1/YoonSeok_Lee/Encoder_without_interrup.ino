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
// put your setup code here, to run once:
pinMode(pinThree, INPUT_PULLUP);
pinMode(pinTwo, INPUT_PULLUP);
Serial.begin(9600);
Serial.print("\n");
}
void loop() {
Serial.println(MyEnc());
while (millis()<last_time_ms + desired_Ts_ms) {
//wait until desired time passes to go top of the loop
}
last_time_ms = millis();
}
long MyEnc() {
// put your main code here, to run repeatedly:
// it is the code to implement the table.
thisA = digitalRead(pinThree);
thisB = digitalRead(pinTwo);
if (lastA != thisA || lastB != thisB) {
last = (lastA << 1) | lastB;
current = (thisA << 1) | thisB;
if(last == 0 && current == 1) {
count--;
}
else if(last == 1 && current == 3){
count--;
}
else if(last == 2 && current == 0){
count--;
}
else if(last == 3 && current == 2){
count--;
}
else{
count++;
}
lastA = thisA;
lastB = thisB;
return count;
}
}
