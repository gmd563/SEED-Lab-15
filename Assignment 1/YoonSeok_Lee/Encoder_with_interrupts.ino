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
attachInterrupt(digitalPinToInterrupt(pinTwo), turn, CHANGE);
//added interrupt
Serial.begin(9600);
Serial.print("\n");
}
void loop() {
}
void turn() {
// put your main code here, to run repeatedly:
thisA = digitalRead(pinThree);
thisB = digitalRead(pinTwo);
if (lastA != thisA || lastB != thisB) {
last = (lastA << 1) | lastB;
current = (thisA << 1) | thisB;
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
else{
count--;
}
lastA = thisA;
lastB = thisB;
Serial.println(count);
}
}
