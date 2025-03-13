void setup() {
  // put your setup code here, to run once:
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
}

void loop() {
  analogWrite(5,100);
  analogWrite(6,70);
  digitalWrite(24,0);
  digitalWrite(26,1);
  digitalWrite(25,0);
  digitalWrite(27,1);
}
