int val = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(0);
  
  Serial.write(map(val, 0, 1023, 0, 255));
  delay(100);
}
