void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int A;
  int B;
  
  A = analogRead(0);
  B = analogRead(1);
  
  Serial.print(A);
  Serial.print("\t");
  Serial.println(B);
}
