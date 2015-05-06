int E1 = 12;
int M1 = 13;
int E2 = 10;
int M2 = 11;
int direc = LOW;

void setup() {
  // put your setup code here, to run once:
  pinMode(E1,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(E2,OUTPUT);
  pinMode(M2,OUTPUT);
  //analogWriteResolution(12); 
  digitalWrite(M1,direc);
  digitalWrite(M2,direc);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(E1,125);
  analogWrite(E2,255);
  delay(1000);
  analogWrite(E1,0);
  analogWrite(E2,0);
  delay(1000);
  direc = !direc;
//  digitalWrite(M1,direc);
  digitalWrite(M2,direc);
  delay(1000);
}
