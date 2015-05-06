int direc = LOW;

int M1 = 11;
int M2 = 12;
int E2 = 8;
int E1 = 9;

void setup() {
  // put your setup code here, to run once:
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(M1,direc);
  digitalWrite(M2,direc);
//  analogWriteResolution(12);
  analogWrite(E1,255);
  analogWrite(E2,255);
  delay(1000);
  analogWrite(E1,0);
  analogWrite(E2,0);
  delay(1000);
//  direc = !direc;
//  digitalWrite(M2, direc);
  delay(1000);
  
}
