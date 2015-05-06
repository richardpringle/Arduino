int dir = 3;
int stp = 12;
int slp = 5;
int rst = 6;
int ms2 = 7;
int ms1 = 8;
int ms0 = 9;
int en = 11;

int direc = LOW;

void fullStep(){
  digitalWrite(ms0,LOW);
  digitalWrite(ms1,LOW);
  digitalWrite(ms2,LOW);
  Serial.println("fullStep();");
}

void halfStep(){
  digitalWrite(ms0,HIGH);
  digitalWrite(ms1,LOW);
  digitalWrite(ms2,LOW);
  Serial.println("halfStep();");
}

void quarterStep(){
  digitalWrite(ms0,LOW);
  digitalWrite(ms1,HIGH);
  digitalWrite(ms2,LOW);
  Serial.println("quarterStep();");
}

void eighthStep(){
  digitalWrite(ms0,HIGH);
  digitalWrite(ms1,HIGH);
  digitalWrite(ms2,LOW);
  Serial.println("eigthStep();");
}

void sixteenthStep(){
  digitalWrite(ms0,LOW);
  digitalWrite(ms1,LOW);
  digitalWrite(ms2,HIGH);
  Serial.println("sixtennthStep();");
}

void thirtysecondthStep() {
  digitalWrite(ms0,HIGH);
  digitalWrite(ms1,HIGH);
  digitalWrite(ms2,HIGH);
  Serial.println("1/32 Step;");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir,OUTPUT);
  pinMode(stp,OUTPUT);
  pinMode(slp,OUTPUT);
  pinMode(rst,OUTPUT);
  pinMode(ms2,OUTPUT);
  pinMode(ms1,OUTPUT);
  pinMode(ms0,OUTPUT);
  pinMode(en,OUTPUT);
  
  digitalWrite(en,HIGH);
  delay(10);
  digitalWrite(slp,HIGH);
  digitalWrite(dir,direc);
  digitalWrite(stp,HIGH);
  digitalWrite(rst,HIGH);
  quarterStep();

  digitalWrite(en,LOW);
  delay(10); 
  digitalWrite(rst,LOW);
  delay(10);
  digitalWrite(rst,HIGH);
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
//  delay(1500);

// !!!!!!!!!!!!!!!!!!!!!!!!!These values didn't work that well, but they did work.

  for (int i=0;i<530;i++) {
    digitalWrite(stp,LOW);
    delayMicroseconds(400);
    digitalWrite(stp,HIGH);
    delayMicroseconds(400);    
  }
  delay(2000);
  digitalWrite(dir,!direc);
  direc = !direc;
  delay(2000);

}
