int d0 = 23;
int d1 = 25;
int d2 = 27;
int d3 = 29;
int d4 = 31;
int d5 = 33;
int d6 = 35;
int d7 = 37;

int cs = 50;

int wr = 48;
int rd = 46;

int cd = 47;
int xy = 49;

long xPosition = 0;
long yPosition = 0;

int level[8];

void dataOut(){
  pinMode(d0,OUTPUT);
  pinMode(d1,OUTPUT);
  pinMode(d2,OUTPUT);
  pinMode(d3,OUTPUT);
  pinMode(d4,OUTPUT);
  pinMode(d5,OUTPUT);
  pinMode(d6,OUTPUT);
  pinMode(d7,OUTPUT);
}

void dataIn() {
  pinMode(d0,INPUT);
  pinMode(d1,INPUT);
  pinMode(d2,INPUT);
  pinMode(d3,INPUT);
  pinMode(d4,INPUT);
  pinMode(d5,INPUT);
  pinMode(d6,INPUT);
  pinMode(d7,INPUT);
}

void dataRead() {
  dataIn();
  delay(1000);
  level[0] = digitalRead(d0);
  level[1] = digitalRead(d1);
  level[2] = digitalRead(d2);
  level[3] = digitalRead(d3);
  level[4] = digitalRead(d4);
  level[5] = digitalRead(d5);
  level[6] = digitalRead(d6);
  level[7] = digitalRead(d7);
}

void writeControlX() {
  delay(100);
  digitalWrite(wr, LOW); // write control
  delay(10);
  digitalWrite(cs, LOW);
  delay(10);
  digitalWrite(cd, HIGH); //control 
  digitalWrite(xy, LOW); // choose x
  delay(100);  
  digitalWrite(wr, HIGH); // end write control
  delay(10);
  digitalWrite(cs, HIGH);
  delay(100);
}

void writeDataX() {
  delay(100);
  digitalWrite(wr, LOW); // write control
  delay(10);
  digitalWrite(cs, LOW); // chip select
  delay(10);
  digitalWrite(cd, LOW); //data 
  digitalWrite(xy, LOW); // choose x
  delay(100);  
  digitalWrite(wr, HIGH); // end write control
  delay(10);
  digitalWrite(cs, HIGH);
  delay(100);
}

void readDataX() {
  digitalWrite(rd, LOW); // read data
  delay(10);
  digitalWrite(cs, LOW); //chip select
  delay(10);
  digitalWrite(cd, LOW); // data   
  digitalWrite(xy, LOW); // choose x
  delay(1000);
  dataRead();
  digitalWrite(rd, HIGH); // end read data
  delay(10);
  digitalWrite(cs,HIGH); //end chip select
  delay(10);
}

void writeControlY() {
  digitalWrite(cd, HIGH); //control 
  delay(1000);  
  digitalWrite(xy, HIGH); // choose y
  delay(1000);
  digitalWrite(wr, LOW); // write control
  delay(1000);
  digitalWrite(wr, HIGH); // end write  control
  delay(1000);  
}

void writeDataY() {
  digitalWrite(cd, LOW); //data 
  delay(1000);  
  digitalWrite(xy, HIGH); // choose y
  delay(1000);
  digitalWrite(wr, LOW); // write data
  delay(1000);
  digitalWrite(wr, HIGH); // end write  data
  delay(1000);  
}

void readDataY() {
  digitalWrite(cd, HIGH); // data 
  delay(1000);  
  digitalWrite(xy, HIGH); // choose y
  delay(1000);
  digitalWrite(rd, LOW); // read data
  delay(1000);
  digitalWrite(rd, HIGH); // end read data
  delay(1000);
}

void clockData() {
  dataOut();
  delay(1000);
  // Setup for 1MHz Oscillator
  digitalWrite(d7, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void clockSetup() {
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, HIGH);
  digitalWrite(d3, HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void inputSetup() {
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, HIGH);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,HIGH);
}

void quadX1(){
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,HIGH);
  digitalWrite(d4, LOW);
  digitalWrite(d3, HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void quadX2(){
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,HIGH);
  digitalWrite(d4, HIGH);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void quadX4(){
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,HIGH);
  digitalWrite(d4, HIGH);
  digitalWrite(d3, HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void bpReset() {
  dataOut();
  delay(1000);
  digitalWrite(d7, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,HIGH);
}

void bpReset_B() {
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,HIGH);
}

void counterReset() {
  dataOut();
  delay(1000);
  digitalWrite(d7, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,HIGH);
  digitalWrite(d0,LOW);
}

void counterReset_B() {
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,HIGH);
  digitalWrite(d0,LOW);
}

void transferCounter() {
  dataOut();
  delay(1000);
  digitalWrite(d7, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
  digitalWrite(cs, LOW);
}

void transferCounter_OL(){
  // d7 is LOW for x-only and HIGH for both x and y
  dataOut();
  delay(1000);
  digitalWrite(d7, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, HIGH);
  digitalWrite(d3, LOW);
  digitalWrite(d2,LOW);
  digitalWrite(d1,LOW);
  digitalWrite(d0,LOW);
}

void resetEFlag() {
  dataOut();
  delay(1000);
  digitalWrite(d7, HIGH);
  digitalWrite(d6, LOW);
  digitalWrite(d5,LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d2,HIGH);
  digitalWrite(d1,HIGH);
  digitalWrite(d0,LOW);
}

void initialize7226() {
  resetEFlag();
  writeControlX();
  bpReset_B();
  writeControlX();
  clockData();
  writeDataX();
  clockSetup();
  writeControlX();
  inputSetup();
  writeControlX();
  quadX1();
  writeControlX();
  counterReset();
  writeControlX();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
  pinMode(cs,OUTPUT);
  delay(1000);
  digitalWrite(cs,HIGH);
  pinMode(wr,OUTPUT);
  delay(1000);
  digitalWrite(wr, HIGH);
  pinMode(rd,OUTPUT);
  delay(1000);
  digitalWrite(rd,HIGH);
  pinMode(xy,OUTPUT);
  delay(1000);
  digitalWrite(xy, LOW);
  pinMode(cd,OUTPUT);
   
  initialize7226();
  
  transferCounter_OL();
  writeControlX();
  bpReset_B();
  writeControlX();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  int j;
  
  transferCounter_OL();
  writeControlX();
  bpReset_B();
  writeControlX();
  
  xPosition = 0; // initialize variable to 0 before first byte read
  
  for (i = 0;i<3;i++) {
    readDataX();
    delay(10);
    for (j = 0;j < 8; j++) {
        xPosition += level[j]*pow(2,j+(i*8));
    }
  }
  
  Serial.println("x: ");
  Serial.println(xPosition);
  Serial.println("d0");
  Serial.println(level[0]);
  Serial.println("d1");
  Serial.println(level[1]);
  Serial.println("d2");
  Serial.println(level[2]);
  Serial.println("d3");
  Serial.println(level[3]);
  Serial.println("d4");
  Serial.println(level[4]);
  Serial.println("d5");
  Serial.println(level[5]);
  Serial.println("d6");
  Serial.println(level[6]);
  Serial.println("d7");
  Serial.println(level[7]);
  delay(1000);
//  xPosition = 0;
  
}
