#include <SPI.h>
#include <math.h>

// Initialize lookup table for base joint distance
double dTable[1100];
int dStep;

void base(double *in){
  	
  double pi = 4.*atan(1.);
  double a = 7.92509;
  double b = 0.88114906;
  double xt = 0;
  double t = 3*pi / 2.;
  double alpha, yt;
  double table[1100];
  int step = 1;
  
  table[0] = *in;
  
  table[0] = 0;
  
  *in = table[0];
  in += 1;

  while (xt < 44.5) {
    alpha = (step/1600.)*2.*pi;
    yt = -(a*(t-b+alpha)*sin(t));
    while ( yt > 30.3625 ) {
      t = -(30.3625/(a*sin(t))) - (alpha - b);
      yt = -(a*(t-b+alpha)*sin(t));
    }
    xt = rint( -(a*(t-b+alpha)*cos(t))*10. )/10.;
    table[step] = xt;
    *in = table[step];
    in += 1;    
    step++;
  }
  
}

// START Stepper
int dir = 3;
int stp = 12;
int slp = 5;
int rst = 6;
int ms2 = 7;
int ms1 = 8;
int ms0 = 9;
int en = 11;

int direc = HIGH;

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
// END Stepper

// START Counter
long count_L;
long count_R;

double EE[2]; // EE[0] = x, EE[1] = y

// I'm pretty sure all these byte variables should be #define
// and same goes for the counter_top and counter_bottom

// Instruction Register (8 bits)
// instruction bits (B7 and B6)
byte CLR = 0x00;
byte RD = 0x40;
byte WR = 0x80;
byte LOAD = 0xC0;
// register bits (B5, B4, B3)
byte MDR0 = 0x08;
byte MDR1 = 0x10;
byte DTR = 0x18;
byte CNTR = 0x20;
byte OTR = 0x28;
byte STR = 0x30;
// xxx - don't care (B2, B1, B0)

// MDRO Register data for write-to-MDR0 command (8 bits)
// MDR0 Quadrature Mode (B1 and B0)
byte non_quad_mode = 0x00;
byte quad_x1 = 0x01;
byte quad_x2 = 0x02;
byte quad_x4 = 0x03;
//MDR0 Counting Mode (B3 and B2)
byte free_run_mode = 0x00;
byte one_cycle_mode = 0x04;
byte range_limit_mode = 0x08;
byte moduloN_mode = 0x0C;
// MDR0 Configure Index (B5 and B4)
byte disable_index = 0x00;
byte index_load_CNTR = 0x10;
byte index_reset_CNTR = 0x20;
byte index_load_OTR = 0x30;
// MDR0 index sign (B6)
byte index_negative = 0x00;
byte index_positive = 0x40;
//MDR0 Filter Clock Division Factor (B7)
byte filterClockBy_1 = 0x00;
byte filterClockBy_2 = 0x80;

// MDR1 Register data for write(read)-to(from)-MDR1 (8 bits)
// MDR1 counter mode (B1 and B0)
byte B4_mode = 0x00;
byte B3_mode = 0x01;
byte B2_mode = 0x02;
byte B1_mode = 0x03;
// MDR1 Enable Counting (B2)
byte enable_count = 0x00;
byte disable_count = 0x04;


// pins on arduino
int counter_top = 10;  // green to counter_top
int counter_bottom = 4;  // green to bottom

// map function for doubles
double d_map(long x, long in_min, long in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Encoder init() for individual encoder (input: counter_top or counter_bottom)
void initEncoder(int counter) {
  
  // Configure Arduino SPI to work with LS7366
  SPI.begin(counter);
  SPI.setDataMode(counter,SPI_MODE0);
  SPI.setBitOrder(counter,MSBFIRST);
  // Set Encoder to 2-Byte mode
  SPI.transfer(counter,WR+MDR1,SPI_CONTINUE);
  SPI.transfer(counter,B2_mode,SPI_LAST);
  // Set DTR register to 4095 (middle of 4x count of 1024 encoder for one full up and one full down)
  // Use Modulo-N with range of +/- 4095 counts
  SPI.transfer(counter,WR+DTR,SPI_CONTINUE);
//  temporarily set to 0x1200 for 45deg initialization
  SPI.transfer(counter,0x12,SPI_CONTINUE);
  SPI.transfer(counter,0x00,SPI_LAST);
  // Set counting mode to positive index, reset to DTR (see above)on INDEX, and 4x count
  SPI.transfer(counter,WR+MDR0,SPI_CONTINUE);
  SPI.transfer(counter,free_run_mode+quad_x4,SPI_LAST);
 
  Serial.println("Encoder Initialized...");
  delay(10);

}

// Encoder read()  
long readEncoder(int counter) {
  unsigned int response1, response2;
  long response;
  
  SPI.transfer(counter,RD+CNTR,SPI_CONTINUE);
  response1 = SPI.transfer(counter,0x00,SPI_CONTINUE);
  response2 = SPI.transfer(counter,0x00,SPI_LAST);
  response = (response1 << 8) + response2;
  
  return response;  
}

// Encoder clear() 
void clearEncoder(int counter) {
  
  SPI.transfer(counter,CLR+CNTR);
 
  delay(10);
  
  Serial.println("Encoder Cleared...");
}

// Load starting point number into CNTR to solve negative number problem
// Not needed when using INDEX
void loadEncoder(int counter) {
  // Load DTR with 8 times the number of counts per revolution
  // Why 8: 4x quad mode then multiply by 2 so that the counter can count one full revolution down or up
//  SPI.transfer(counter,WR+DTR,SPI_CONTINUE);
//  SPI.transfer(counter,0x0F,SPI_CONTINUE);
//  SPI.transfer(counter,0xFF,SPI_LAST);
  SPI.transfer(counter,LOAD+CNTR);  // Loads counter with number from DTR
//  // reset DTR to range limit
//  SPI.transfer(counter_top,WR+DTR,SPI_CONTINUE);
//  SPI.transfer(counter_top,0x10,SPI_CONTINUE);
//  SPI.transfer(counter_top,0x00,SPI_LAST);
}

// Position function using the angles and the forward kinematics
void pos(double *in, int countL, int countR, double x){
  double d, L1, L2, L3, L4;
  double angleL, angleR;
  double angle3, angle4, angleP;
  double x1, x2, x3;
  double y1, y2, y3;
  double p3x, p3y;
  double p[2];
  
  // Need to change the location of these values
  d = x;             // Distance along x-axis from origin to a single base joint 
  L1 = 105.000;      // Crank-arm length
  L2 = 138.000;      // Coupler-link length
  
  p[0] = *in;        // Local position matrix
  
  angleL = d_map(countL, 0, 8192, -2*PI, 2*PI);  // Map the pulse count to an angle in rad
  angleR = d_map(countR, 0, 8192, -2*PI, 2*PI);
  
  x1 = L1*cos(angleL) + d;    // For any position of EE along y-axis, both x1 and x2 will be positive
  x2 = L1*cos(angleR) + d;    // The vector of the left revolute joint is [-x1,y1]
  x3 = x1 + x2;               // x3 is the [always positive] x-component of the line connecting passive joints
  
  y1 = L1*sin(angleL);        
  y2 = L1*sin(angleR);
  y3 = fabs(y1-y2);           // y3 is always positive for simplicity as is x3
  
  L3 = sqrt(pow(x3,2) + pow(y3,2));          // L3 is the length of vector [x3,y3] - again, always positive 
  L4 = sqrt(pow(L2,2) - pow((L3/2),2));      // L4 connects the middle of L3 to the EE
  
  angle3 = atan(y3/x3);                      // Angle of L3 -always positive
  angle4 = 90 + copysign(angle3,(y2-y1));    // Angle of L4 in quadrant I or II
  
  p3x = x2 - 0.5*L3*cos(angle3);                      // [p3x,p3y] connects the origin to L3 and L4
  p3y = y2 - copysign(0.5*L3*cos(angle3),(y2-y1));    // If (y2 > y1) EE will be in quadrant II

  p[0] = p3x + L4*cos(angle4);    // p[0] == EEx
  p[1] = p3y + L4*sin(angle4);    // p[1] == EEy
  
  *in = p[0];
  in += 1;
  *in = p[1];  
}
// END Counter

// START Driver
// Compact Protocol
// Exit Safe Start -> command
#define exit_safe_start 0x83
// Drive Motor -> command, speedByte1, speedByte2
// speedByte1 = speed % 32 (speed & 0x1F), speedByte2 = speed/32 (speed >> 5)
// 0 < speed < 3200
#define motor_forward 0x85
#define motor_reverse 0x86
// 7-bit speed resolution -> command, speed
// 0 < speed < 127
#define quick_forward 0x89
#define quick_reverse 0x8A
// Stop Motor -> command
#define stop_motor 0xE0

void compact1(byte a) {
  Serial1.write(a);
}

void compact1(byte a, byte b) {
  Serial1.write(a);
  Serial1.write(b);
}

void compact1(byte a, byte b, byte c) {
  Serial1.write(a);
  Serial1.write(b);
  Serial1.write(c);
}

void compact2(byte a) {
  Serial2.write(a);
}

void compact2(byte a, byte b) {
  Serial2.write(a);
  Serial2.write(b);
}

void compact2(byte a, byte b, byte c) {
  Serial2.write(a);
  Serial2.write(b);
  Serial2.write(c);
}

byte speedByte1(int x) {
  return x & 0x1F;
}

byte speedByte2(int x) {
  return x >> 5;
}
// END Driver

void setup() {
  // put your setup code here, to run once:

  // START Stepper
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
  eighthStep();

  digitalWrite(en,LOW);
  delay(10); 
  digitalWrite(rst,LOW);
  delay(10);
  digitalWrite(rst,HIGH);
  delay(10);
  // END Stepper
  
  // START Counter
    // initEncoder includes loading DTR
  // DTR is loaded to CNTR at INDEX pulsse
  initEncoder(counter_top);
  initEncoder(counter_bottom);
//  clearEncoder(counter_top);
//  clearEncoder(counter_bottom);
  loadEncoder(counter_top);
  loadEncoder(counter_bottom);
  
//  Serial1.begin(115200);
  
  delay(1000);
  // END Counter
  
  // START Driver
  Serial1.begin(115200);  
  compact1(exit_safe_start);
  delay(1000);
  Serial2.begin(115200);  
  compact2(exit_safe_start);
  delay(1000);
  // END Driver
  
  // START base lookup table
  base(dTable);
  dStep = 0;
  // END base 
  

}

void loop() {
  // put your main code here, to run repeatedly:
  long k = 100;
  long x;
  int spd;
  
  count_R = readEncoder(counter_top);
  count_L = readEncoder(counter_bottom);
  pos(EE,count_L,count_R,dTable[dStep]);
  
  if (EE[0] > 60) {
    digitalWrite(slp,HIGH);
    digitalWrite(dir,HIGH);
    for (int i=0;i<265;i++) {
      digitalWrite(stp,LOW);
      delayMicroseconds(1000);
      digitalWrite(stp,HIGH);
      delayMicroseconds(1000);
      dStep++;
    }    
  } else if (EE[0] < -60) {
       digitalWrite(slp,HIGH);
       digitalWrite(dir,LOW);
       for (int i=0;i<265;i++) {
          digitalWrite(stp,LOW);
          delayMicroseconds(1000);
          digitalWrite(stp,HIGH);
          delayMicroseconds(1000);
          dStep--;    
       }
       if (dStep < 0) { dStep = 0; }      
   } else {
         digitalWrite(slp,LOW);
   }
   
    if (EE[1] < 165) {
      x = 165 - EE[1];
    } else {
      x = 0;
    }
    
    if ((k*x) < 3200) {
      spd = k*x;
    } else {
      spd = 3200;
    }
  
//    compact1(motor_reverse,speedByte1(spd),speedByte2(spd));
//    compact2(motor_reverse,speedByte1(spd),speedByte2(spd)); 



    Serial.print(EE[0]);
    Serial.print(", ");
    Serial.print(EE[1]);
    Serial.print(", ");
    Serial.println(dStep);
    
}
