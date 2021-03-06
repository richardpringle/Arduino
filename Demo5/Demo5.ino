#include <SPI.h>
#include <math.h>

unsigned int resp;

union binaryFloat 
{
 float floatingPoint;
 byte binary[4];
};

union binaryFloat inData[2];
union binaryFloat outData[4];

int motorTorque[2];
int count = 0;

byte inBytes[9];
byte outBytes[16];
byte motor1Bytes[3];
byte motor2Bytes[3];
boolean outFull = false, inFull = false;

//binaryFloat data0, data1, data2, data3;

byte garbage;
boolean sEvent = false;

// Link Lengths
double L1 = 105.000;      // Crank-arm length
double L2 = 138.000;      // Coupler-link length

// For force generation in on-board-Arduino environment  
long k = 18000;
long b = 750;
long penetration;
long damping;
long wall;
int spd;

// For calculating speed
int dt;
unsigned long t,t0;
double EEx0, EEx1, EEx2, EEy0, EEy1, EEy2;
double vx, vx0, vx1, vx2, vy, vy0, vy1, vy2;
double bfx0, bfx1, bfx2, bfy0, bfy1, bfy2;
// 2nd order Butterworth-filter coefficients
double omega = tan(PI/64);
double den = 1 + sqrt(2)*omega + pow(omega,2);
double a1 = (2*(pow(omega,2)-1))/den;
double a2 = (1 - sqrt(2)*omega + pow(omega,2))/den;
double b0 = (pow(omega,2))/den;
double b1 = (2*pow(omega,2))/den;
double b2 = (pow(omega,2))/den;


// START Lookup Table
// Initialize lookup table for base joint distance
double dTable[1100];
int dStep;
boolean out = true;

void base(double *in){
  	
  double a = 7.92509;
  double b = 0.88114906;
  double xt = 0;
  double t = 3*PI / 2.;
  double alpha, yt;
  double table[1100];
  int step = 1;
  
  table[0] = *in;
  
  table[0] = 0;
  
  *in = table[0];
  in += 1;

  while (xt < 44.5) {
    alpha = (step/1600.)*2.*PI;
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
// END Lookup Table


// START Stepper
#define dir 3
#define stp 12
#define slp 5
#define rst 6
#define ms2 7
#define ms1 8
#define ms0 9
#define en 11

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
long count_L, count_R;
double theta_L, theta_R, F_theta_L, F_theta_R;

float EE[2];// EE[0] = x, EE[1] = y
float V[2]; // V[0] = vx, V[1] = vy -> both filtered with butterworth filter

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
  // Set to 0x1200 for 45deg initialization
  // Set to 0x1000 for 0 ded initialization
  SPI.transfer(counter,0x10,SPI_CONTINUE);
  resp = SPI.transfer(counter,0x00,SPI_LAST);
  // Set counting mode to positive index, reset to DTR (see above)on INDEX, and 4x count
  SPI.transfer(counter,WR+MDR0,SPI_CONTINUE);
  SPI.transfer(counter,free_run_mode+quad_x4,SPI_LAST);
 
  Serial.println(resp);
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
  
//  Serial.println(response1);
//  Serial.println(response2);
  
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
  SPI.transfer(counter,LOAD+CNTR);  // Loads counter with number from DTR
}
// END Counter


// START Forward Kinematics
// Position function using the angles and the forward kinematics
void pos(float *in, double angleL, double angleR, double d){
  double L3, L4;
  double angle3, angle4;
  double x1, x2, x3;
  double y1, y2, y3;
  double p3x, p3y;
  double p[2];
  int temp0, temp1;
  
  p[0] = *in;        // Local position matrix
  
  x1 = L1*cos(angleL) + d;    // For any position of EE along y-axis, both x1 and x2 will be positive
  x2 = L1*cos(angleR) + d;    // The vector of the left revolute joint is [-x1,y1]
  x3 = x1 + x2;               // x3 is the [always positive] x-component of the line connecting passive joints
  
  y1 = L1*sin(angleL);        
  y2 = L1*sin(angleR);
  y3 = fabs(y1-y2);           // y3 is always positive for simplicity as is x3
  
  L3 = sqrt(pow(x3,2) + pow(y3,2));          // L3 is the length of vector [x3,y3] - again, always positive 
  L4 = sqrt(pow(L2,2) - pow((L3/2),2));      // L4 connects the middle of L3 to the EE
  
  angle3 = atan(y3/x3);                      // Angle of L3 -always positive
  angle4 = (PI/2.) + copysign(angle3,(y2-y1));    // Angle of L4 in quadrant I or II
  
  p3x = x2 - 0.5*L3*cos(angle3);                      // [p3x,p3y] connects the origin to L3 and L4
  p3y = y2 - copysign(0.5*L3*sin(angle3),(y2-y1));    // If (y2 > y1) EE will be in quadrant II

  p[0] = p3x + L4*cos(angle4);    // p[0] == EEx
  p[1] = p3y + L4*sin(angle4);    // p[1] == EEy
  
  *in = p[0];
  in += 1;
  *in = p[1];  
}
// END Forward Kinematics


// START Velocity Calculation with Filter
/* Might transfer the velocity estimation here later */
// END Velocity


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
  Serial1.flush();
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
  Serial2.flush();
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


int motor1 = 0, motor2 = 0;
// START Force
void force(float Fx, float Fy, float angleL, float angleR, float d) {
  double x,y;
  int TL,TR;
  byte direcL, direcR;
  
  x = EE[0];
  y = EE[1];
  
  angleL = PI - angleL;
  
  // From paper
  double a11 = L1*(y*cos(angleL) - (x + d)*sin(angleL)); 
  double a22 = L1*(y*cos(angleR)+(d - x)*sin(angleR));
  double b11 = x + d - L1*cos(angleL);
  double b12 = y - L1*sin(angleL);
  double b21 = x - d - L1*cos(angleR);
  double b22 = y - L1*sin(angleR);
  
  double j11 = b11/a11;
  double j12 = b12/a11;
  double j21 = b21/a22;
  double j22 = b22/a22;
  
  // T = J*F -> Here, J is the inverse Jacobian
  TL = -floor(j11*Fx + j12*Fy);
  TR = floor(j21*Fx + j22*Fy);
  
  if (TL < 0) {
//    direcL = motor_forward;
    motor1Bytes[0] = motor_forward;
//    motor1Bytes[0] = quick_forward;
  } else {
//    direcL = motor_reverse;
    motor1Bytes[0] = motor_reverse;
//    motor1Bytes[0] = quick_reverse;    
  }
  
  if (TR < 0) {
//    direcR = motor_forward;
    motor2Bytes[0] = motor_forward;
//    motor2Bytes[0] = quick_forward;    
  } else {
//    direcR = motor_reverse;
    motor2Bytes[0] = motor_reverse;
//    motor2Bytes[0] = quick_reverse;    
  }
  
  ////////////////////////!!!!!!!!!!!!!!!!!!!!!!
/* do the absolute value */  
  motorTorque[0] = abs(TL);
  motorTorque[1] = abs(TR);
  
  // Until I get a better power supply
//  if ((motorTorque[0].integer + motorTorque[2].integer) > 3800) {
//    motorTorque[0] = 1900;
//    motorTorque[1] = 1900;
//  } else {
//  }
//  if ((TL + TR) > 3800) {TL = 1900;TR=1900;}
    if (motorTorque[0] > 1900) {motorTorque[0] = 1900;}
    if (motorTorque[1] > 1900) {motorTorque[1] = 1900;}
    
//    motorTorque[0] = map(motorTorque[0], 0, 1900, 0, 255);
//    motorTorque[1] = map(motorTorque[1], 0, 1900, 0, 255);
//    motor1Bytes[1] = motorTorque[0];
//    motor2Bytes[1] = motorTorque[1];
    motor1Bytes[1] = motorTorque[0] & 0x1F;
    motor1Bytes[2] = motorTorque[0] >> 5;
    motor2Bytes[1] = motorTorque[1] & 0x1F;
    motor2Bytes[2] = motorTorque[1] >> 5;
    
    Serial1.write(motor1Bytes, 3);
    Serial2.write(motor2Bytes, 3);
  
//  compact1(direcL,speedByte1(TL),speedByte2(TL));     // Driver1 is the bottom (angleL)as
//  compact2(direcR,speedByte1(TR),speedByte2(TR));     // Driver2 is the top (angleR)

}
// END Force


// START Step
void stepperMove(byte out) {
  if ((out == 0x0A) && (dStep < 900)) {
    digitalWrite(slp,HIGH);
    digitalWrite(dir,HIGH);
    for (int i=0;i<300 ;i++) {
      digitalWrite(stp,LOW);
      delayMicroseconds(1000);
      digitalWrite(stp,HIGH);
      delayMicroseconds(1000);
      dStep++;      
    }
    digitalWrite(slp,LOW);
  } else if ((out == 0x0B) && (dStep>10)) { 
    digitalWrite(slp,HIGH);
    digitalWrite(dir,LOW);
    for (int i=0;dStep>0 ;i++) {
      digitalWrite(stp,LOW);
      delayMicroseconds(1000);
      digitalWrite(stp,HIGH);
      delayMicroseconds(1000);
      dStep--;      
    }
    digitalWrite(slp,LOW);
  } else {
    digitalWrite(slp,LOW);
  }
}
// END Step


// Allows the first loop to run differently
boolean first_loop = true;
// Iterators:
int io,jo,no, ii, ji, ni;


// START Setup
void setup() {

  // START Stepper
  Serial.begin(115200);
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
  digitalWrite(slp,LOW);
  delay(10);
  // END Stepper
  
  
  // START Counter
  // initEncoder includes loading DTR
  // DTR is loaded to CNTR at INDEX pulsse
  initEncoder(counter_top);
  initEncoder(counter_bottom);
  loadEncoder(counter_top);
  loadEncoder(counter_bottom);
  
  delay(1000);
  // END Counter
  
  
  // START base lookup table
  base(dTable);
  dStep = 0;
  // END base 
  
  
  // START Driver
//  Serial1.begin(115200);  
//  compact1(exit_safe_start);
//  delay(1000);
//  Serial2.begin(115200);  
//  compact2(exit_safe_start);
//  delay(1000);
  Serial1.begin(230400);  
  compact1(exit_safe_start);
  delay(1000);
  Serial2.begin(230400);  
  compact2(exit_safe_start);
  delay(1000);
  // END Driver

  // Get inital position
  // Could combine if I want to increase speed
//  while (EE[1] < 200) {
//    count_R = readEncoder(counter_top);
//    count_L = readEncoder(counter_bottom);
//    
//    // Map the pulse count to an angle in rad
//    theta_L = d_map(count_L, 0, 8192, -2*PI, 2*PI);  
//    theta_R = d_map(count_R, 0, 8192, -2*PI, 2*PI);
//    
//    pos(EE,theta_L,theta_R,dTable[dStep]);
//  }
  
  EEx0 = EE[0];
  EEy0 = EE[1];
  EEx1 = EE[0];
  EEy1 = EE[1];
  vx0 = 0;
  vy0 = 0;
  vx1 = 0;
  vy1 = 0;
  bfx0 = 0;
  bfy0 = 0;
  bfx1 = 0;
  bfy1 = 0;
  
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
}
// END Setup


void loop() {
  // put your main code here, to run repeatedly:
 
  // Reade the Encoders
  count_R = readEncoder(counter_top);
  count_L = readEncoder(counter_bottom);
  
  // Map the pulse count to an angle in rad
  theta_L = d_map(count_L, 0, 8192, -2*PI, 2*PI);  
  theta_R = d_map(count_R, 0, 8192, -2*PI, 2*PI);
  F_theta_L = theta_L;
  F_theta_R = theta_R;
  
  // Read Position
  pos(EE,theta_L,theta_R,dTable[dStep]);
  t = micros();

  // Velocity Estimation
  if (!first_loop) {  // on the first loop dt is too small and vx2, vy2 both approach infinity

    // get current position
    EEx1 = EE[0]; 
    EEy1 = EE[1];
    
    // Find time elapsed between NOW and LAST position reading  
    dt = (t - t0);
    t0 = t;
    
    // Calculate Position based on backwards difference
    vx2 = (EEx1 - EEx0)*1000000/dt;
    vy2 = (EEy1 - EEy0)*1000000/dt;
  
    // Apply butterworth filter with coefficients from top of program
    bfx2 = (b0*vx2 + b1*vx1 + b2*vx0 - a1*bfx1 - a2*bfx0);
    bfy2 = (b0*vy2 + b1*vy1 + b2*vy0 - a1*bfy1 - a2*bfy0);
    
    // Take positions and velocities from NOW and cycle them the LAST position and velocity variables
    EEx0 = EEx1;
    EEy0 = EEy1;
    vx1 = vx2;
    vy1 = vy2;  
    vx0 = vx1;
    vy0 = vy1;
    bfx1 = bfx2;
    bfy1 = bfy2;  
    bfx0 = bfx1;
    bfy0 = bfy1;
    
    // Filter out any velocity values less than 5 [mm/s] -> (Not sure about the units)
    bfx2 = (fabs(bfx2) < 5.0) ? 0 : bfx2;    
    bfy2 = (fabs(bfy2) < 5.0) ? 0 : bfy2;

    V[0] = bfx2;    
    V[1] = bfy2;    
        
  }
  // END Velocity Estimation
  
  // Buffer EE state [x,y,xdot,ydot]
  outData[0].floatingPoint = EE[0];
  outData[1].floatingPoint = EE[1];
//  outData[2].floatingPoint = V[0];
//  outData[3].floatingPoint = V[1];
//  outData[2].floatingPoint = inData[0].floatingPoint;
//  outData[3].floatingPoint = inData[1].floatingPoint;
  io=0;
  jo=0;
  outFull = false;
  while (io<16) {
    no = 0;
    while (no<4) {
      outBytes[io] = outData[jo].binary[no];
      ++io;
      ++no;      
    }
    ++jo;
  }
  outFull = true;
   
  first_loop = false;
  
  if (EE[0] < -75) {
    stepperMove(0x0A);
  }
  
  if (EE[0] > 75) {
    stepperMove(0x0B);
  }
  
//  Serial.println(EE[0]);
  
}

void serialEvent() {
  inFull = false;
  if ( Serial.available() == 9) {
    
//    Serial.println(Serial.readBytes(inBytes, 9));
    Serial.readBytes(inBytes, 9);
    ii=1;
    ji=0;
    while (ii<9) {
      ni = 0;
      while (ni<4) {
        inData[ji].binary[ni] = inBytes[ii];
        ++ii;
        ++ni;     
      }
      ++ji;
    }
    inFull = true;
   
   outData[2].floatingPoint = count_R;
   outData[3].floatingPoint = count_L; 
    
//    if ((inBytes[0] == 0x0A) || (inBytes[0] == 0x0B)) {
//      stepperMove(inBytes[0]);
////      stepperMove(0x00);
//    }    
//      force(0, 0, F_theta_L, F_theta_R, dTable[dStep]);
    force(inData[0].floatingPoint, inData[1].floatingPoint, F_theta_L, F_theta_R, dTable[dStep]); 
  }

  if (outFull && inFull) {
    Serial.write(outBytes, 16);
  }
    
}


