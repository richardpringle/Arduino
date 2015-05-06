#include <SPI.h>
#include <math.h>

long count_L;
long count_R;

double EE[2];

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


// Temporary Encoder init() for one encoder with ss on pin 10
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
void pos(double *in, int countL, int countR){
  double d, L1, L2, L3, L4;
  double angleL, angleR;
  double angle1, angle2;
  double x1, x2, x3;
  double y1, y2, y3;
  double p3x, p3y;
  double p[2];
  
  // Need to change the location of these values
  d = 15.000;
  L1 = 105.000;
  L2 = 138.000;
  
  p[0] = *in;
  
  angleL = d_map(countL, 0, 8192, -2*PI, 2*PI);
  angleR = d_map(countR, 0, 8192, -2*PI, 2*PI);
  
  x1 = L1*cos(angleL);
  x2 = L1*cos(angleR);
  x3 = (x1 + x2 + d)/2;
  p3x = (x2-x1)/2;
  
  y1 = L1*sin(angleL);
  y2 = L1*sin(angleR);
  y3 = (fabs(y1-y2)/2);
  p3y = (y1+y2)/2;
  
  if (p3x != 0) {
    angle1 = atan(p3y/p3x);
  } else {
    angle1 = PI/2.;
  }
  
  L3 = sqrt(pow(x3,2) + pow(y3,2));
  angle2 = acos(L3/L2);
  L4 = L2*sin(angle2);

  p[0] = p3x + copysign(L4,(countL-countR))*cos(angle1);
  p[1] = p3y + fabs(L4*sin(angle1));
  
  *in = p[0];
  in += 1;
  *in = p[1];  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(3000);
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
}

void loop() {
  //while(HIGH) {
    // put your main code here, to run repeatedly:
    count_R = readEncoder(counter_top);
    count_L = readEncoder(counter_bottom);
  //  Serial.print("counter:");
  //  Serial.print(count_L);
  //  Serial.print(",");
  //  Serial.println(count_R);
    pos(EE,count_L,count_R);
    
//    analogWriteResolution(12);
    
  //  // Back wall
  //  if (EE[1] < 140.) {
  //    digitalWrite(M1, LOW);
  //    digitalWrite(M2, LOW);
  //    analogWrite(E1,4095);
  //    analogWrite(E2,4095);
  //  } 
  //  
  //  if () {
  //    // left wall
  //    if (EE[0] < 0) {
  //      digitalWrite(M1,LOW);
  //      digitalWrite(M2, HIGH);
  //      if (EE[0] < -21.) {
  //        analogWrite(E1,4095);
  //        analogWrite(E2,4095);
  //      } 
  //    }
  //    
  //    // right wall
  //    if (EE[0] >= 0) {
  //      digitalWrite(M1,HIGH);
  //      digitalWrite(M2,LOW);
  //      if (EE[0] > 21.) {
  //        analogWrite(E1,4095);
  //        analogWrite(E2,4095);
  //      }
  //    } 
    
  //  if ((EE[1]>145.) && (EE[0]<20.) && (EE[0]>-20.)) {
  //    analogWrite(E1,0);
  //    analogWrite(E2,0);
  //    if (EE[1]<165) {
  //      digitalWrite(M1,LOW);
  //      digitalWrite(M2,LOW);
  //    } else if(EE[0] >= 0) {
  //      digitalWrite(M1,HIGH);
  //      digitalWrite(M2,LOW);
  //    } else { 
  //      digitalWrite(M1,LOW);
  //      digitalWrite(M2,HIGH);
  //    }
  //  } else {
  //    analogWrite(E1,4095);
  //    analogWrite(E2,4095);
  //    Serial.println("ON"); 
  //  }
    
    Serial.print("Position:");
    Serial.print(EE[0]);
    Serial.print(",");
    Serial.println(EE[1]);
  //} 
}
