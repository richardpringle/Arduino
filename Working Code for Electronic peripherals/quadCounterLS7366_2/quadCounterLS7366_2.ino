#include <SPI.h>

long count;

// Instruction Register (8 bits)
// instruction bits (B7 and B6)
byte CLR = 0x00;
byte RD = 0x40;
byte WR = 0x80;
byte LOAD = 0xC0;
// register bits (B6, B5, B4)
byte MDR0 = 0x08;
byte MDR1 = 0x10;
byte DTR = 0x18;
byte CNTR = 0x20;
byte OTR = 0x28;
byte STR = 0x30;

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
int counter_top = 10;
int counter_bottom = 52;


// Temporary Encoder init() for one encoder with ss on pin 10
void initEncoder() {
  
  // Configure Arduino SPI to work with LS7366
  SPI.begin(counter_top);
  SPI.setDataMode(counter_top,SPI_MODE0);
  SPI.setBitOrder(counter_top,MSBFIRST);
  // Set Encoder to 2-Byte mode
  SPI.transfer(counter_top,WR+MDR1,SPI_CONTINUE);
  SPI.transfer(counter_top,B2_mode,SPI_LAST);
  // Set DTR register to range limit of 4096 (I probably don't need to do this)
  SPI.transfer(counter_top,WR+DTR,SPI_CONTINUE);
  SPI.transfer(counter_top,0x08,SPI_CONTINUE);
  SPI.transfer(counter_top,0x00,SPI_LAST);
  // Set counting mode (can probably keep counter in free_running mode)
  SPI.transfer(counter_top,WR+MDR0,SPI_CONTINUE);
  SPI.transfer(counter_top,range_limit_mode+quad_x4,SPI_LAST);
 
  Serial.println("Encoder Initialized...");
  delay(10);
}

// Temporary Encoder read() for one encoder with ss on pin 10 and 2-byte transfer mode
long readEncoder() {
  unsigned int response1, response2;
  long response;
  
  SPI.transfer(counter_top,RD+CNTR,SPI_CONTINUE);
  response1 = SPI.transfer(counter_top,0x00,SPI_CONTINUE);
  response2 = SPI.transfer(counter_top,0x00,SPI_LAST);
  response = (response1 << 8) + response2;
  
  return response;  
}

// Temporary Encoder clear() for one encoder with ss on pin 10
void clearEncoder() {
  
  SPI.transfer(counter_top,CLR+CNTR);
 
  delay(10);
  
  Serial.println("Encoder Cleared...");
}

// Load starting point number into CNTR to solve negative number problem
//void loadEncoder() {
//  // Load DTR with 8 times the number of counts per revolution
//  // Why 8: 4x quad mode then multiply by 2 so that the counter can count one full revolution down or up
//  SPI.transfer(counter_top,WR+DTR,SPI_CONTINUE);
//  SPI.transfer(counter_top,0x07,SPI_CONTINUE);
//  SPI.transfer(counter_top,0xFF,SPI_LAST);
//  SPI.transfer(counter_top,LOAD+CNTR);  // Loads counter with number from DTR
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(3000);
  initEncoder();
  delay(10);
  clearEncoder();
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  count = readEncoder();
  Serial.println(count);
  delay(10);
//  clearEncoder();
//  delay(1000);
}
