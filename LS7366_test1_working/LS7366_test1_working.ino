#include <SPI.h>

byte test1 = 0x00;
byte test2 = 0xFF;
byte WR = 0x80;
byte MDR0 = 0x08;
byte quad_x1 = 0x01;
byte MDR1 = 0x10;
byte B1_mode = 0x03;
byte LOAD = 0xC0;
byte STR = 0x30;
byte DTR = 0x18;
byte RD = 0x40;
byte CNTR = 0x20;

int counter_top = 10;

void initEncoder() {
  
  SPI.begin(counter_top);
  delay(3000);
  SPI.setDataMode(counter_top,SPI_MODE0);
  delay(3000);
  SPI.setBitOrder(counter_top,MSBFIRST);
  delay(3000);
  
  SPI.transfer(counter_top,WR+MDR0,SPI_CONTINUE);
  delay(3000);
  SPI.transfer(counter_top,quad_x1,SPI_LAST);
  delay(3000);
  SPI.transfer(counter_top,WR+MDR1,SPI_CONTINUE);
  SPI.transfer(counter_top,B1_mode,SPI_LAST);
  Serial.println("Encoder Initialized...");
  delay(3000);
}

void clearEncoder() {
  SPI.transfer(counter_top,WR+DTR,SPI_CONTINUE);
  SPI.transfer(counter_top,0x00,SPI_LAST);\
  
  delay(10);
  
  SPI.transfer(counter_top,LOAD+STR);
  Serial.println("Encoder Cleared...");
}

long readEncoder() {
  SPI.transfer(counter_top,RD+CNTR,SPI_CONTINUE);
  byte response = SPI.transfer(counter_top,0x00,SPI_LAST);
  return response;  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initEncoder();
  clearEncoder();
}

void loop() {
  // put your main code here, to run repeatedly:
  SPI.transfer(counter_top,0x00,SPI_CONTINUE);
  delay(3000);
  SPI.transfer(counter_top,0x00,SPI_LAST);
  delay(3000);
  Serial.println(readEncoder());

}
