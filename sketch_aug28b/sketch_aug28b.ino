# include <SPI.h>

void setup() {
  // put your setup code here, to run once:
  SPI.begin(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  SPI.transfer(10,0x00,SPI_CONTINUE);
  delay(3000);
  SPI.transfer(10,0x00,SPI_LAST);
  delay(3000);
}
