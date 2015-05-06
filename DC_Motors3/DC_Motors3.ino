// Pololu Protocol
#define pololu_protocol 0xAA
// Exit Safe Start -> pololu_protocol, deviceNumber, command
#define pololu_exit_safe_start 0x03
// Drive Motor -> pololu_protocol, deviceNumber, command, speedByte1, speedByte2
// speedByte1 = (speed % 32) or (speed & 0x1F), speedByte2 = (speed/32) or (speed >> 5)
// 0 < speed < 3200
#define pololu_forward 0x05
#define pololu_reverse 0x06
// 7-bit speed resolution -> pololu_protocol, deviceNumber, command, speed
// 0 < speed < 127
#define pololu_quick_forward 0x09
#define pololu_quick_reverse 0x0A
// Stop Motor -> pololu_protocol, deviceNumber, command
#define pololu_stop 0x60

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

// Mini SSC Protocol
// Drive Motor -> command, deviceNumber, speed
// speed = signedSpeed + 127
// -127 < signedSpeed < 127
#define mini_SSC 0xFF

void pololu(byte a, byte b) {
  Serial1.write(pololu_protocol);
  Serial1.write(a);
  Serial1.write(b);
}

void pololu(byte a, byte b, byte c) {
  Serial1.write(pololu_protocol);
  Serial1.write(a);
  Serial1.write(b);
  Serial1.write(c);
}

void pololu(byte a, byte b, byte c, byte d) {
  Serial1.write(pololu_protocol);
  Serial1.write(a);
  Serial1.write(b);
  Serial1.write(c);
  Serial1.write(d);
}

void compact(byte a) {
  Serial1.write(a);
}

void compact(byte a, byte b) {
  Serial1.write(a);
  Serial1.write(b);
}

void compact(byte a, byte b, byte c) {
  Serial1.write(a);
  Serial1.write(b);
  Serial1.write(c);
}

byte speedByte1(int x) {
  return x & 0x1F;
}

byte speedByte2(int x) {
  return x >> 5;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  
  compact(exit_safe_start);
  
  delay(1000);  
}

int spd;

void serialEvent(){
  if (Serial.available() > 1) {
    spd = Serial.parseInt();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  if (Serial.available() > 1) {
//    Serial.println("Start");
//    Serial.println(spd);
//    spd = Serial.parseInt();
//    Serial.println(spd);
//    Serial.println("End");
//  }
  
//    if (Serial.available() > 1) {
//      spd = Serial.parseInt();
//    }
  
//  Serial.println(spd);
  compact(motor_forward,speedByte1(spd),speedByte2(spd));
  
}
