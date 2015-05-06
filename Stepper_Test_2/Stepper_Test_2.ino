#define Direction 8                     //Direction Pin - Initial State is ZERO
#define Step 9     //Step Pin - Pulse this to step the motor in the direction selected by the Direction Pin
#define Sleepy  2
#define Reset 3
#define MSA 4
#define MSB 5
#define MSC 6
#define Enable 7
#define StepsPerRev 200               //How many steps our motor needs to do a full rotation


int DirectionToggle = 1;                 //Just a Toggle for the Direction Flag

void setup() {
  // put your setup code here, to run once:
  pinMode(Direction, OUTPUT);            
  pinMode(Step, OUTPUT);
  pinMode(Sleepy, OUTPUT);
  pinMode(Reset, OUTPUT);
  pinMode(MSC, OUTPUT);
  pinMode(MSB, OUTPUT);
  pinMode(MSA, OUTPUT);
  pinMode(Enable, OUTPUT);
//
 digitalWrite(Enable,LOW);
 digitalWrite(Direction,HIGH);
 digitalWrite(Sleepy,HIGH);
 digitalWrite(Reset,HIGH);
 digitalWrite(MSA,LOW);
 digitalWrite(MSB,LOW);
 digitalWrite(MSC,LOW);

 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
    //digitalWrite(Direction,DirectionToggle);
  //delay(5);
//  for (int loopy=0; loopy<512;loopy++)
//  {
    digitalWrite(Step,HIGH);
    delay(10); 
    //delayMicroseconds(500);
    digitalWrite(Step,LOW);
    delay(10); 
    //delayMicroseconds(500);
//}
  //DirectionToggle=!DirectionToggle;
//  Serial.print(F("Loop1 - Dir Flag : "));// NOT needed - But lets us see feedback to see if the Code is running
//  Serial.println(DirectionToggle);       // NOT needed - But lets us see feedback to see if the Code is running
}
