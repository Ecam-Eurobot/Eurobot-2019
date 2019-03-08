/* Arduino Mega 2560 interrupt pins
Interrupt number: 0    1    2    3    4    5
physical pin:     2    3    21   20   19   18
*/

/* port mapping
       7  6  5  4  3  2  1  0
       -----------------------
PORTA: 29 28 27 26 25 24 23 22
PORTB: 13 12 11 10 50 51 52 53
PORTC: 30 31 32 33 34 35 36 37
PORTD: 38 -- -- -- 18 19 20 21
PORTE: -- -- 3  2  5  -- 1  0
PORTG: -- -- 4  -- -- 39 40 41
PORTH: -- 8  9  7  6  -- 16 17
PORTJ: -- -- -- -- -- -- 14 15
PORTL: 42 43 44 45 16 47 48 49
*/

// Macros for easier port access
#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))

// Quadrature Encoders
#define c_LeftEncoderPinA 2 // PORTE4
#define c_LeftEncoderPinB 3 // PORTE5
#define c_RightEncoderPinA 19 // PORTD2
#define c_RightEncoderPinB 18 // PORTD3

#define leftLed 4 // PORTH6
#define rightLed 5 // PORTH5

volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile long _LeftEncoderTicks = 0;

volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile bool _RightEncoderAPrev;
volatile bool _RightEncoderBPrev;
volatile long _RightEncoderTicks = 0;

volatile  byte leftLedState = LOW;
volatile  byte rightLedState = LOW;


void setup()
{
  Serial.begin(9600);
  
  pinMode(c_LeftEncoderPinA, INPUT);     // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);     // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinB), HandleLeftMotorInterrupt, CHANGE);

  pinMode(c_RightEncoderPinA, INPUT);     // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);     // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinA), HandleRightMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinB), HandleRightMotorInterrupt, CHANGE);

  pinMode(leftLed, OUTPUT);
  digitalWrite(leftLed, LOW);
  pinMode(rightLed, OUTPUT);
  digitalWrite(rightLed, LOW);
}

void loop()
{
  Serial.print("Left encoder Ticks: ");
  Serial.print(_LeftEncoderTicks);
  Serial.print("  Revolutions: ");
  Serial.print(_LeftEncoderTicks/4000.0);//4000 Counts Per Revolution
  Serial.print("    Right encoder Ticks: ");
  Serial.print(_RightEncoderTicks);
  Serial.print("  Revolutions: ");
  Serial.print(_RightEncoderTicks/4000.0);//4000 Counts Per Revolution
  Serial.print("\n");


//  Serial.println(PIND, BIN);

  digitalWrite(leftLed, leftLedState);
  digitalWrite(rightLed, rightLedState);
}

void HandleLeftMotorInterrupt(){
  _LeftEncoderBSet = ((PINE & B00100000)>>5); //digitalRead(3) PE5
  _LeftEncoderASet = ((PINE & B00010000)>>4); //digitalRead(2) PE4
  
  _LeftEncoderTicks+=ParseLeftEncoder();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

void HandleRightMotorInterrupt(){
  _RightEncoderBSet = ((PIND & B00000100)>>2); // digitalRead(20) PD2
  _RightEncoderASet = ((PIND & B00001000)>>3); // digitalRead(21) PD3
  
  _RightEncoderTicks+=ParseRightEncoder();
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}

int ParseLeftEncoder(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }
}

int ParseRightEncoder(){
  if(_RightEncoderAPrev && _RightEncoderBPrev){
    leftLedState = HIGH;
    rightLedState = HIGH;
    if(!_RightEncoderASet && _RightEncoderBSet) return 1;
    if(_RightEncoderASet && !_RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && _RightEncoderBPrev){
    leftLedState = LOW;
    rightLedState = HIGH;
    if(!_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && !_RightEncoderBPrev){
    leftLedState = LOW;
    rightLedState = LOW;
    if(_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(!_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(_RightEncoderAPrev && !_RightEncoderBPrev){
    leftLedState = HIGH;
    rightLedState = LOW;
    if(_RightEncoderASet && _RightEncoderBSet) return 1;
    if(!_RightEncoderASet && !_RightEncoderBSet) return -1;
  }
}
