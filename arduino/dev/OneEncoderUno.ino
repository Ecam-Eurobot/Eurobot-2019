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
#define c_EncoderPinA 2 // PORTE4
#define c_EncoderPinB 3 // PORTE5

#define leftLed 8 // PORTH6
#define rightLed 9 // PORTH5

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long _EncoderTicks = 0;


volatile  byte leftLedState = LOW;
volatile  byte rightLedState = LOW;


void setup()
{
  Serial.begin(9600);
  
  pinMode(c_EncoderPinA, INPUT);     // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_EncoderPinB, INPUT);     // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_EncoderPinA), HandleMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_EncoderPinB), HandleMotorInterrupt, CHANGE);

  pinMode(leftLed, OUTPUT);
  digitalWrite(leftLed, LOW);
  pinMode(rightLed, OUTPUT);
  digitalWrite(rightLed, LOW);
}

void loop()
{
  Serial.print(" encoder Ticks: ");
  Serial.print(_EncoderTicks);
  Serial.print("  Revolutions: ");
  Serial.print(_EncoderTicks/4000.0);//4000 Counts Per Revolution
  Serial.print("\n");


//  Serial.println(PIND, BIN);

  digitalWrite(leftLed, leftLedState);
  digitalWrite(rightLed, rightLedState);
}

void HandleMotorInterrupt(){
  _EncoderBSet = ((PINE & B00100000)>>5); //digitalRead(3) PE5
  _EncoderASet = ((PINE & B00010000)>>4); //digitalRead(2) PE4
  
  _EncoderTicks+=ParseEncoder();
  
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;
}


int ParseEncoder(){
  if(_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && _EncoderBSet) return 1;
    if(_EncoderASet && !_EncoderBSet) return -1;
  }else if(!_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && !_EncoderBSet) return 1;
    if(_EncoderASet && _EncoderBSet) return -1;
  }else if(!_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && !_EncoderBSet) return 1;
    if(!_EncoderASet && _EncoderBSet) return -1;
  }else if(_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && _EncoderBSet) return 1;
    if(!_EncoderASet && !_EncoderBSet) return -1;
  }
  return 0;
}

