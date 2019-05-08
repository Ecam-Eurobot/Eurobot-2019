
const byte pwm_motor2 = 9;
const byte dir_motor2 = 11;

void setup() {
  //set level pins
  pinMode(pwm_motor2, OUTPUT);
  pinMode(dir_motor2, OUTPUT);

  Serial.begin(9600);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

}

void loop() {
  digitalWrite(dir_motor2, 0);
  analogWrite(pwm_motor2, 200);
}
