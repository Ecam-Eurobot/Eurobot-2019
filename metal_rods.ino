/* The "Servo" library must be included to handle the servomotor */
#include <Servo.h>

/* Creating a servo object to control the servo motor */
Servo monServomoteur;
Servo monServomoteur2;

void setup() {
  /* Attach the servomotor to pin x */
  monServomoteur.attach(9);
  monServomoteur2.attach(10);

  /* According to the signal on the pin 7 activation of one of the servomotor */
  /* Definition of the starting point at 90° */
  if(digitalRead(7)){
     monServomoteur2.write(90);
     delay(15);
  }else{
     monServomoteur.write(90);
     delay(15);
  }
}


void loop() {
  /* The left one : right to left */
  if(digitalRead(7)){
    for (int position = 110; position <= 40; position++) {
      monServomoteur2.write(position);
      delay(15);
    }
  }else{
  /* The right one : left to right */
    for (int position = 110; position >= 40; position--) {
      monServomoteur.write(position);
      delay(15);
    }
  }
}
