 float vPow = 4.7; 
float r1 = 100000; 
float r2 = 10000;
int ledOutput = 7;
const int buzzer = 9; //buzzer to arduino pin 9

#include <LiquidCrystal.h> 
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
   pinMode(buzzer, OUTPUT);
 pinMode(ledOutput, OUTPUT);
   lcd.begin(16, 2);

   Serial.begin(9600);

   Serial.println("--------------------"); 
   Serial.println("DC VOLTMETER"); 
   Serial.print("Maximum Voltage: "); 
   Serial.print((int)(vPow / (r2 / (r1 + r2)))); 
   Serial.println("V"); 
   Serial.println("--------------------"); 
   Serial.println("");

   delay(2000); }
                 
void loop() { 
    float v = (analogRead(A0) * vPow) / 1024.0; 
    float v2 = v / (r2 / (r1 + r2));
      digitalWrite(ledOutput, LOW);


        if (v2 <= 17){
                 lcd.clear();
           Serial.println(" Voltage=");
      Serial.println(v2);
  Serial.println("V");
    lcd.print("Voltage= ");
      lcd.print(v2);
        lcd.print(" V");
         tone(buzzer, 1000); 
         delay(1000);
         noTone(buzzer); 
         delay(100);
    digitalWrite(ledOutput, HIGH); 
     delay(100);

        digitalWrite(ledOutput, LOW); 
     delay(100);
 
  }
  else { 
    lcd.clear();
     Serial.println("Voltage=");
      Serial.println(v2);
  Serial.println("V");
    lcd.print("Voltage= ");
      lcd.print(v2);
      lcd.print("V");
  delay(500);

    noTone(buzzer);     // Stop sound... 
    digitalWrite(ledOutput, LOW); 
       delay(100); 
     } 
}   
     
      

