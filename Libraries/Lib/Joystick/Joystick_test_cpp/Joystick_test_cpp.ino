#include "Joystick.h"

Joystick Jx(A7);

void setup() {

 Serial.begin(9600);
 Jx.Initialise();
 
}

/*
int test (){
 //  Serial.begin(9600);
 Serial.print("Hello ");Serial.println(micros());
 
 int a;
 analogWrite(3, 128);
// for(int i =0; i < 1000000000; i++);
  Serial.print("Hello ");Serial.print(micros());
 delay(10);
 a = analogRead(A6);
 Serial.print(analogRead(A6));
    return a + 3000;
}

const int k = test();
*/

void loop() {
  // put your main code here, to run repeatedly:

  Jx.Joystick_Debug();

//  Serial.println(k);
  
  delay(200);
}




