#include "DC_Motor.h"

DC_Motor M1(2, 22), M2(3, 24), M3(4, 26), M4(5, 28);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

pinMode(3,INPUT);        // High => High Direction   Low => Low Direction
pinMode(5,INPUT);        // Enable/ Disable

M1.Initialise();
M2.Initialise();
M3.Initialise();
M4.Initialise();

}

void loop() {

  bool Dir = digitalRead(42);
  bool Enable = digitalRead(43);
  float Vr = 0.4;

  if(Enable == LOW)      Vr = 0;
  else                   Vr = 0.4;

 if(Dir == HIGH){
 M1.DriveMotor(Vr);
 M2.DriveMotor(Vr);
 M3.DriveMotor(Vr);
 M4.DriveMotor(Vr);
 Serial.println(Vr);
}
else{ 
 M1.DriveMotor(-Vr);
 M2.DriveMotor(-Vr);
 M3.DriveMotor(-Vr);
 M4.DriveMotor(-Vr);
 Serial.println(-Vr);
}

}
