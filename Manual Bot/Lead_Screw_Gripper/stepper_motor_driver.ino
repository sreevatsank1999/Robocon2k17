#include<Servo.h>

Servo servo_joint, servo_gripper;

const int stepPin = 3; 
const int dirPin = 4; 
const int left_push_button = 7;  //for moving stepper left side
const int right_push_button = 8; //for moving stepper right side
int left_button_reading = 0;     //variable to store left_push_button
int right_button_reading = 0;    //variable to store right_push_button
const int stepper_speed = 500;         //range if 300-4000
const int pitch_per_press = 1;         //no. of pitche(s) moved by pressing button once 
const int servo_angle_control = A0;    //attach left potentiometer to A0 (x-axis servo)
const int grip_open = A1 ;
const int grip_close = A2;

void setup() {

  servo_joint.attach(3);
  servo_gripper.attach(4);

  pinMode(servo_angle_control, INPUT);
  pinMode(grip_open, INPUT);
  pinMode(grip_close, INPUT);
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT);
  pinMode(left_push_button, INPUT);
  pinMode(right_push_button, INPUT);
}
 
void loop() {

    left_button_reading = digitalRead(left_push_button);
    right_button_reading = digitalRead(right_push_button);
    
    int pot_val = analogRead(servo_angle_control);
    int servo_angle = map(pot_val, -128, 127, 0, 170);
    servo_joint.write(servo_angle);

    int grip_open_reading=digitalRead(grip_open);
    int grip_close_reading=digitalRead(grip_close);
    
    if(grip_open_reading == 1 && grip_close_reading == 0)
     {
       servo_gripper.write(130);
     }
    else if(grip_open_reading == 0 && grip_close_reading == 1)
     {
       servo_gripper.write(175);
     }
    else{}
 
    if(left_button_reading == 1 && right_button_reading == 0)
      {
        digitalWrite(dirPin, LOW);
        for(int i=0; i<200*pitch_per_press; i++)
        {        
          digitalWrite(stepPin,HIGH); 
          delayMicroseconds(stepper_speed); 
          digitalWrite(stepPin,LOW); 
          delayMicroseconds(stepper_speed);
        }
      }
      
    else if(right_button_reading == 1 && left_button_reading == 0)
      {
        digitalWrite(dirPin, HIGH);
        for(int i=0; i<200*pitch_per_press; i++)
        {        
          digitalWrite(stepPin,HIGH); 
          delayMicroseconds(stepper_speed); 
          digitalWrite(stepPin,LOW); 
          delayMicroseconds(stepper_speed);
        }
      }

    else{}
    delay(10);
}

