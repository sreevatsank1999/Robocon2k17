#include "../../Libraries/Lib/Joystick/Joystick_ver_Arduino/Joystick.h"
#include "../../Libraries/Lib/SoftwareServo/SoftwareServo.h"

class MyServo : public SoftwareServo {
  const unsigned char PinNo;

  MyServo(const unsigned char Num):PinNo(Num), SoftwareServo(){}

  void Initialize(){
    (*this).attach(PinNo);
  }
};

class Robotic_Arm{

  Joystick &Jrw;                    //For Planar Motion
  Joystick &Jo;                     //For Inclination

  MyServo &S_Base;
  MyServo &S_01, &S_02, &S_03;
  MyServo &S_Grip;

  const float r1, r2, r3;

  public:
    float A_Base;                   // Servo Base Angle
    float A_01, A_02, A_03;         // Servo Limb Angles

    float O;                        // Arm Inclination

    float ModR;
    
    float k;          // Angle of Attack Ratio


  Robotic_Arm(Joystick &rw, Joystick &o, MyServo arrS[5], const float Limb1, const float Limb2, const float Limb3)
            : Jrw(rw), Jo(o),
              S_Base(arrS[0]), S_01(arrS[1]), S_02(arrS[2]), S_03(arrS[3]), S_Grip(arrS[4]),
              r1(Limb1), r2(Limb2}, r3(Limb3)
        {  }

  void Initialize(){
    
    A_Base = PI/2;
    A_01 = PI/2;
    A_02 = PI;
    A_03 = 0;

    Drive_S();
    Set_ModR();
  }

  void S_Refresh(){

    S_Base.refresh();
    S_01.refresh();
    S_02.refresh();
    S_03.refresh();
    S_Grip();
  }

  void Set_ModR(){
    ModR = (r1*cos(A_01)+r2*cos(A_01+A_02)+r3*cos(A_01+A_02+A_03));
  }
  int Set_A(const float C){
    A_01 = asin((r3*sin(C))/sqrt(sq(ModR-r3*cos(C)) + sq(r3*sin(C)))) + acos((sq(r1)+sq(ModR-r3*cos(C))+sq(r3*sin(C))-sq(r3))/2*r1*sqrt(sq(ModR-r3*cos(C)) + sq(r3*sin(C))));
    A_02 = (-1)*( A_01 + asin((r1*sin(A_01)-r3*sin(C))/(ModR-r1*cos(A_01)-r3*cos(C))));
    A_03 = (-1)*(C - asin((r1*sin(A_01)-r3*sin(C))/(ModR-r1*cos(A_02)-r3*cos(C))));

    return 0;
  }
};

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
