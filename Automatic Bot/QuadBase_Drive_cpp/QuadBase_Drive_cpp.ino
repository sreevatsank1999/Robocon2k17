#include "../../Libraries/Lib/DC_Motor/DC_Motor.h"
#include "../../Libraries/Lib/Joystick/Joystick_ver_Arduino/Joystick.h"
#include "../../Libraries/Lib/PID/PID.cpp"
#include "../../Libraries/Lib/Line Follower/Line_Follower.cpp"

class DualLineSensor{

	Cytron &Cytron_5k;
	Polulo &Polulo_QTRRC;

	const float dist;

public: 
	float tani;
	const float InpMax;

	DualLineSensor(Cytron &cytron, Polulo &polulo, const float MaxInp, const float Dist) : Cytron_5k(cytron), Polulo_QTRRC(polulo), InpMax(MaxInp), dist(Dist)
	{}

	int Update() {
		tani = (Cytron_5k.Real_dist() - Polulo_QTRRC.Real_dist()) / dist;
	}
	float PID_Inp() {
		return tani;
	}
};

class QuadBaseDrive{            // C++ Square Base Drive Class

    Joystick &Jxy;
    Joystick &Jw;
    
    DC_Motor &M1;
    DC_Motor &M2;
    DC_Motor &M3;
    DC_Motor &M4;

    int pwm_Max;

  public:
      QuadBaseDrive(Joystick &xy, Joystick &w, DC_Motor Motor[4], const int Max_pwm = 220)
      : Jxy(xy), Jw(w),
        M1(Motor[0]), M2(Motor[1]), M3(Motor[2]), M4(Motor[3])
      {
        M1.pwm_Max_Set(Max_pwm);
        M2.pwm_Max_Set(Max_pwm);
        M3.pwm_Max_Set(Max_pwm);
        M4.pwm_Max_Set(Max_pwm);

        pwm_Max = Max_pwm;
      }

    void Initialise(){
      M1.Initialise();
      M2.Initialise();
      M3.Initialise();
      M4.Initialise();
    }

      int Read_Drive(){                                     // Realtime Read and Drive Function with default pwm_Max
      return Read_Drive(pwm_Max);
    }
    int Read_Drive(const int Max_pwm) {                    // Realtime Read and Drive Function

      Linear_Drive(Max_pwm);
      delay(25);

 //     Rotate_Drive(Max_pwm);
 //     delay(25);

      return 0;
    }

    int Linear_Drive(){                                    // Realtime Read and Drive Function with default pwm_Max
      return   Linear_Drive(pwm_Max);
    }
    int Linear_Drive(const int Max_pwm) {                 // Realtime Read and Drive Function

      Jxy.Update();
      Linear_Set(Jxy.K, Jxy.CosO, Jxy.SinO, M1.Vr, M2.Vr, M3.Vr, M4.Vr);
      Drive(Max_pwm);

      return 0;
    }

    int Rotate_Drive(){                                 // Realtime Read and Drive Function with default pwm_Max
      return Rotate_Drive(pwm_Max);
    }
    int Rotate_Drive(const int Max_pwm) {                     // Realtime Read and Drive Function

      Jw.Update();
      float Wr = Jw.K;
      // Vri => Vi/Vm, Vm - Max Motor Velocity
      Rotate_Set(Wr, M1.Vr, M2.Vr, M3.Vr, M4.Vr);
      Drive(Max_pwm);

      return 0;
    }

   int Linear_Set(float K, float CosO, float SinO, float &Vr1, float &Vr2, float &Vr3, float &Vr4) {      // Sets last used Value
  /*    
        Serial.print(CosO);
        Serial.print(", ");
        Serial.print(SinO);
        Serial.print(", ");
        Serial.print(K);
        Serial.println("   ");
      */

     
         Vr1 = K*(((float)1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO);
         Vr2 = K*(((float)1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO);                  // ***** ERROR******
         Vr3 = K*(((float)-1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO);    
         Vr4 = K*(((float)-1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO);      
    
/*
        Serial.print(Vr1);
        Serial.print(", ");
        Serial.print(Vr2);
        Serial.print(", ");
        Serial.print(Vr3);
        Serial.print(", ");
        Serial.print(Vr4);
        Serial.println("   ");
*/
      return 0;
    }
    int Rotate_Set(float Wr, float &Vr1, float &Vr2, float &Vr3, float &Vr4) {                             // Sets last used Value

      Vr1 = Vr2 = Vr3 = Vr4 = Wr;

      return 0;
    }
    
    int Drive(const int Max_pwm) {                                                            // Drives All Motors with last Set Value

      M1.Drive(Max_pwm);
      M2.Drive(Max_pwm);
      M3.Drive(Max_pwm);
      M4.Drive(Max_pwm);

      return 0;
    }
    
};

/*
class Joystick_custom : public Joystick {

	void Set_K_CosO_SinO(float Extrn_K, float Extrn_CosO, float Extrn_SinO) {
		K = Extrn_K;
		CosO = Extrn_CosO;
		SinO = Extrn_SinO;
	}
	
	int Update() {

		Set_K_CosO_SinO()

		return 0;
	}
};
*/

DC_Motor Motor_arr[4] = {DC_Motor(5, 28), DC_Motor(2,22), DC_Motor(3,24), DC_Motor(4,26,150,20)};

Cytron LineFollower_5k(A0, 10, 350, 10.0);
Joystick_PID<Cytron> PID_5k_Jxy(LineFollower_5k, 350, 1.71,0,0);

Polulo Polulo_QTRRC(QTRSensorsRC((unsigned char[8]){ 37, 39, 41, 43, 45, 47, 49, 51 }, 8, 2500, 53), 7000/2, 5.0);
Joystick_PID<Polulo> PID_Polulo_Jxy(Polulo_QTRRC, 3500, 1.71,0,0);

//QuadBaseDrive QuadBase(PID_Polulo_Jxy, PID_5k_Jxy, Motor_arr, 150);
QuadBaseDrive QuadBase_Polulo(PID_Polulo_Jxy, PID_5k_Jxy, Motor_arr, 150);
QuadBaseDrive QuadBase_Cytron(PID_5k_Jxy, PID_Polulo_Jxy, Motor_arr, 150);

void setup() {
  
 Serial.begin(57600);

 //LineFollower_5k.Initialise();
 PID_5k_Jxy.Initialise();

// QuadBase.Initialise();
QuadBase_Polulo.Initialise();
QuadBase_Cytron.Initialise();

  Polulo_QTRRC.Initialise();
  PID_Polulo_Jxy.Initialise();

  delay(1000);
}
void loop() {
 

//  QuadBase.Read_Drive();
 
// PID_5k_Jxy.Update();
// PID_5k_Jxy.Joystick_Debug();

//Serial.println(LineFollower_5k.Val);

  PID_Polulo_Jxy.Update();
  PID_Polulo_Jxy.Joystick_Debug();

/*
QuadBase_Cytron.Read_Drive();
  delay(20);
  
QuadBase_Polulo.Read_Drive();
  delay(20);
*/
}

/*
int QuadBaseDrive(float K, float CosO, float SinO, float &Vr1, float &Vr2, float &Vr3, float &Vr4){

  Vr1 = K*(((float)1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO);
  Vr2 = K*(((float)1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO);    
  Vr3 = K*(((float)-1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO);    
  Vr4 = K*(((float)-1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO);      
    
   return 0;
}

int Drive(float Vr1, float Vr2, float Vr3, float Vr4){

  M1.DriveMotor(Vr1);
  M2.DriveMotor(Vr2);
  M3.DriveMotor(Vr3);
  M4.DriveMotor(Vr4);

  return 0;
}
*/


