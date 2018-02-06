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

    Joystick &Jxy, &Jw;
    
    MotorAssmbly<DC_Motor> M1, M2, M3, M4;

	const float r;	// Distance of Center Of Mass to Motors //TODO : Set Wmax, V_RealWrold Function , W_RealWorld Function And then PIDD

	const float Vmax, V_limit_k;		// Limit the Max Speed of configuration by a V_limit_k% of Vmax, Vmax depends on the Motors' Vmax
	const float Wmax, W_limit_k;


  public:
      QuadBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float Vk_Limit = 1.0, const float Wk_Limit = 1.0)
      : Jxy(xy), Jw(w),
        M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]), M4(M_Assmbly[3]),
		 r(R),
		  Vmax(Vmax_clac()), V_limit_k(Vk_Limit),
		  Wmax(Wmax_calc()), W_limit_k(Wk_Limit)
		  {}

	  QuadBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float k_Limit = 1.0)
		  : Jxy(xy), Jw(w),
		  M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]), M4(M_Assmbly[3]),
		  r(R),
		  Vmax(Vmax_clac()), V_limit_k(k_Limit),
		  Wmax(Wmax_calc()), W_limit_k(k_Limit)
	  {}


    void Initialise(){
      M1.Initialise();
      M2.Initialise();
      M3.Initialise();
      M4.Initialise();
    }

	inline float Get_V() {								// RealWorld V
		return abs(Jxy.K*V_limit_k)*(1 - abs(Jw.K*W_limit_k))*Vmax;
	}
	inline float Get_Vmax() {							// RealWorld Vmax
		return Vmax*V_limit_k;
	}
	inline float Get_W() {								// RealWorld W
		return abs(Jw.K*W_limit_k)*Wmax;
	}
	inline float Get_Wmax() {							// RealWorld Wmax
		return Wmax*W_limit_k;
	}

	int Read_Update() {
		
		Jxy.Update();
		Jw.Update();
		Vr_Update(Jxy.K, Jw.K, Jxy.CosO, Jxy.SinO, M1.Vr, M2.Vr, M3.Vr, M4.Vr);

		return 0;
	}
    int Read_Drive() {                    // Realtime Read and Drive Function
		Read_Update();
		Drive();
    }

   int Vr_Update(float K, float Wr, float CosO, float SinO, float &Vr1, float &Vr2, float &Vr3, float &Vr4) {      // Sets last used Value
																						
	// Vri => Vi/Vm, Vm - Max Motor Velocity

         Vr1 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k))*(((float)1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO) - Wr*W_limit_k;
         Vr2 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k))*(((float)1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO) - Wr*W_limit_k;
         Vr3 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k))*(((float)-1/sqrt(2))*CosO + ((float)1/sqrt(2))*SinO) - Wr*W_limit_k;
         Vr4 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k))*(((float)-1/sqrt(2))*CosO - ((float)1/sqrt(2))*SinO) - Wr*W_limit_k;
    
	   return 0;
    }
    
	void Debug() {
		Debug_Dev();
		Serial.println("");
	}

	private: 
		int Drive() {                                                            // Drives All Motors with last Set Value

			M1.Drive();
			M2.Drive();
			M3.Drive();
			M4.Drive();

			return 0;
		}

	inline float Vmax_clac() {
			return min(M1.Vmax, M2.Vmax, M3.Vmax, M4.Vmax);
		}
	inline float Wmax_calc() {
		return Vmax / r;
	}

	void Debug_Dev() {
		Serial.print(Vr1); erial.print(", ");Serial.print(Vr2);Serial.print(", ");Serial.print(Vr3);Serial.print(", ");Serial.print(Vr4);Serial.print("   ");
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

MotorAssmbly<DC_Motor> M_Assmbly[4] = { MotorAssmbly<DC_Motor>(DC_Motor(5, 28, 300, 220), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(2,22, 300, 220), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(3,24, 300, 200), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(4,26,300,220,20), Wheel(0.1)};

Cytron LineFollower_5k(A0, 10, 350, 10.0);
Joystick_PID<Cytron> PID_5k_Jxy(LineFollower_5k, 350, 1.71,0,0);

Polulo Polulo_QTRRC(QTRSensorsRC((unsigned char[8]){ 37, 39, 41, 43, 45, 47, 49, 51 }, 8, 2500, 53), 7000/2, 5.0);
Joystick_PID<Polulo> PID_Polulo_Jxy(Polulo_QTRRC, 3500, 1.71,0,0);

//QuadBaseDrive QuadBase(PID_Polulo_Jxy, PID_5k_Jxy, Motor_arr, 150);

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