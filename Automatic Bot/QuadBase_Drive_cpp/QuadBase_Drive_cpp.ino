#include "../../Libraries/Lib/DC_Motor/DC_Motor.h"
#include "../../Libraries/Lib/Joystick/Joystick_ver_Arduino/Joystick.h"
#include "../../Libraries/Lib/PID/PID.cpp"
#include "../../Libraries/Lib/Line Follower/Line_Follower.cpp"
#include <stddef.h>

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

    Joystick *Jxy, *Jw;
    
    MotorAssmbly<DC_Motor> M1, M2, M3, M4;

	const float r;					// Distance of Center Of Mass to Motors 

	const float Vmax, V_limit_k;		// Limit the Max Speed of configuration by a V_limit_k% of Vmax, Vmax depends on the Motors' Vmax
	const float Wmax, W_limit_k;


  public:									// TODO: Add procedure to Manually set Vmax  (Done !!)+
	  QuadBaseDrive(MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float k_Limit)
		  : Jxy(NULL), Jw(NULL), 
		  M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]), M4(M_Assmbly[3]),
		  r(R),
		  Vmax(Vmax_clac()), V_limit_k(k_Limit),
		  Wmax(Wmax_calc()), W_limit_k(k_Limit) {}

	  QuadBaseDrive(MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float k_Limit, const float Max_V_Real)
		  :Jxy(NULL), Jw(NULL), 
		   M1(M_Assmbly[0], Inv_Vmax_clac(Max_V_Real)), M2(M_Assmbly[1], Inv_Vmax_clac(Max_V_Real)),
		   M3(M_Assmbly[2], Inv_Vmax_clac(Max_V_Real)), M4(M_Assmbly[3], Inv_Vmax_clac(Max_V_Real)),
		  r(R),
		  Vmax(Max_V_Real), V_limit_k(k_Limit),
		  Wmax(Wmax_calc()), W_limit_k(k_Limit) {}

	  QuadBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float k_Limit)
		  : Jxy(&xy), Jw(&w),
		  M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]), M4(M_Assmbly[3]),
		  r(R),
		  Vmax(Vmax_clac()), V_limit_k(k_Limit),
		  Wmax(Wmax_calc()), W_limit_k(k_Limit)	{}

      QuadBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[4], const float R, const float k_Limit, const float Max_V_Real)
      : Jxy(&xy), Jw(&w),
        M1(M_Assmbly[0], Inv_Vmax_clac(Max_V_Real)), M2(M_Assmbly[1], Inv_Vmax_clac(Max_V_Real)), 
		M3(M_Assmbly[2], Inv_Vmax_clac(Max_V_Real)), M4(M_Assmbly[3], Inv_Vmax_clac(Max_V_Real)),
		 r(R),
		  Vmax(Max_V_Real), V_limit_k(k_Limit),
		  Wmax(Wmax_calc()), W_limit_k(k_Limit){}

	  void attach_Joystick(Joystick &xy) {
		  Jxy = &xy;
		  Jw = new Virtual_Joystick<void>(0.0, 0.0, 0.0);
	  }
	  void attach_Joystick(Joystick &xy, Joystick &w) {
		  Jxy = &xy;
		  Jw = &w;
	}
	  void detach_Joystick() {
		  Jxy = new Virtual_Joystick<void>(0.0, 0.0, 0.0);
		  Jw = new Virtual_Joystick<void>(0.0, 0.0, 0.0);
	  }

    void Initialise(){
      M1.Initialise();
      M2.Initialise();
      M3.Initialise();
      M4.Initialise();
    }

	inline float Get_V() {								// RealWorld V
		return abs((*Jxy).K*V_limit_k)*(1 - abs((*Jw).K*W_limit_k))*Vmax;
	}
	inline float Get_Vmax() {							// RealWorld Vmax
		return Vmax*V_limit_k;
	}
	inline float Get_W() {								// RealWorld W
		return abs((*Jw).K*W_limit_k)*Wmax;
	}
	inline float Get_Wmax() {							// RealWorld Wmax
		return Wmax*W_limit_k;
	}

	int Read_Update() {
		
		(*Jxy).Update();
		(*Jw).Update();
		Vr_Update((*Jxy).K, (*Jw).K, (*Jxy).CosO, (*Jxy).SinO, M1.Vr, M2.Vr, M3.Vr, M4.Vr);

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
			return min(min(M1.Vmax, M2.Vmax), min(M3.Vmax, M4.Vmax));
		}
	inline float Inv_Vmax_clac(const float Max_V) {
		return Max_V;
	}
	inline float Wmax_calc() {
		return Vmax / r;
	}

	void Debug_Dev() {
		Serial.print(M1.Vr); Serial.print(", ");Serial.print(M2.Vr);Serial.print(", ");Serial.print(M3.Vr);Serial.print(", ");Serial.print(M4.Vr);Serial.print("   ");
	}
};

template<unsigned int N>
class MasterController {
	
	int(*Event[N])(); 
	bool(*Trig[N])();

	int i;

	bool is_enabled;
public:
	MasterController()
		: i(0), is_enabled(true) {}
	MasterController(int(*arrEvent[N])(), bool(*arrTrig_Fn[N])(), const int Start_E = 0)
		:Event(arrEvent), Trig(arrTrig_Fn),
		  i(0), is_enabled(true)  {}

	void attach_Evnt_Trig(int(*arrEvent[N])(), bool(*arrTrig_Fn[N])()) {
		Event = arrEvent;
		Trig = arrTrig_Fn;
	}

	int Update() {
		
		if (is_enabled == false)			return -1;

		bool blnTriggered = Trig[i]();
		if (blnTriggered == true) {
			int jmp_to_i = Event[i]();
			if (jmp_to_i == 0)		i++;
			else					i = jmp_to_i;
		}

		if (i > N - 1)		is_enabled = false;

		return i;
	}
};


MotorAssmbly<DC_Motor> M_Assmbly[4] = { MotorAssmbly<DC_Motor>(DC_Motor(8, 7, 100, 1), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(10, 9, 100, 1), Wheel(0.1)),
MotorAssmbly<DC_Motor>(DC_Motor(6, 5, 100, 1), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(12, 11, 100, 1), Wheel(0.1)) };

//Polulo Polulo_QTRRC((unsigned char[8]){5,6,7,8,9,10,11,12}, 8, 2500, 4, 0.5);
///Joystick_PID<Polulo, QuadBaseDrive> PID_Polulo(0.25, sqrt(3), 0.0, 0.025);

//QuadBaseDrive QuadBase_Polulo(M_Assmbly, 0.4, 1.0);

Cytron Cytron_LSA08(A0, 20, 0.1);
Joystick_PID<Cytron, QuadBaseDrive> PID_Cytron(0.05, sqrt(3), 0.0, 0.0125);

Virtual_Joystick<void> VJxy(0, 0.0, 1.0, 0.0), VJw(1, 0.0, 0.0, 0.0);
Virtual_Joystick<void> VJk_pid(2, 1.0, 1.0, 0.0);

//QuadBaseDrive QuadBase_Cytron(VJxy, VJw, M_Assmbly, 0.4, 1.0);

QuadBaseDrive QuadBase_Cytron(M_Assmbly, 0.4, 1.0);

MasterController<7> Automatic_Bot_Brain;

int(*arrEvent[7])() = { E0, E1, E2, E3, E4, E5, E6 };
bool(*arrTrig_Fn[7])() = { T0, T1, T2, T3, T4, T5, T6 };

void setup() {

	Serial.begin(57600);

	// Polulo_QTRRC.Initialise();
	// PID_Polulo.attach(Polulo_QTRRC, QuadBase_Polulo, VJk_pid);
	// PID_Polulo.Initialise();

	// QuadBase_Polulo.attach_Joystick(PID_Polulo.Jxy, PID_Polulo.Jw);
	//QuadBase_Polulo.Initialise();


	Cytron_LSA08.Initialise();
	PID_Cytron.attach(Cytron_LSA08, QuadBase_Cytron, VJk_pid);
	PID_Cytron.Initialise();

	QuadBase_Cytron.attach_Joystick(PID_Cytron.Jxy, PID_Cytron.Jw);
	QuadBase_Cytron.Initialise();

	Automatic_Bot_Brain.attach_Evnt_Trig(arrEvent, arrTrig_Fn);

	delay(3000);
}
void loop() {

	// Cytron_LSA08.Update();
	// Cytron_LSA08.Debug();

	// PID_Cytron.Update();
	// PID_Cytron.Joystick_Debug();

	//  PID_Polulo.Update();
	//  PID_Polulo.Joystick_Debug();


	QuadBase_Cytron.Read_Drive();
	//QuadBase_Cytron.Read_Update();
	//QuadBase_Cytron.Debug();

	//QuadBase_Polulo.Read_Drive();

	Automatic_Bot_Brain.Update();

	delay(20);
}


/////////////////////////////////////////////////////////*********Trig And Event**********////////////////////////////////////////////////////////////////////////////////////////////////

//**********************Important Fns********************************//

int Do_Nothing() { return 0; }
bool No_Trig() { return true; }

template <class LineFollower>
bool Jn_Trig(LineFollower &Obj) {
	static unsigned int oldJnCount = 0;

	if (Obj.JnCount = (oldJnCount + 1)) { oldJnCount = Obj.JnCount; return true; }
	else					return false;
}

void Rotate_Set(QuadBaseDrive &A, const float Wr) {
	VJxy.Update(0.0, 1.0, 0.0); VJw.Update(Wr, 0.0, 0.0);
	A.attach_Joystick(VJxy, VJw);
}

//**********************Trig And Event Fns*************************//

bool T0() { return No_Trig(); }
int  E0() { return Do_Nothing(); }
bool T1() { return Jn_Trig<Cytron>(Cytron_LSA08); }
int  E1() { PID_Cytron.disable();  Rotate_Set(QuadBase_Cytron, 0.85); return 0; }
bool T2() { return Jn_Trig<Cytron>(Cytron_LSA08);}
int  E2() { PID_Cytron.Initialise(); QuadBase_Cytron.attach_Joystick(PID_Cytron.Jxy, PID_Cytron.Jw); VJk_pid.Update(+1.0, 1.0, 0.0); return 0; }
bool T3() { return Jn_Trig<Cytron>(Cytron_LSA08);}
int  E3() { return Do_Nothing(); }
bool T4() { return Jn_Trig<Cytron>(Cytron_LSA08); }
int  E4() { PID_Cytron.disable();  Rotate_Set(QuadBase_Cytron, 0.85); return 0; }
bool T5() { return Jn_Trig<Cytron>(Cytron_LSA08); }
int  E5() { PID_Cytron.Initialise(); QuadBase_Cytron.attach_Joystick(PID_Cytron.Jxy, PID_Cytron.Jw); VJk_pid.Update(+1.0, 1.0, 0.0); return 0; }
bool T6() { return Jn_Trig<Cytron>(Cytron_LSA08); }
int  E6() { QuadBase_Cytron.detach_Joystick(); return 0; }
