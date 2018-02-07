#include <Arduino.h>
#include "../Joystick/Joystick_ver_Arduino/Joystick.h"

template <class PIDObj, class PIDVehicle>
class Joystick_PID {

	Virtual_Joystick<Joystick_PID> Jxy;
	Virtual_Joystick<Joystick_PID> Jw;

	const float Kp, Ki, Kd;
	const float Yo;

	PIDObj &PID_Obj;
	PIDVehicle &Base;

	float Y;
	float Intg_Val, delY_by_delX;

	unsigned long now_Prev;
public:
	Joystick_PID()
		: Jxy(0, (*this)), Jw(1, (*this)),
		   Kp(p), Ki(i), Kd(d),
		   Yo(0.0) {}

  Joystick_PID(PIDObj &pid_Obj, PIDVehicle &base_Obj, float zeroVal, float p, float i, float d) 
    : PID_Obj(pid_Obj), Base(base_Obj)
	  Jxy(0, (*this)), Jw(1, (*this)),
      Kp(p), Ki(i), Kd(d),
        Yo(zeroVal)
  {
    Y = 0.0;
    Intg_Val = 0.0;
    delY_by_delX = 0.0;
  }
	void Initialise() {
		Start(PID_Obj.PID_Inp() - Yo);
	}
	void Start(float PVal) {
		Y = PVal;
		now_Prev = micros();
	}

	int Update(const unsigned int J_ID) {

		if (J_ID == 1)		return 1;					// Don't Update if Jw.Update() is called

		PID_Obj.Update();

		float newProp_Val = PID_Obj.PID_Inp() - Yo;
		Update(newProp_Val);

		return 0;
	}
	int Update(float newPVal) {
		unsigned long now = micros();

		return Update(newPVal, (now - now_Prev)/1000000);
	}
	int Update(float newPVal, float del_t) {
		
		P_CosSin(newPVal, Jxy.CosO, Jxy.SinO);
	    Set_K(Jxy.K);

		float newdelY_by_delX;
		D_Wr(newPVal, del_t, Jw.K, newdelY_by_delX);

		now_Prev = micros();

		return true;
	}

	int Joystick_Debug() {
		
		Debug_Dev();
		Serial.println("");

		return 0;
	}

private:
	int Debug_Dev() {

		Serial.print(Y); Serial.print(", "); Serial.print(delY_by_delX); Serial.print(", "); Serial.print(Intg_Val); Serial.print(", "); Serial.print(Yo);
		Serial.print("      ");

		Jxy.Debug_Dev(); Serial.print("      "); Jw.Debug_Dev();


		return 0;
	}

  int Set_K(float &K) {												// TODO : Make fn. for K (Variable K) 50%-120% Range
	  K = sqrt(1/2);
    return 0;
  }
	int P_CosSin(float PVal, float &Cosa, float &Sina) {
		
		const float ThrshldCos = 0.005;
		const float ThrshldSin = 0.005;
		const float j = 0.98;

		float Yr = -pow((PVal / j*(PID_Obj.InpMax() - Yo)), 3);
//Serial.print(PVal); Serial.print(", "); Serial.print(PID_Obj.InpMax() - Yo); Serial.print(", "); Serial.println(Yr);
		Sina = Kp*Yr / sqrt(sq(Kp*Yr) + (float)1);
		Cosa = (float)1 / sqrt(sq(Kp*Yr) + (float)1);

//Serial.print(Cosa); Serial.print(", "); Serial.println(Sina);

		if (abs(Sina) < ThrshldSin)                       Sina = 0.0;
		if (abs(Cosa) < ThrshldCos)                       Cosa = 0.0;

		return 0;
	}
	int D_Wr(float Yobs, float del_t, float prev_SinO, float &Wr, float &newdelY_by_delX) {
												// ** All Units in SI Units **
		const float ThrshldV = 0.05;
		
		float V = Base.Get_V();								
		float Wmax = Base.Get_Wmax();

		float Yexp = Y + (V*del_t)*prev_SinO;
															// TODO: Put Threshold on V 
		newdelY_by_delX = -(Yobs - Yexp) / (V*del_t);

		Wr = Kd*(newdelY_by_delX / del_t)*(1 / Wmax);

		return 0;
	}
	float Integration(float PVal, float del_t) {
		
		Intg_Val += (PVal - Yo )*del_t;
		return Intg_Val;
	}
};
