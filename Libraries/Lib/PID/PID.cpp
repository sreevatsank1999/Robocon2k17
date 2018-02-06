#include <Arduino.h>
#include "../Joystick/Joystick_ver_Arduino/Joystick.h"

template <class PIDObj, class BaseDrive>
class Joystick_PID {

	Virtual_Joystick<Joystick_PID> Jxy;
	Virtual_Joystick<Joystick_PID> Jw;

	const float Kp, Ki, Kd;
	const float Zero_Val;

	PIDObj &PID_Obj;
	BaseDrive &Base;

	float Prop_Val;
	float Intg_Val, Diff_Val;

	unsigned long now_Prev;
public:
	Joystick_PID()
		: Jxy(0, (*this)), Jw(1, (*this)),
		   Kp(p), Ki(i), Kd(d),
		   Zero_Val(zeroVal) {}

  Joystick_PID(PIDObj &pid_Obj, BaseDrive &base_Obj, float zeroVal, float p, float i, float d) 
    : PID_Obj(pid_Obj), Base(base_Obj)
	  Jxy(0, (*this)), Jw(1, (*this)),
      Kp(p), Ki(i), Kd(d),
        Zero_Val(zeroVal)
  {
    Prop_Val = 0.0;
    Intg_Val = 0.0;
    Diff_Val = 0.0;
  }
	void Initialise() {
		Start(PID_Obj.PID_Inp() - Zero_Val);
	}
	void Start(float PVal) {
		Prop_Val = PVal;
		now_Prev = millis();
	}

	int Update(const unsigned int J_ID) {
		PID_Obj.Update();

		float newProp_Val = PID_Obj.PID_Inp() - Zero_Val;

		return Update(newProp_Val);
	}
	int Update(float newPVal) {
		unsigned long now = millis();

		return Update(newPVal, now - now_Prev);
	}
	int Update(float newPVal, float del_t) {
		
		P_CosSin(newPVal, Jxy.CosO, Jxy.SinO);
	    Set_K(Jxy.K);

		float newDiff_Val = D_Wr(newPVal, del_t, Jw.K);
		now_Prev = millis();

		return true;
	}

	int Joystick_Debug() {
		
		Debug_Dev();
		Serial.println("");

		return 0;
	}

private:
	int Debug_Dev() {

		Serial.print(Prop_Val); Serial.print(", "); Serial.print(Diff_Val); Serial.print(", "); Serial.print(Intg_Val); Serial.print(", "); Serial.print(Zero_Val);
		Serial.print("      ");

		Joystick::Debug_Dev();

		return 0;
	}

  int Set_K(float &K) {												// TODO : Make fn. for K (Variable K) 50%-120% Range
	  K = sqrt(2);
    return 0;
  }
	int P_CosSin(float PVal, float &Cosa, float &Sina) {
		
		const float ThrshldCos = 0.005;
		const float ThrshldSin = 0.005;
		const float j = 0.98;

		float Yr = -pow((PVal / j*(PID_Obj.InpMax - Zero_Val)), 3);
//Serial.print(PVal); Serial.print(", "); Serial.print(PID_Obj.InpMax - Zero_Val); Serial.print(", "); Serial.println(Yr);
		Sina = Kp*Yr / sqrt(sq(Kp*Yr) + (float)1);
		Cosa = (float)1 / sqrt(sq(Kp*Yr) + (float)1);

//Serial.print(Cosa); Serial.print(", "); Serial.println(Sina);

		if (abs(Sina) < ThrshldSin)                       Sina = 0.0;
		if (abs(Cosa) < ThrshldCos)                       Cosa = 0.0;

		return 0;
	}
	float D_Wr(float PVal, float del_t, float &Wr) {
		
		const float ThrshldV = 0.05;
		
		float V = Base.Velocity();

		return 0;
	}
	float Integration(float PVal, float del_t) {
		
		Intg_Val += (PVal - Zero_Val )*del_t;
		return Intg_Val;
	}
};
