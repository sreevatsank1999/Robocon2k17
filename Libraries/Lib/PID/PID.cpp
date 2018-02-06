#include <Arduino.h>
#include "../Joystick/Joystick_ver_Arduino/Joystick.h"

template <class Object>
class Joystick_PID : public Joystick{

	const float Kp, Ki, Kd;
	const float Zero_Val;

	Object &PID_Obj;

	float Prop_Val;
	float Intg_Val, Diff_Val;

	unsigned long now_Prev;
public:
  Joystick_PID(Object &Obj, float ini_Val, float p, float i, float d) 
    : PID_Obj(Obj),
      Kp(p), Ki(i), Kd(d),
        Zero_Val(ini_Val)
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

	int Update() {
		PID_Obj.Update();

		Prop_Val = PID_Obj.PID_Inp() - Zero_Val;

		return Update(Prop_Val);
	}
	int Update(float PVal) {
		unsigned long now = millis();

		return Update(PVal, now - now_Prev);
	}
	int Update(float PVal, float del_t) {

		P_CosSin(PVal, CosO, SinO);
	    Set_K();

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

  int Set_K() {
    K = 0.707;
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
	int Differentiation(float PVal, float del_t) {
		
		
		
		return 0;
	}
	int Integration(float PVal, float del_t) {
		
		Intg_Val += (PVal - Zero_Val )*del_t;
		return Intg_Val;
	}
};
