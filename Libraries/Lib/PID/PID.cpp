#include <Arduino.h>
#include "../Joystick/Joystick_ver_Arduino/Joystick.h"

template <class Object>
class Joystick_PID : public Joystick{

	const float Kp, Ki, Kd;
	const float Zero_Val;

	Object &PID_Obj;

	float Prev_Val;

	float Intg_Val, Der_Val;
	unsigned long now_Prev;
public:
	Joystick_PID()
    : Joystick(256, 256, 50, 50),
      Kp(0.0), Ki(0.0), Kd(0.0),
      Zero_Val(0.0)
  {
    Prev_Val = 0.0;
    Intg_Val = 0.0;
    Der_Val = 0.0;
  }

  Joystick_PID(Object &Obj, float ini_Val, float p, float i, float d) 
    : Joystick(256, 256, 50, 50),
      PID_Obj(Obj),
      Kp(p), Ki(i), Kd(d),
        Zero_Val(ini_Val)
  {
    Prev_Val = 0.0;
    Intg_Val = 0.0;
    Der_Val = 0.0;
  }
	void Initialise() {
		Start(PID_Obj.PID_Inp());
	}
	void Start(float InpVal) {
		Prev_Val = InpVal;
		now_Prev = millis();
	}

	int Update() {
    PID_Obj.Update();
		return Update(PID_Obj.PID_Inp() - Zero_Val);
	}
	int Update(float InpVal) {
		unsigned long now = millis();

		return Update(InpVal, now - now_Prev);
	}
	int Update(float InpVal, float del_t) {

		P_CosSin(InpVal, CosO, SinO);
    Set_K();
    
		Prev_Val = InpVal;
		now_Prev = millis();

		return true;
	}

  void RawRead(int &X, int &Y){
    
  }
private:
  int Set_K() {
    K = 0.707;
    return 0;
  }
	int P_CosSin(float InpVal, float &Cosa, float &Sina) {
		
		const float ThrshldCos = 0.005;
		const float ThrshldSin = 0.005;
		const float j = 0.98;

		float Yr = -(InpVal / (j*PID_Obj.InpMax));
//Serial.print(InpVal); Serial.print(", "); Serial.print(PID_Obj.InpMax); Serial.print(", "); Serial.println(Yr);
		Sina = Kp*Yr / sqrt(sq(Kp*Yr) + (float)1);
		Cosa = (float)1 / sqrt(sq(Kp*Yr) + (float)1);

//Serial.print(Cosa); Serial.print(", "); Serial.println(Sina);

		if (abs(Sina) < ThrshldSin)                       Sina = 0.0;
		if (abs(Cosa) < ThrshldCos)                       Cosa = 0.0;

		return 0;
	}
	int Differentiation(float InpVal, float del_t) {
		
		
		
		return 0;
	}
	int Integration(float InpVal, float del_t) {
		
		Intg_Val += (InpVal - Zero_Val )*del_t;
		return Intg_Val;
	}
};
