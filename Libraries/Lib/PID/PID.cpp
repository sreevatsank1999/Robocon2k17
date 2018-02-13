#include <Arduino.h>
#include "../Joystick/Joystick_ver_Arduino/Joystick.h"

template <class PIDObj, class PIDVehicle>
class Joystick_PID {
	const float Kp, Ki, Kd;
	const float Yo;

	PIDObj *ptrPID_Obj;
	PIDVehicle *ptrBase;

	float Y;
	float Intg_Y, delY_by_delX;

	unsigned long now_Prev;
public:

	Virtual_Joystick<Joystick_PID> Jxy, Jw;
	
	Joystick_PID(PIDVehicle &base_Obj, float zeroVal, float p, float i, float d)
		: Jxy(0, (*this)), Jw(1, (*this)),
			ptrBase(&base_Obj),
			Kp(p), Ki(i), Kd(d),
			Yo(zeroVal)
	{
		Y = 0.0;
		Intg_Y = 0.0;
		delY_by_delX = 0.0;
	}
	Joystick_PID(PIDObj &pid_Obj, float zeroVal, float p, float i, float d)
		: Jxy(0, (*this)), Jw(1, (*this)),
			ptrPID_Obj(&pid_Obj),
			Kp(p), Ki(i), Kd(d),
			Yo(zeroVal)
	{
		Y = 0.0;
		Intg_Y = 0.0;
		delY_by_delX = 0.0;
	}
  Joystick_PID(PIDObj &pid_Obj, PIDVehicle &base_Obj, float zeroVal, float p, float i, float d)
    : Jxy(0, (*this)), Jw(1, (*this)),
	  ptrPID_Obj(&pid_Obj), ptrBase(&base_Obj),
      Kp(p), Ki(i), Kd(d),
        Yo(zeroVal)
  {
    Y = 0.0;
    Intg_Y = 0.0;
    delY_by_delX = 0.0;
  }
  void attach_Obj(PIDObj &pid_Obj) {
	  ptrPID_Obj = &pid_Obj;
  }
  void attach_Vehicle(PIDVehicle &base_Obj) {
	  ptrBase = &base_Obj;
  }

	void Initialise() {
		Start((*ptrPID_Obj).PID_Inp() - Yo);
	}
	void Start(float PVal) {
		Y = PVal;
		Intg_Y = 0.0;
		delY_by_delX = 0.0;

		now_Prev = micros();
	}
	int Update() {
		return Update(Jxy);
	}
	int Update(Virtual_Joystick<Joystick_PID> &J_caller) {			// Caller Joystick Reference

		if (J_caller.get_ID() == 1)		return 1;					// Don't Update if Jw.Update() is called

		(*ptrPID_Obj).Update();

		float newPVal = (*ptrPID_Obj).PID_Inp() - Yo;
		Update(newPVal);

		return 0;
	}
	int Update(float newPVal) {
		unsigned long now = micros();

		return Update(newPVal, (float)(now - now_Prev)/(float)1000000);
	}
	int Update(float newPVal, float del_t) {
		
		float newIntg_Y;
		I(newPVal, del_t, newIntg_Y);

		float newdelY_by_delX;
		D_Wr(newPVal, del_t, Jxy.SinO, Jw.K, newdelY_by_delX);

		P_CosSin(newPVal, Jxy.CosO, Jxy.SinO);
	    Set_K(Jxy.K);

		Y = newPVal;
		delY_by_delX = newdelY_by_delX;
		Intg_Y = newIntg_Y;

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

		Serial.print(Y); Serial.print(", "); Serial.print(delY_by_delX); Serial.print(", "); Serial.print(Intg_Y); Serial.print(", ");
		Serial.print("      ");

		Jxy.Debug_Dev(); Serial.print("      "); Jw.Debug_Dev();

		return 0;
	}

  int Set_K(float &K) {												// TODO : Make fn. for K (Variable K) 50%-120% Range
	  K = sqrt((float)1/(float)2);
    return 0;
  }
	int P_CosSin(float PVal, float &Cosa, float &Sina) {
		
		const float ThrshldCos = 0.005;
		const float ThrshldSin = 0.005;
		const float j = 0.98;

		float Yr = -pow(((float)PVal / (float)(j*((*ptrPID_Obj).InpMax() - Yo))), 3);

		Sina = Kp*Yr / sqrt(sq(Kp*Yr) + (float)1);
		Cosa = (float)1 / sqrt(sq(Kp*Yr) + (float)1);

		if (abs(Sina) < ThrshldSin)                       Sina = 0.0;
		if (abs(Cosa) < ThrshldCos)                       Cosa = 0.0;

		return 0;
	}
	int D_Wr(float Yobs, float del_t, float prev_SinO, float &Wr, float &newdelY_by_delX) {
												// ** All Units in SI Units **
		const float ThrshldV = 0.05;
		
		float V = (*ptrBase).Get_V();								
		float Wmax = (*ptrBase).Get_Wmax();

		float Yexp = Y + (V*del_t)*prev_SinO;
															
		if (V < ThrshldV)			newdelY_by_delX = 0;
		else						newdelY_by_delX = -(Yobs - Yexp) / (V*del_t);

		Wr = Kd*(newdelY_by_delX / del_t)*(1 / Wmax);

		return 0;
	}
	int  I(float PVal, float del_t, float &newIntg_Y) {
		
		newIntg_Y += Ki*(PVal*del_t);

		return 0;
	}
};
