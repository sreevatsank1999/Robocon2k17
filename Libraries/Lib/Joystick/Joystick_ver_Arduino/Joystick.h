#ifndef Joystick_h 
#define Joystick_h

#include <Arduino.h>

class Joystick {                      // C++ Joystick Class  // Joystick(Axis_x, Axis_y = -1, Threshold_x = 50, Threshold_y = 50, Max_x = 512, Max_y = 512);
public:
	float K;
	float CosO;
	float SinO;

	Joystick();

	int Joystick_Get(float &Extern_K);
	int Joystick_Get(float &Extern_K, float &Extern_CosO);
	int Joystick_Get(float &Extrn_K, float &Extrn_CosO, float &Extrn_SinO);    // Complete Input Function, Gives Amplitude Factor(K), Angles(CosO & SinO)

	virtual int Update();

	virtual int Joystick_Debug();
protected:
	int Debug_Dev();
};

class Joystick_Analog : public Joystick {
protected:
	int JVal_X, JVal_Y; float A;

	const int Jx, Jy;

	int OffstX, OffstY;

	const int AMax_X, AMax_Y;

	const int ThrshldX, ThrshldY;

public:
	Joystick_Analog(const int Axis_x, const int Axis_y = -1, const int Threshold_x = 50, const int Threshold_y = 50, const int Max_x = 512, const int Max_y = 512);

	virtual void Initialise();
	virtual int Update();
	virtual int Joystick_Debug();

protected:
	int Debug_Dev();

	bool Read();
	virtual void RawRead(int &X, int &Y);
	virtual float Max_Amp();
	float Kfactor(float AMax);

	int A_Cos_Sin();
	int A_Cos_Sin(int X, int Y, float &Amp, float &Cosa, float &Sina);
};

template <class ParentObj>
class Virtual_Joystick : public Joystick {
protected:
	const unsigned int ID;

	ParentObj &Parent;

public:
	Virtual_Joystick(const unsigned int id, ParentObj &P);

	virtual int Update();
};
#endif // !Joystick_h 