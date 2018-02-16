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
	
	int Debug_Dev();

protected:
	bool Read();
	virtual void RawRead(int &X, int &Y);
	virtual float Max_Amp();
	float Kfactor(float AMax);

	int A_Cos_Sin();
	int A_Cos_Sin(int X, int Y, float &Amp, float &Cosa, float &Sina);
};

//  ** Template Section **  //

template <class ParentObj>
class Virtual_Joystick : public Joystick {
protected:
	ParentObj *ptrParent;
	unsigned int ID;

public:
	Virtual_Joystick()
		:ID(0), ptrParent(NULL) {}

	Virtual_Joystick(const float extern_K, const float extern_CosO, const float extern_SinO, const unsigned int id = 0)
		:ID(id), ptrParent(NULL) 
	{
		K = extern_K; CosO = extern_CosO; SinO = extern_SinO;
	}

	Virtual_Joystick(const float extern_K, const float extern_CosO, const float extern_SinO, const unsigned int id, ParentObj &P)
		:ID(id), ptrParent(&P) 
	{
		K = extern_K; CosO = extern_CosO; SinO = extern_SinO;
	}

	Virtual_Joystick(const unsigned int id, ParentObj &P) 
		:ID(id), ptrParent(&P) {}

	virtual int Update() {
		if(ptrParent != NULL)		(*ptrParent).Update((*this));
		return 0;
	}

	virtual int Update(const float extern_K, const float extern_CosO, const float extern_SinO) {
		K = extern_K;
		CosO = extern_CosO;
		SinO = extern_SinO;
	}

	void set_ID(unsigned int &id) {
		ID = id;
	}
	inline unsigned  int get_ID() {
		return ID;
	}
	void attach_Parent(ParentObj &P) {
		ptrParent = &P;
	}
	void detach_Parent() {
		ptrParent = NULL;
	}
};

template <> class Virtual_Joystick<void> : public Joystick {
protected:
	unsigned int ID;

public:
	Virtual_Joystick()
		:ID(0) {}

	Virtual_Joystick(const unsigned int id, const float extern_K = 0.0, const float extern_CosO = 0.0, const float extern_SinO = 0.0)
		:ID(id)
	{
		K = extern_K; CosO = extern_CosO; SinO = extern_SinO;
	}

	virtual int Update() {
								// Do Nothing !!
		return 0;
	}

	virtual int Update(const float extern_K, const float extern_CosO, const float extern_SinO) {
		K = extern_K;
		CosO = extern_CosO;
		SinO = extern_SinO;
	}

	void set_ID(unsigned int &id) {
		ID = id;
	}
	inline unsigned  int get_ID() {
		return ID;
	}
};
#endif // !Joystick_h 