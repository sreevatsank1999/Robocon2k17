#ifndef Joystick_h 
#define Joystick_h

#include <Arduino.h>

class Joystick {                      // C++ Joystick Class  // Joystick(Axis_x, Axis_y = -1, Threshold_x = 50, Threshold_y = 50, Max_x = 512, Max_y = 512);
protected:
  int JVal_X,JVal_Y; float A;

  const int Jx, Jy;

  int OffstX, OffstY;

  const int AMax_X, AMax_Y; 

  const int ThrshldX, ThrshldY;

public:
  float K;
  float CosO;
  float SinO;

  Joystick(const int Axis_x, const int Axis_y = -1, const int Threshold_x = 50, const int Threshold_y = 50, const int Max_x = 512, const int Max_y = 512);

  int Joystick_Get(float &Extern_K);
  int Joystick_Get(float &Extern_K, float &Extern_CosO);
  int Joystick_Get(float &Extrn_K, float &Extrn_CosO, float &Extrn_SinO);    // Complete Input Function, Gives Amplitude Factor(K), Angles(CosO & SinO)
  virtual void Initialise();
 
  virtual int Update();

  int Joystick_Debug(); 

protected:
  bool Read();
  virtual void RawRead(int &X, int &Y);
  virtual float Max_Amp();
  float Kfactor(float AMax);

  int A_Cos_Sin();
  int A_Cos_Sin(int X, int Y, float &Amp, float &Cosa, float &Sina);
};

class Joystick_Analog : public Joystick {
	
public:	
	Joystick_Analog(const int Axis_x, const int Axis_y = -1, const int Threshold_x = 50, const int Threshold_y = 50, const int Max_x = 512, const int Max_y = 512);
	void RawRead(int &X, int &Y);
};
#endif // !Joystick_h 
