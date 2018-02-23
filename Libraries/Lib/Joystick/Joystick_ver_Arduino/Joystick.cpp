#include "Joystick.h"

Joystick::Joystick() {
	K = 0.0;
	CosO = 0.0;
	SinO = 0.0;
}

int Joystick::Joystick_Get(float &Extern_K) {
	float Extern_CosO, Extern_SinO;

	return Joystick_Get(Extern_K, Extern_CosO, Extern_SinO);
}
int Joystick::Joystick_Get(float &Extern_K, float &Extern_CosO) {
	float Extern_SinO;

	return Joystick_Get(Extern_K, Extern_CosO, Extern_SinO);
}
int Joystick::Joystick_Get(float &Extrn_K, float &Extrn_CosO, float &Extrn_SinO) {	// Complete Input Function, Gives Amplitude Factor(K), Angles(CosO & SinO)

	Extrn_K = K;
	Extrn_CosO = CosO;
	Extrn_SinO = SinO;

	return 0;
}

int Joystick::Joystick_Debug() {
	Debug_Dev();
	Serial.println("");

	return 0;
}
int Joystick::Debug_Dev() {
	
	Serial.print(K); Serial.print(", "); Serial.print(CosO); Serial.print(", "); Serial.print(SinO);

	return 0;
}

Joystick_Sqr::Joystick_Sqr(const int Threshold, const int Max_Val, unsigned char Axis_Sel)
	:AMax_X(Max_Val), AMax_Y(Max_Val),
	ThrshldX(Threshold), ThrshldY(Threshold),
	xy_Slct(Axis_Sel)	{}
Joystick_Sqr::Joystick_Sqr(const int Threshold_x, const int Threshold_y, const int Max_x, const int Max_y)
	: AMax_X(Max_x), AMax_Y(Max_y),
	ThrshldX(Threshold_x), ThrshldY(Threshold_y),
	xy_Slct((J_x_Slct + J_y_Slct))												{}

void Joystick_Sqr::Initialise() {
	OffstX = 0; OffstY = 0;

	RawRead(OffstX, OffstY);
}
int Joystick_Sqr::Update() {

	Read();
	A_Cos_Sin();

	float AMax = Max_Amp();
	Kfactor(AMax);

	return 0;
}

int Joystick_Sqr::Joystick_Debug() {
	Debug_Dev();
	Serial.println("");

	return 0;
}

int Joystick_Sqr::Debug_Dev() {
	

	Serial.print(JVal_X); Serial.print(", "); Serial.print(JVal_Y); Serial.print(", "); Serial.print(A); Serial.print("   "); Joystick::Debug_Dev(); Serial.println("");
	//   Serial.print(Jx);Serial.print(", ");Serial.print(Jy);Serial.println("  ");
	//   Serial.print(OffstX);Serial.print(", ");Serial.print(OffstY);Serial.print(", ");Serial.print(ThrshldX);Serial.print(", ");Serial.println(ThrshldY);
	//   Serial.print(AMax_X);Serial.print(", ");Serial.print(AMax_Y);Serial.println("  ");  

	return 0;
}

bool Joystick_Sqr::Read() {        // Read Input, with Threshold
	int X = 0, Y = 0;
	RawRead(X, Y);

	X -= OffstX;
	Y -= OffstY;

	JVal_X = 0;
	JVal_Y = 0;

	if (abs(X) > ThrshldX)   JVal_X = X;
	if (abs(Y) > ThrshldY)   JVal_Y = Y;

	bool Thrshld = !(abs(X) < ThrshldX) && (abs(Y) < ThrshldY); // Thrshld == false => No Input
	return Thrshld;
}

float Joystick_Sqr::Max_Amp() {     //  Max Joystick Amplitude in a Given direction

	const float ThrshldCos = 0.005;
	const float ThrshldSin = 0.005;

	float AMax;           // AMax is a function of angle O

	float AMax_b, Cosb, Sinb;							// b - AMax_x, AMax_y Diagonal Angle
	A_Cos_Sin(AMax_X, AMax_Y, AMax_b, Cosb, Sinb);

	if ((abs(SinO) < ThrshldSin) && (abs(CosO) < ThrshldCos))   return min(AMax_X, AMax_Y);
	if (abs(SinO) < ThrshldSin)                                 return AMax_X * (abs(CosO) / CosO);
	if (abs(CosO) < ThrshldCos)                                 return AMax_Y * (abs(SinO) / SinO);

	if (abs(SinO) < abs(Sinb))       AMax = AMax_X / CosO;
	else                             AMax = AMax_Y / SinO;

	// Error correction Due to Offst

	float Offst_A, Offst_CosO, Offst_SinO;
	A_Cos_Sin(abs(OffstX) - abs(AMax_X), abs(OffstY) - abs(AMax_Y), Offst_A, Offst_CosO, Offst_SinO);

	float delAMax = -Offst_A*(Offst_CosO*CosO + Offst_SinO*SinO);

	return AMax - delAMax;
}
float Joystick_Sqr::Kfactor(float AMax) {     // Kfactor - Amplitude factor        

	K = A / AMax;

	return K;
}

int Joystick_Sqr::A_Cos_Sin() {                 // Set Amplitude(A),CosO,SinO Values

	return A_Cos_Sin(JVal_X, JVal_Y, A, CosO, SinO);
}
int Joystick_Sqr::A_Cos_Sin(int X, int Y, float &Amp, float &Cosa, float &Sina) {                 // Set Amplitude(A),CosO,SinO Values

	if ((X == 0.0) && (Y == 0.0)) {
		Cosa = 0.0;
		Sina = 0.0;
		Amp = 0.0;
		return 0;
	}

	Amp = sqrt((long)X * (long)X + (long)Y * (long)Y);
	Cosa = (X / Amp);
	Sina = (Y / Amp);

	return 0;
}

JSqr_real::JSqr_real(const int Axis_x, const int Axis_y, const int Threshold_x, const int Threshold_y, const int Max_x, const int Max_y)
	: Jx(Axis_x), Jy(Axis_y),
	  Joystick_Sqr(Threshold_x, Threshold_y, Max_x, Max_y, (J_x_Slct + J_y_Slct))		{}

JSqr_real::JSqr_real(const int pin, const int Threshold, const int Max_Val, unsigned char Axis_Sel)
	: Jx((Axis_Sel&J_x_Slct) ? pin : -1), Jy((Axis_Sel&J_y_Slct) ? pin : -1),
	Joystick_Sqr(Threshold, Max_Val, Axis_Sel)											{}

void JSqr_real::RawRead(int &X, int &Y) {			// Jx(or)Jy = -1 => Axis Disabled hence No read
	if (xy_Slct & J_x_Slct)		X = analogRead(Jx);
	if (xy_Slct & J_y_Slct)		Y = analogRead(Jy);
}