#include "Joystick.h"

Joystick::Joystick(const int Axis_x, const int Axis_y, const int Threshold_x, const int Threshold_y, const int Max_x, const int Max_y)
  : Jx(Axis_x), Jy(Axis_y),
  //      OffstX((Jx != -1)? 0:analogRead(Axis_x)), 
  //      OffstY((Jy != -1)? 0:analogRead(Axis_y)),
  AMax_X(Max_x), AMax_Y(Max_y),
  ThrshldX(Threshold_x), ThrshldY(Threshold_y) {}

int Joystick::Joystick_Input(float &Extrn_K, float &Extrn_CosO, float &Extrn_SinO) {  // Complete Input Function, Gives Amplitude Factor(K), Angles(CosO & SinO)

  Update();
  Joystick_Get(Extrn_K, Extrn_CosO, Extrn_SinO);

  return 0;
}
void Joystick::Initialise() {

  if (Jx != -1) OffstX = analogRead(Jx);
  else         OffstX = 0;
  if (Jy != -1) OffstY = analogRead(Jx);
  else         OffstY = 0;

}
int Joystick::Update() {

  bool Thrshld;

  float AMax;

  Thrshld = Read();
  A_Cos_Sin();

  AMax = Max_Amp();
  Kfactor(AMax);

  /* Debug
  Serial.print(*K);
  Serial.print(", ");
  Serial.print(*CosO);
  Serial.print(", ");
  Serial.print(*SinO);
  Serial.println("    ");
  */
  return Thrshld;
}
int Joystick::Joystick_Get(float &Extrn_K, float &Extrn_CosO, float &Extrn_SinO) {

  Extrn_K = K;
  Extrn_CosO = CosO;
  Extrn_SinO = SinO;

  return 0;
}

int Joystick::Joystick_Debug() {
  float Extrn_K, Extrn_CosO, Extrn_SinO;

  Joystick_Input(Extrn_K, Extrn_CosO, Extrn_SinO);
  Serial.print(JVal_X); Serial.print(", "); Serial.print(JVal_Y); Serial.print(", "); Serial.print(A); Serial.print("   "); Serial.print(Extrn_K); Serial.print(", "); Serial.print(Extrn_CosO); Serial.print(", "); Serial.println(Extrn_SinO);
  //   Serial.print(Jx);Serial.print(", ");Serial.print(Jy);Serial.println("  ");
  //   Serial.print(OffstX);Serial.print(", ");Serial.print(OffstY);Serial.print(", ");Serial.print(ThrshldX);Serial.print(", ");Serial.println(ThrshldY);
  //   Serial.print(AMax_X);Serial.print(", ");Serial.print(AMax_Y);Serial.println("  ");  

  return 0;
}

bool Joystick::Read() {        // Read Input, with Threshold
  int X = 0, Y = 0;
  RawRead(X, Y);

  if (Jx != -1)    X = X - OffstX;
  if (Jy != -1)    Y = Y - OffstY;

  /* Debug
  Serial.print(X);
  Serial.print(", ");
  Serial.prln(Y);
  */

  JVal_X = 0;
  JVal_Y = 0;

  if (abs(X) > ThrshldX)   JVal_X = X;
  if (abs(Y) > ThrshldY)   JVal_Y = Y;

  bool Thrshld = !(abs(X) < ThrshldX) && (abs(Y) < ThrshldY); // Thrshld == false => No Input
  return Thrshld;
}
void Joystick::RawRead(int &X, int &Y) {
  X = analogRead(Jx);
  Y = analogRead(Jy);
}

float Joystick::Max_Amp() {     //  Max Joystick Amplitude in a Given direction

  const float ThrshldCos = 0.005;
  const float ThrshldSin = 0.005;

  float AMax;           // AMax is a function of angle O

  float AMax_b, Cosb, Sinb;             // β - AMax_x, AMax_y Diagonal Angle
  A_Cos_Sin(AMax_X, AMax_Y, AMax_b, Cosb, Sinb);

  if ((SinO < ThrshldSin) && (CosO < ThrshldCos))   AMax = min(AMax_X, AMax_Y);
  if (abs(SinO) < ThrshldSin)                       AMax = AMax_X;
  if (abs(CosO) < ThrshldCos)                       AMax = AMax_Y;

  if (abs(SinO) < abs(Sinb))       AMax = AMax_X / CosO;
  else                             AMax = AMax_Y / SinO;

  // Error correction Due to Offst

  float Offst_A, Offst_CosO, Offst_SinO;
  A_Cos_Sin(OffstX, OffstY, Offst_A, Offst_CosO, Offst_SinO);

  float delAMax = -Offst_A*(Offst_CosO*CosO + Offst_SinO*SinO);

  return AMax - delAMax;
}
float Joystick::Kfactor(float AMax) {     // Kfactor - Amplitude factor        

  K = A / AMax;

  return K;
}

int Joystick::A_Cos_Sin() {                 // Set Amplitude(A),CosO,SinO Values

  return A_Cos_Sin(JVal_X, JVal_Y, A, CosO, SinO);
}
int Joystick::A_Cos_Sin(int X, int Y, float &Amp, float &Cosa, float &Sina) {                 // Set Amplitude(A),CosO,SinO Values

  if ((X == 0.0) && (Y == 0.0)) {
    Cosa = 0.0;
    Sina = 0.0;
    Amp = 0.0;
    return 0;
  }

  Amp = sqrt((long)X * (long)X + (long)Y * (long)Y);
  Cosa = (X / Amp);
  Sina = (Y / Amp);
  /*
  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(*A);
  Serial.print(", "); */
  /*
  Serial.print(*CosO);
  Serial.print(", ");
  Serial.print(*SinO);
  Serial.print("   ");
  */
  return 0;
}
