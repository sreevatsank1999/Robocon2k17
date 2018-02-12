#include "../../Libraries/Lib/Joystick/Joystick_ver_Arduino/Joystick.h"
#include "../../Libraries/Lib/DC_Motor/DC_Motor.h"
#include "../../Libraries/Lib/PS2X_lib/PS2X_lib.h"

class TriBaseDrive {                  // C++ Trianglar Base Drive Class

    Joystick &Jxy;
    Joystick &Jw;
    
	MotorAssmbly<DC_Motor> M1, M2, M3;

	const float r;

	const float Vmax, V_limit_k;		// Limit the Max Speed of configuration by a V_limit_k% of Vmax, Vmax depends on the Motors' Vmax
	const float Wmax, W_limit_k;
        
  public:
    TriBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[3], const float R, const float k_Limit)
    : Jxy(xy), Jw(w), 
      M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]),
	  r(R),
	  Vmax(Vmax_clac()), V_limit_k(k_Limit),
	  Wmax(Wmax_calc()), W_limit_k(k_Limit) {}

	TriBaseDrive(Joystick &xy, Joystick &w, MotorAssmbly<DC_Motor> M_Assmbly[3], const float R, const float Vk_Limit, const float Wk_Limit)
		: Jxy(xy), Jw(w),
		M1(M_Assmbly[0]), M2(M_Assmbly[1]), M3(M_Assmbly[2]),
		r(R),
		Vmax(Vmax_clac()), V_limit_k(Vk_Limit),
		Wmax(Wmax_calc()), W_limit_k(Wk_Limit) {}


	void Initialise() {
		M1.Initialise();
		M2.Initialise();
		M3.Initialise();
	}

	inline float Get_V() {								// RealWorld V
		return abs(Jxy.K*V_limit_k)*(1 - abs(Jw.K*W_limit_k))*Vmax;
	}
	inline float Get_Vmax() {							// RealWorld Vmax
		return Vmax*V_limit_k;
	}
	inline float Get_W() {								// RealWorld W
		return abs(Jw.K*W_limit_k)*Wmax;
	}
	inline float Get_Wmax() {							// RealWorld Wmax
		return Wmax*W_limit_k;
	}
	int Read_Update() {

		Jxy.Update();
		Jw.Update();
		Vr_Update(Jxy.K, Jw.K, Jxy.CosO, Jxy.SinO, M1.Vr, M2.Vr, M3.Vr, M4.Vr);

		return 0;
	}
	int Read_Drive() {                    // Realtime Read and Drive Function
		Read_Update();
		Drive();
		return 0;
	}


    int Vr_Update(float K, float Wr, float CosO, float SinO, float &Vr1, float &Vr2, float &Vr3) {      // Sets last used Value

	// Vri => Vi/Vm, Vm - Max Motor Velocity

      Vr1 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k)) * (((float)1 / (float)2) * CosO - (sqrt(3) / 2) * SinO) - Wr*W_limit_k;
      Vr2 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k)) * (-CosO) - Wr*W_limit_k;
      Vr3 = abs(K*V_limit_k)*(1 - abs(Wr*W_limit_k)) * (((float)1 / (float)2) * CosO + (sqrt(3) / 2) * SinO) - Wr*W_limit_k;

      return 0;
    }

	void Debug() {
		Debug_Dev();
		Serial.println("");
	}

private:
	int Drive() {                                                            // Drives All Motors with last Set Value

		M1.Drive();
		M2.Drive();
		M3.Drive();

		return 0;
	}

	inline float Vmax_clac() {
		return min(min(M1.Vmax, M2.Vmax), M3.Vmax);
	}
	inline float Wmax_calc() {
		return Vmax / r;
	}

	void Debug_Dev() {
		Serial.print(M1.Vr); Serial.print(", "); Serial.print(M2.Vr); Serial.print(", "); Serial.print(M3.Vr); Serial.print("   ");
	}
};

class Joystick_PS2 : public Joystick_Analog {
  
   PS2X &PS2_Ctrl;

   const int CLK, CMD, ATT, DAT;
   const bool Pressure, Rumble;
   
public:  
    Joystick_PS2(PS2X &PS2Controller, const int clk, const int cmd, const int att, const int dat, bool pressures, bool rumble, bool LR) 
    : Joystick_Analog(LR?PSS_LX:PSS_RX, LR?PSS_LY:PSS_RY, 50, 50, 128, 128), PS2_Ctrl(PS2Controller),
      CLK(clk), CMD(cmd), ATT(att), DAT(dat), Pressure(pressures), Rumble(rumble)
    {
      //  PS2_Ctrl.config_gamepad(clk, cmd, att, dat, pressures, rumble);
    }
    void RawRead(int &X, int &Y){
      PS2_Ctrl.read_gamepad();
      X = PS2_Ctrl.Analog(Jx);
      Y = PS2_Ctrl.Analog(Jy);
    }
    void Initialise(){
      Config(CLK, CMD, ATT, DAT, Pressure, Rumble);
      Joystick_Analog::Initialise();
    }
private:
   void Config(const int clk, const int cmd, const int att, const int dat, bool pressures, bool rumble){
      PS2_Ctrl.config_gamepad(clk, cmd, att, dat, pressures, rumble);
    }
};
                                                          
                                                         
Joystick_Analog Jw(A3, -1, 50);

PS2X ps2x;                                                    // PS2X +3.3V(Yellow+Red), +5(Ornage), GND(), CLK(Brown+Blue), CMD(Orange+Orange), ATT(Voilet+Yellow), DAT(Yellow+Voilet/Brown)
Joystick_PS2 Jxy(ps2x, 46, 50, 48, 52, true, false, true);    //using left Joystick

MotorAssmbly<DC_Motor> M_Assmbly[3] = { MotorAssmbly<DC_Motor>(DC_Motor(5, 28, 300, 220), Wheel(0.1)), MotorAssmbly<DC_Motor>(DC_Motor(2,22, 300, 220), Wheel(0.1)),
										MotorAssmbly<DC_Motor>(DC_Motor(3,24, 300, 200), Wheel(0.1))};

TriBaseDrive ManualBot(Jxy, Jw, M_Assmbly, 0.36, 1.0);

void setup() {

Serial.begin(57600);

Jxy.Initialise();
ManualBot.Initialise();
}

void loop() {

  Jxy.Update();
  Jxy.Joystick_Debug();

delay(250);
}
