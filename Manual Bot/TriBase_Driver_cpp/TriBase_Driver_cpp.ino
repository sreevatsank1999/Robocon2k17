#include "Joystick.h"
#include "DC_Motor.h"
#include "PS2X_lib.h"

class TriBaseDrive {                  // C++ Trianglar Base Drive Class

    Joystick &Jxy;
    Joystick &Jw;
    
    DC_Motor &M1;
    DC_Motor &M2;
    DC_Motor &M3;

    int pwm_Max;
    
  public:
    TriBaseDrive(Joystick &xy, Joystick &w, DC_Motor Motor[3], const int Max_pwm = 220)
    : Jxy(xy), Jw(w), 
      M1(Motor[0]), M2(Motor[1]), M3(Motor[2]) 
    {
/*      M1 = new DC_Motor(Motor[0], Max_pwm);
      M2 = new DC_Motor(Motor[1], Max_pwm);
      M3 = new DC_Motor(Motor[2], Max_pwm);

      Jxy = new Joystick(xy);
      Jw  = new Joystick(w);
*/    
        M1.pwm_Max_Set(Max_pwm);
        M2.pwm_Max_Set(Max_pwm);
        M3.pwm_Max_Set(Max_pwm);

        pwm_Max = Max_pwm;
    }

    int Read_Drive(){                                     // Realtime Read and Drive Function with default pwm_Max
      return Read_Drive(pwm_Max);
    }
    int Read_Drive(const int Max_pwm) {                    // Realtime Read and Drive Function

      Linear_Drive(Max_pwm);
      delay(25);

 //     Rotate_Drive(Max_pwm);
 //     delay(25);

      return 0;
    }

    int Linear_Drive(){                                    // Realtime Read and Drive Function with default pwm_Max
      return   Linear_Drive(pwm_Max);
    }
    int Linear_Drive(const int Max_pwm) {                 // Realtime Read and Drive Function

      Jxy.Update();
      Linear_Set(Jxy.K, Jxy.CosO, Jxy.SinO, M1.Vr, M2.Vr, M3.Vr);
      Drive(Max_pwm);

      return 0;
    }
    int Rotate_Drive(){                                 // Realtime Read and Drive Function with default pwm_Max
      return Rotate_Drive(pwm_Max);
    }
    int Rotate_Drive(const int Max_pwm) {                     // Realtime Read and Drive Function

      Jw.Update();
      float Wr = Jw.K;
      // Vri => Vi/Vm, Vm - Max Motor Velocity
      Rotate_Set(Wr, M1.Vr, M2.Vr, M3.Vr);
      Drive(Max_pwm);

      return 0;
    }

    int Linear_Set(float K, float CosO, float SinO, float &Vr1, float &Vr2, float &Vr3) {      // Sets last used Value
  /*    
        Serial.print(CosO);
        Serial.print(", ");
        Serial.print(SinO);
        Serial.print(", ");
        Serial.print(K);
        Serial.println("   ");
      */

      Vr1 = K * (((float)1 / (float)2) * CosO - (sqrt(3) / 2) * SinO);
      Vr2 = K * (-CosO);
      Vr3 = K * (((float)1 / (float)2) * CosO + (sqrt(3) / 2) * SinO);

        Serial.print(Vr1);
        Serial.print(", ");
        Serial.print(Vr2);
        Serial.print(", ");
        Serial.print(Vr3);
        Serial.println("   ");
      return 0;
    }
    int Rotate_Set(float Wr, float &Vr1, float &Vr2, float &Vr3) {                             // Sets last used Value

      Vr1 = Vr2 = Vr3 = Wr;

      return 0;
    }
    int Drive(const int Max_pwm) {                                                            // Drives All Motors with last Set Value

      M1.Drive(Max_pwm);
      M2.Drive(Max_pwm);
      M3.Drive(Max_pwm);

      return 0;
    }
};

class Joystick_PS2 : public Joystick {
  
   PS2X &PS2_Ctrl;

   const int CLK, CMD, ATT, DAT;
   const bool Pressure, Rumble;
   
public:  
    Joystick_PS2(PS2X &PS2Controller, const int clk, const int cmd, const int att, const int dat, bool pressures, bool rumble, bool LR) 
    : Joystick(LR?PSS_LX:PSS_RX, LR?PSS_LY:PSS_RY, 50, 50, 128, 128), PS2_Ctrl(PS2Controller),
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
      Joystick::Initialise();
    }
private:
   void Config(const int clk, const int cmd, const int att, const int dat, bool pressures, bool rumble){
      PS2_Ctrl.config_gamepad(clk, cmd, att, dat, pressures, rumble);
    }
};
                                                          
                                                          //******* Color Convention*********
                                                         // Arduino GND(Green), Vin(Red)

//Joystick_Analog Jxy(A0, A1, 50, 50), Jw(A3, -1, 50);
Joystick_Analog Jw(A3, -1, 50);

PS2X ps2x;                                                    // PS2X +3.3V(Yellow+Red), +5(Ornage), GND(), CLK(Brown+Blue), CMD(Orange+Orange), ATT(Voilet+Yellow), DAT(Yellow+Voilet/Brown)
Joystick_PS2 Jxy(ps2x, 46, 50, 48, 52, true, false, true);    //using left Joystick

DC_Motor Motor_arr[3] = {DC_Motor(8, 5), DC_Motor(9, 6), DC_Motor(10, 7)};       //DC_Motor(PWM, Dir);    //M1(Brown,Red) GND(Ornage), M2(Purple,Grey) GND(White), M3(Green,Blue) GND(Purple+Blue)
TriBaseDrive ManualBot(Jxy, Jw, Motor_arr, 220);

void setup() {

Serial.begin(57600);

Jxy.Initialise();
}

void loop() {

 // ManualBot.Read_Drive(220);
  Jxy.Update();
  Jxy.Joystick_Debug();

delay(25);
}
