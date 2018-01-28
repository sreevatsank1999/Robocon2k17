#ifndef DC_Motor_h
#define DC_Motor_h

#include <Arduino.h>

class DC_Motor {                      // C++ DC_Motor Class    // M(PWM_pin, DIR_pin, Max_pwm=220);

	const int M_PWM;
	const int M_DIR;
	const int pwm_Bias;
	
	int pwm_Max;

public:
	float Vr;            // Vr => V/Vm, Vm - Max Motor Velocity

	DC_Motor(const int PWM_pin, const int DIR_pin, const int Max_pwm = 220, const int Bias_pwm = 0);
	DC_Motor(const DC_Motor &Motor);

	void Initialise();

	void Vr_Set(const float k);
	void pwm_Max_Set(const int Extern_pwm);

	void DriveMotor(const float k);
	void DriveMotor(const float k, const int Extern_pwm);
	void Drive(const int Max_pwm);
};
#endif // !DC_Motor_h