#ifndef DC_Motor_h
#define DC_Motor_h

#include <Arduino.h>

class DC_Motor {                      // C++ DC_Motor Class    // M(PWM_pin, DIR_pin, Rated_RPM, rpm_limit=220, pwm_Bias=0);

	const int M_PWM;
	const int M_DIR;

	const float Rated_RPM;				
	
	const float rpm_limit;
	const int pwm_Bias;

public:
	float Vr;            // Vr => V/Vm, Vm - Max Motor Velocity

	DC_Motor(const int PWM_pin, const int DIR_pin, const float rated_rpm, const float Max_rpm = 220, const int Bias_pwm = 0);
	DC_Motor(const DC_Motor &Motor, const float Max_rpm);

	void Initialise();

	float Get_rpm();
	unsigned int rpm_to_pwm(const float rpm);

	void Drive();
	void Drive(const float k);
	void Drive(const float k, const float rpm);
};
#endif // !DC_Motor_h