#include "DC_Motor.h"

DC_Motor::DC_Motor(const int PWM_pin, const int DIR_pin, const float rated_rpm, const float Max_rpm, const int Bias_pwm)
	: M_PWM(PWM_pin), M_DIR(DIR_pin), Rated_RPM(rated_rpm),
		rpm_limit(Max_rpm) , pwm_Bias(Bias_pwm) 
{
	Vr = 0.0;
}
DC_Motor::DC_Motor(const DC_Motor &Motor, const float Max_rpm) 
	: M_PWM(Motor.M_PWM), M_DIR(Motor.M_DIR), Rated_RPM(Motor.Rated_RPM),
		rpm_limit(Max_rpm),	pwm_Bias(Motor.pwm_Bias) 
{
	Vr = 0.0;
}

void DC_Motor::Initialise() {
	pinMode(M_DIR, OUTPUT);
}

inline float DC_Motor::Get_rpm() {
	return Vr*rpm_limit;
}

inline unsigned int DC_Motor::rpm_to_pwm(const float rpm) {	
	const unsigned int MAX_PWM = 255;
	
	return (rpm / Rated_RPM)*MAX_PWM;
}

void DC_Motor::Drive() {
	Drive(Vr, rpm_limit);
}
void DC_Motor::Drive(const float k) {
	Vr = k;
	Drive(Vr, rpm_limit);
}
void DC_Motor::Drive(const float k, const float Max_rpm) {
	// High => CounterClockwise  , Low => Clockwise

	if (k > 0)   digitalWrite(M_DIR, HIGH);
	else          digitalWrite(M_DIR, LOW);

	const int pwm = abs(k) * rpm_to_pwm(rpm_limit);

	analogWrite(M_PWM, pwm);
}

Wheel::Wheel(const float Diameter = 0.0)
	:d(Diameter) {}

template<class MotorClass>
MotorAssmbly::MotorAssmbly(const MotorClass Motor, const Wheel wheel)
	:MotorClass(Motor), Wheel(wheel),
	Vmax((Rated_RPM / 60)*PI*d)  {}

template<class MotorClass>
MotorAssmbly::MotorAssmbly(const MotorClass Motor, const Wheel wheel, const float Max_V)
	:MotorClass(Motor), Wheel(wheel),
	Vmax(Max_V) {}

template<class MotorClass>
inline float MotorAssmbly::Get_V() {
	return Vr*Vmax;
}
