#include "DC_Motor.h"


DC_Motor::DC_Motor(const int PWM_pin, const int DIR_pin, const int Max_pwm, const int Bias_pwm) : M_PWM(PWM_pin), M_DIR(DIR_pin), pwm_Bias(Bias_pwm) {
	pwm_Max = Max_pwm;
	Vr = 0.0;
}
DC_Motor::DC_Motor(const DC_Motor &Motor) : M_PWM(Motor.M_PWM), M_DIR(Motor.M_DIR), pwm_Bias(Motor.pwm_Bias) {
	pwm_Max = Motor.pwm_Max;
	Vr = 0.0;
}

void DC_Motor::Initialise() {
	pinMode(M_DIR, OUTPUT);
}

void DC_Motor::Vr_Set(const float k) {

	Vr = k;
}
void DC_Motor::pwm_Max_Set(const int Extern_pwm) {

	pwm_Max = Extern_pwm;
}

void DC_Motor::DriveMotor(const float k) {
	Vr_Set(k);
	Drive(pwm_Max);
}
void DC_Motor::DriveMotor(const float k, const int Extern_pwm) {
	Vr_Set(k);
	Drive(Extern_pwm);
}
void DC_Motor::Drive(const int Max_pwm) {
	// High => CounterClockwise  , Low => Clockwise

	if (Vr > 0)   digitalWrite(M_DIR, HIGH);
	else          digitalWrite(M_DIR, LOW);

	const int pwm = abs(Vr) * Max_pwm;

	analogWrite(M_PWM, pwm);
}