#include "Motor.h"

DC_Motor::DC_Motor(const int PWM_pin, const int DIR_pin, const float rated_rpm, const float rpm_k, const int Bias_pwm)
	: M_PWM(PWM_pin), M_DIR(DIR_pin), Rated_RPM(rated_rpm),
		rpm_limit_k(rpm_k) , pwm_Bias(Bias_pwm) 
{
	Vr = 0.0;
}
DC_Motor::DC_Motor(const DC_Motor &Motor, const float rpm_k) 
	: M_PWM(Motor.M_PWM), M_DIR(Motor.M_DIR), Rated_RPM(Motor.Rated_RPM),
		rpm_limit_k(rpm_k),	pwm_Bias(Motor.pwm_Bias) 
{
	Vr = 0.0;
}

void DC_Motor::Initialise() {
	pinMode(M_DIR, OUTPUT);
	pinMode(M_PWM, OUTPUT);
}

inline float DC_Motor::Get_rpm() {
	return Vr*rpm_limit_k*Rated_RPM;
}

inline unsigned int DC_Motor::rpm_to_pwm(const float rpm) {	
	const unsigned int MAX_PWM = 255;
	
	return (rpm / Rated_RPM)*MAX_PWM;
}

void DC_Motor::Drive() {
	Drive(Vr, rpm_limit_k);
}
void DC_Motor::Drive(const float k) {
	Vr = k;
	Drive(Vr, rpm_limit_k);
}
void DC_Motor::Drive(const float k, const float rpm_k) {
	// High => CounterClockwise  , Low => Clockwise

	if (k > 0)   digitalWrite(M_DIR, HIGH);
	else          digitalWrite(M_DIR, LOW);

	const int pwm = abs(k) * rpm_to_pwm(rpm_k*Rated_RPM);

	analogWrite(M_PWM, pwm);
}


Wheel::Wheel(const float Diameter)
	:d(Diameter) {}


MyServo::MyServo(const unsigned char Num, const int Ini_alpha, const int Min_alpha, const int Max_alpha, const float k_limit, const float max_W)
	: PinNo(Num),
	Wmax(max_W), W_limit_k(k_limit),
	alpha0(Ini_alpha), alpha_min(Min_alpha), alpha_max(Max_alpha),
	is_enabled(true)
{
	Wr = 0.0;
	alpha = 0;
}

void MyServo::Initialise() {
	(*this).attach(PinNo);
	alpha = alpha0;
	(*this).write(alpha0);
	delay(5);
	LastDriven = millis();

	is_enabled = true;
}

void MyServo::Drive() {
	unsigned long now = millis();

	if (is_enabled == false)      return;

	Drive((float)(now - LastDriven) / (float)1000);
}

bool MyServo::isenabled() {
		return is_enabled;
	}

int MyServo::Debug() {
	Serial.print(Wr); Serial.print(", "); Serial.print(alpha); Serial.print("    ");
	Serial.print(alpha_min); Serial.print(", "); Serial.print(alpha_max); Serial.print(", "); Serial.print(W_limit_k); Serial.println("   ");
}

int MyServo::Drive(const float del_t) {
	float newalpha = alpha + Wr*W_limit_k*(Wmax*(180 / PI))*del_t;

	if (newalpha > alpha_max) { newalpha = alpha_max; }
	if (newalpha < alpha_min) { newalpha = alpha_min; }

	(*this).write(newalpha);
	alpha = newalpha;

	LastDriven = millis();
}


Stepper::Stepper(int Pin_step, int Pin_dir, float resolution, int PulseWidth_min, int PulseWidth_max, float k_limit)
	:StepPin(Pin_step), DirPin(Pin_dir),
	StepResolution(resolution),
	Wmin(PulseWidth_to_W_calc(PulseWidth_max)), Wmax(PulseWidth_to_W_calc(PulseWidth_min)), W_limit_k(k_limit)
{
	stepCount = 0;
}

void Stepper::Initialise() {
	pinMode(DirPin, OUTPUT);
	pinMode(StepPin, OUTPUT);
	LastDriven = micros();
}

void Stepper::Drive() {
	unsigned long now = micros();

	Drive((float)(now - LastDriven) / (float)1000000);
}

float Stepper::Get_W() {
		return Wr*W_limit_k*Wmax;
	}

int Stepper::Debug() {
	Serial.print(Wr); Serial.print(", "); Serial.print(Get_W()); Serial.print(", "); Serial.print(stepCount); Serial.print("    ");
	Serial.print(Wmin); Serial.print(", "); Serial.print(Wmax); Serial.print(", "); Serial.print(W_limit_k); Serial.print("   ");
	Serial.print(W_to_PulseWidth_calc(Get_W())); Serial.println("   ");
}

int Stepper::W_to_PulseWidth_calc(float W) {

	if (abs(W) < Wmin)      return (StepResolution / (2 * Wmin))*(PI / (float)180)*(float)1000000;
	else                    return (StepResolution / (2 * abs(W)))*(PI / (float)180)*(float)1000000;
}

float Stepper::PulseWidth_to_W_calc(const int PulseWidth) {
		return ((float)StepResolution / (float)2 * (float)PulseWidth)*(PI / (float)180);
	}
void Stepper::Drive(const float del_t) {

		int StepNo = (int)((abs(Get_W())*del_t)*((float)180 / PI) / (float)StepResolution);			// Add Estimated time to finish fn also to Reach Exact Current Pos;

		stepCount += StepNo;

		if (Wr > 0)   digitalWrite(DirPin, HIGH);
		else          digitalWrite(DirPin, LOW);

		unsigned int PulseWidth = W_to_PulseWidth_calc(Get_W());

		for (int i = 0; i<abs(StepNo); i++)
		{
			digitalWrite(StepPin, HIGH);
			delayMicroseconds(PulseWidth);
			digitalWrite(StepPin, LOW);
			delayMicroseconds(PulseWidth);
		}
		LastDriven = micros();
     	 // Serial.print(del_t); Serial.print("   ");
	}
