#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Servo.h>

class DC_Motor {                      // C++ DC_Motor Class    // M(PWM_pin, DIR_pin, Rated_RPM, rpm_limit_k = 1.0, pwm_Bias=0);

	const int M_PWM;
	const int M_DIR;
protected:
	const float Rated_RPM;					// Physical RPM
	
	const float rpm_limit_k;					// Physical RPM
	const int pwm_Bias;

public:
	float Vr;            // Vr => V/Vm, Vm - Max Motor Velocity

	DC_Motor(const int PWM_pin, const int DIR_pin, const float rated_rpm, const float rpm_k = 1.0, const int Bias_pwm = 0);
	DC_Motor(const DC_Motor &Motor, const float rpm_k);

	void Initialise();

	float Get_rpm();
	unsigned int rpm_to_pwm(const float rpm);

	void Drive();
	void Drive(const float k);
	void Drive(const float k, const float rpm);
};

class Wheel {
public:
	const float d;						// Physical Diameter

	Wheel(const float Diameter = 0.0);
};


class MyServo : public Servo {
	const unsigned char PinNo;

	const float Wmax, W_limit_k;			// Wmax = PI/sec;

	const unsigned int alpha0, alpha_min, alpha_max;

	unsigned long LastDriven;

	bool is_enabled;
public:
	float Wr;
	unsigned int alpha;

	MyServo(const unsigned char Num, const int Ini_alpha, const int Min_alpha, const int Max_alpha, const float k_limit, const float max_W);

	void Initialise();

	void Drive();

	bool isenabled();

	int Debug();
private:
	int Drive(const float del_t);
};

class Stepper {
	const int StepPin;
	const int DirPin;

	const float StepResolution;				// deg/step

	const float Wmin, Wmax, W_limit_k;			// Wmax = PI/sec 

	unsigned long LastDriven;
public:
	float Wr;
	long stepCount;

	Stepper(int Pin_step, int Pin_dir, float resolution, int PulseWidth_min, int PulseWidth_max, float k_limit);

	void Initialise();

	void Drive();

	float Get_W();

	int Debug();

private:
	int W_to_PulseWidth_calc(float W);
	float PulseWidth_to_W_calc(const int PulseWidth);

	void Drive(const float del_t);
};


//  ** Template Section **  //

template<class MotorClass>
class MotorAssmbly : public MotorClass, public Wheel {
public:
	const float Vmax;					// Real World Velocity Max(in Units)

	MotorAssmbly(const MotorClass Motor, const Wheel wheel)
		:MotorClass(Motor), Wheel(wheel),
		 Vmax((MotorClass::rpm_limit_k*(MotorClass::Rated_RPM) / 60)*PI*d) {}

	MotorAssmbly(const MotorClass Motor, const Wheel wheel, const float Max_V)
		:MotorClass(Motor), Wheel(wheel),
		 Vmax(Max_V) {}

	MotorAssmbly(MotorAssmbly &MotorAsm, const float Max_V)
		:MotorClass(MotorAsm), Wheel(MotorAsm),
		Vmax(Max_V) {}

	inline float Get_V() {					// Real World Velocity(Realtime)
		return MotorClass::Vr*Vmax;
	}					
};

#endif // !Motor_h