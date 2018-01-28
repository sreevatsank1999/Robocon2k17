#include <Arduino.h>
#include "../QTRSensors/QTRSensors.h"

class IR {
protected:
	const unsigned char PinNo;
public:
	IR(const unsigned char No) :PinNo(No) {}

	void Initialise() {
		pinMode(PinNo, INPUT);
	}

	virtual void Read();
};

class IRAnalog : public IR {
public:
	unsigned int Val;

	IRAnalog(const unsigned char No) :IR(No) {}

	void Read() {
		Val = analogRead(PinNo);
	}

};

class IRDigital : public IR {
public:
	bool Val;

	IRDigital(const unsigned char No) :IR(No) {}

	void Read() {
		Val = digitalRead(PinNo);
	}


};

class IRArrayDig {
	IRDigital IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7;

public:
	IRArrayDig(const unsigned char arrIRPin[8])
		:IR0(arrIRPin[0]), IR1(arrIRPin[1]), IR2(arrIRPin[2]), IR3(arrIRPin[3]), IR4(arrIRPin[4]), IR5(arrIRPin[5]), IR6(arrIRPin[6]), IR7(arrIRPin[7])
	{}


	void Initialise() {
		IR0.Initialise();
		IR1.Initialise();
		IR2.Initialise();
		IR3.Initialise();
		IR4.Initialise();
		IR5.Initialise();
		IR6.Initialise();
		IR7.Initialise();
	}
	void Update() {
		IR0.Read();
		IR1.Read();
		IR2.Read();
		IR3.Read();
		IR4.Read();
		IR5.Read();
		IR6.Read();
		IR7.Read();
	}
	float PID_Inp() {
		return ((-4)*IR0.Val + (-3)*IR1.Val + (-2)*IR2.Val + (-1)*IR3.Val + 1 * IR4.Val + 2 * IR5.Val + 3 * IR6.Val + 4 * IR7.Val);
	}
};

class CytronLineFollower {
	const unsigned char ValPin;
	const unsigned char JnPin;

	const float l;					//Module Total length in any Units
public:
	unsigned int Val;
	unsigned int JnCount;
	bool is_Jn;

	CytronLineFollower(const unsigned char AnalogPin, const unsigned char JunctionPin, const float len) 
		:ValPin(AnalogPin), JnPin(JunctionPin), l(len)
	{
		JnCount = 0;
	}

	void Update() {
		Val = analogRead(ValPin);
		is_Jn = digitalRead(JnPin);

		JnCount += is_Jn;
	}

	float PID_Inp() {
		return Val;
	}
};

class Polulo : public QTRSensorsRC {

public:
	unsigned int LinePos;
	const float InpMax;

	Polulo(QTRSensorsRC &qtrrc, const float MaxInp) : QTRSensorsRC(qtrrc), InpMax(MaxInp) {}

	void Initialise() {

		delay(500);
		pinMode(13, OUTPUT);
		digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
		for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
		{
			(*this).calibrate(QTR_EMITTERS_ON);       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
		}
		digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

		delay(1000);
	}

	void Update() {

		unsigned int sensorValues[8];

		LinePos = (*this).readLine(sensorValues, QTR_EMITTERS_ON, 0);
		//   Serial.println(LinePos);
	}

	float PID_Inp() {
		return LinePos;
	}
};