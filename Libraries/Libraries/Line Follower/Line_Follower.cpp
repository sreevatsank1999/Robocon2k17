#include <Arduino.h>

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

public:
	unsigned int Val;
	unsigned int JnCount;
	bool is_Jn;

	CytronLineFollower(const unsigned char AnalogPin, const unsigned char JunctionPin) :ValPin(AnalogPin), JnPin(JunctionPin)
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