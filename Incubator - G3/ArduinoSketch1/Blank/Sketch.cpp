/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */
#include <SimpleDHT.h>
#include <HX711.h>

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>
#include <LiquidCrystal_I2C.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

// for DHT11,
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT11 = 2;

SimpleDHT11 myDHT;
HX711 scale;

// Variable for DHT11
byte temperature = 0;
byte humidity = 0;

// Variable for MQS
const byte MQ131_PIN = 0;
const byte MQ2_PIN = 0;
const byte MQ135_PIN = 0;

typedef struct RoTypeDef {
	float MQ2;
	float MQ131;
	float MQ135;
};

typedef struct MQ2TypeDef {
	float methane;
	float lpg;
	float co;
	float h2;
	float alcohol;
	float propane;
};

typedef struct MQ131TypeDef {
	float NOx;
	float CL2;
	float O3;
};

// Since the MQ135 data sheet is not clear on the other three
// parameters this code will only be used to measure C02, CO and NH4
typedef struct MQ135TypeDef {
	float CO2;
	float CO;
	float NH4;
};

typedef struct SensorCurve {
	float x;	// x coordinate
	float y;	// y coordinate
	float m;	// slope
};

enum GAS_ID {
	MQ2_LPG,
	MQ2_CO,
	MQ2_METHANE,
	MQ2_H2,
	MQ2_ALCOHOL,
	MQ2_PROPANE,
	MQ131_NOx,
	MQ131_CL2,
	MQ131_O3,
	MQ135_CO2,
	MQ135_CO,
	MQ135_NH4
};

enum {
	MQ2_SENSOR,
	MQ131_SENSOR,
	MQ135_SENSOR
};

SensorCurve LPGCurve = (const struct SensorCurve) {
	2.3, 0.21, -0.47
};

SensorCurve COCurve = (const struct SensorCurve) {
	2.3, 0.72, -0.34
};

SensorCurve MethaneCurve = (const struct SensorCurve) {
	2.3, 0.47, -0.37
};

SensorCurve H2Curve = (const struct SensorCurve) {
	2.3, 0.32, -0.53
};

SensorCurve AlcoholCurve = (const struct SensorCurve) {
	2.3, 0.46, -0.41
};

SensorCurve PropaneCurve = (const struct SensorCurve) {
	2.3, 0.23, -0.45
};

SensorCurve NitrogenOxideCurve = (const struct SensorCurve) {
	0.70, 0.95, -0.31
};

SensorCurve ChlorideCurve = (const struct SensorCurve) {
	0.70, 0.90, -0.91
};

SensorCurve Ozone_03Curve = (const struct SensorCurve) {
	0.70, 0.81, -0.90
};

SensorCurve CarbonDioxideCurve = (const struct SensorCurve) {
	1, 0.34, -0.33
};

SensorCurve CarbonMonooxideCurve = (const struct SensorCurve) {
	1, 0.46, -0.24
};

SensorCurve MQ135MethaneCurve = (const struct SensorCurve) {
	1, 0.41, -0.41
};

int RL_VALUE = 1; // define the load resistance on the board, in kilo ohms

int MQ2_RO_CLEAN_AIR_FACTOR = 9.8;
int MQ131_RO_CLEAN_AIR_FACTOR = 15;
int MQ135_RO_CLEAN_AIR_FACTOR = 4.8;

int CALIBARAION_SAMPLE_TIMES = 50;
int CALIBRATION_SAMPLE_INTERVAL = 50;
int READ_SAMPLE_INTERVAL = 50;
int READ_SAMPLE_TIMES = 128;

RoTypeDef Ro = (RoTypeDef) {
	0,0,0
};

// Read temperature from LM35 sensor
float readTemperature(byte lm35pin) {
	return (analogRead(lm35pin) * 0.488);
}

// Read temperature/humidity from sensor
float readTemptratureHumidity() {
	int err = SimpleDHTErrSuccess;
	if ((err = myDHT.read(pinDHT11, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
		delay(1000);
		return -1;
	}

// DHT11 sampling rate is 1HZ.
	delay(1500);
	return 1;
}

// Control fan speed
void controlFanSpeed(uint8_t fanspeed) {
	uint16_t sp = map(fanspeed, 0, 100, 0, 255);
	analogWrite(3, sp);
}

// Control heater element
void controlHeaterElement(uint8_t HeaterTemp) {
	uint16_t ht = map(HeaterTemp, 0, 100, 0, 255);
	analogWrite(3, ht);
}

// Measure Blood Pressure
// This is not complete since we don't have a dedicated BP apparatus to use
void monitorBloodPressure(HardwareSerial ser) {
	char InByte = 0x00;
	if(ser.available()) {
		InByte = ser.read();
	}
}

/******************************** lighControl***************************************
  Input: None
 Output: Control LED GPIO pin
Remarks: Set LED pin when design of schematic is complete
************************************************************************************/
void lightControl(byte pin) {

}

/******************************** getWeight ****************************************
  Input: None
 Output: the calculated weight from the load cell
Remarks: None
************************************************************************************/
uint16_t getWeight() {
	return scale.read();
}

/****************** MQResistanceCalculation ****************************************
  Input: raw_adc - raw value read from adc, which represents the voltage
 Output: the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/

float MQResistanceCalculation(int raw_adc) {
	float rl = (float) RL_VALUE;
	float MQ_Res = 1023/raw_adc;
	MQ_Res -= 1;
	MQ_Res *= rl;
	return MQ_Res;
}

/*****************************  MQRead *********************************************
  Input: mq_pin - analog channel
 Output: Rs of the sensor
Remarks: This function use MQResistanceCalculation to calculate the sensor resistance (Rs).
         The Rs changes as the sensor is in the different concentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/

float MQRead(byte pin) {
	int i;
	float rs = 0;
	int val = analogRead(pin);
	for (i = 0; i < READ_SAMPLE_TIMES; i++) {
		rs += MQResistanceCalculation(val);
		delayMicroseconds(READ_SAMPLE_INTERVAL);
	}
	rs = rs/READ_SAMPLE_TIMES;
	return rs;
}

/*****************************  MQGetPercentage **********************************
  Input: rs_ro_ratio - Rs divided by Ro
              pcurve - pointer to the curve of the target gas
 Output: ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/

int MQGetPercentage(float rs_ro_ratio, SensorCurve argv) {
	return(pow(10,(((log(rs_ro_ratio) - argv.y) / argv.m) + argv.x)));
}

/*****************************  MQGetGasPercentage **********************************
  Input: rs_ro_ratio - Rs divided by Ro
              gas_id - target gas type
 Output: ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/

float MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
	float gasPercentage = 0;
	switch (gas_id) {
		case MQ2_LPG:
			gasPercentage = MQGetPercentage(rs_ro_ratio, LPGCurve);
			break;

		case MQ2_CO:
			gasPercentage = MQGetPercentage(rs_ro_ratio, COCurve);
			break;

		case MQ2_METHANE:
			gasPercentage = MQGetPercentage(rs_ro_ratio, MethaneCurve);
			break;

		case MQ2_H2:
			gasPercentage = MQGetPercentage(rs_ro_ratio, H2Curve);
			break;

		case MQ2_ALCOHOL:
			gasPercentage = MQGetPercentage(rs_ro_ratio, AlcoholCurve);
			break;

		case MQ2_PROPANE:
			gasPercentage = MQGetPercentage(rs_ro_ratio, PropaneCurve);
			break;

		case MQ131_NOx:
			gasPercentage = MQGetPercentage(rs_ro_ratio, NitrogenOxideCurve);
			break;

		case MQ131_CL2:
			gasPercentage = MQGetPercentage(rs_ro_ratio, ChlorideCurve);
			break;

		case MQ131_O3:
			gasPercentage = MQGetPercentage(rs_ro_ratio, Ozone_03Curve);
			break;

		case MQ135_CO2:
			gasPercentage = MQGetPercentage(rs_ro_ratio, CarbonDioxideCurve);
			break;

		case MQ135_CO:
			gasPercentage = MQGetPercentage(rs_ro_ratio, CarbonMonooxideCurve);
			break;

		case MQ135_NH4:
			gasPercentage = MQGetPercentage(rs_ro_ratio, MQ135MethaneCurve);
			break;
	}
	return gasPercentage;
}

/***************************** MQCalibration ****************************************
  Input: mq_pin - analog channel
 Output: Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/

float MQCalibration(byte pin, byte _type) {
	float val = 0;
	for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
		// take multiple samples
		val += MQResistanceCalculation(analogRead(pin));
		delay(CALIBRATION_SAMPLE_INTERVAL);
	}
	val /= CALIBARAION_SAMPLE_TIMES; // calculate the average value

	switch(_type) {
		case MQ2_SENSOR:
			val = val/MQ2_RO_CLEAN_AIR_FACTOR;     // divided by RO_CLEAN_AIR_FACTOR yields the Ro
			break;

		case MQ131_SENSOR:
			val = val/MQ131_RO_CLEAN_AIR_FACTOR;   // divided by RO_CLEAN_AIR_FACTOR yields the Ro
			break;

		case MQ135_SENSOR:
			val = val/MQ135_RO_CLEAN_AIR_FACTOR;   // divided by RO_CLEAN_AIR_FACTOR yields the Ro
			break;
	}
	// according to the chart in the data sheet
	return val;
}

MQ2TypeDef MQ2Read(byte pin) {
	float rs_ro = MQRead(pin) / Ro.MQ2;

	// Declare structure variable for reading MQ2
	MQ2TypeDef mq2data = (const struct MQ2TypeDef) {
		0, 0, 0, 0, 0, 0
	};

	mq2data.lpg = MQGetGasPercentage(rs_ro, MQ2_LPG);
	mq2data.co = MQGetGasPercentage(rs_ro, MQ2_CO);
	mq2data.methane = MQGetGasPercentage(rs_ro, MQ2_METHANE);
	mq2data.h2 = MQGetGasPercentage(rs_ro, MQ2_H2);
	mq2data.alcohol = MQGetGasPercentage(rs_ro, MQ2_ALCOHOL);
	mq2data.propane = MQGetGasPercentage(rs_ro, MQ2_PROPANE);

	return mq2data;
}

MQ131TypeDef MQ131Read(byte pin) {
	float rs_ro = MQRead(pin) / Ro.MQ131;

	// Assignment
	// Declare structure variable for reading MQ131
	MQ131TypeDef mq131data = (const struct MQ131TypeDef) {
		0, 0, 0
	};

	// Assignment
	mq131data.NOx = MQGetGasPercentage(rs_ro, MQ131_NOx);
	mq131data.CL2 = MQGetGasPercentage(rs_ro, MQ131_CL2);
	mq131data.O3 = MQGetGasPercentage(rs_ro, MQ131_O3);

	return mq131data;
}

MQ135TypeDef MQ135Read(byte pin) {
	float rs_ro = MQRead(pin) / Ro.MQ135;

	// Assignment
	// Declare structure variable for reading MQ131
	MQ135TypeDef mq135data = (const struct MQ135TypeDef) {
		0, 0, 0
	};

	// Assignment
	mq135data.CO2 = MQGetGasPercentage(rs_ro, MQ135_CO2);
	mq135data.CO = MQGetGasPercentage(rs_ro, MQ135_CO);
	mq135data.NH4 = MQGetGasPercentage(rs_ro, MQ135_NH4);

	return mq135data;
}

// Measure oxygen level
float oxygenRead() {
	return MQ131Read(MQ131_PIN).O3;
}

// Measure C02
float carbondiaoxideRead() {
	return MQ135Read(MQ135_PIN).CO2;
}

// Get pulse rate
const int OUTPUT_TYPE = SERIAL_PLOTTER;
/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 220 ohm resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin connected to an LED (and resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM. Do not connect pins that has timer 2 on it
*/
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

//All the PulseSensor Playground functions.
PulseSensorPlayground pulseSensor;

// Display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
	// read without samples.
	scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	Ro.MQ2 = MQCalibration(MQ2_PIN, MQ2_SENSOR);

	// Configure the PulseSensor manager.
	pulseSensor.analogInput(PULSE_INPUT);
	pulseSensor.blinkOnPulse(PULSE_BLINK);
	pulseSensor.fadeOnPulse(PULSE_FADE);

	pulseSensor.setSerial(Serial);
	pulseSensor.setOutputType(OUTPUT_TYPE);
	pulseSensor.setThreshold(THRESHOLD);

	// Now that everything is ready, start reading the PulseSensor signal.
	if (!pulseSensor.begin()) {
		/*
		   PulseSensor initialization failed.
		*/
		for(int ii = 0; ii < 20; ii++) {
			// Flash the led to show things didn't work.
			digitalWrite(PULSE_BLINK, LOW);
			delay(100);
			digitalWrite(PULSE_BLINK, HIGH);
			delay(100);
		}

		digitalWrite(PULSE_BLINK, HIGH);
	}

	//Initialize the screen and print the first text
	lcd.init();
	lcd.backlight();
	lcd.clear();

	lcd.setCursor(0, 0);
	lcd.print("UG BIOMEDICAL EN");
	delay(3000);
	lcd.clear();
	lcd.setCursor(0, 0);
}

uint16_t motor_t = 0;
uint16_t temperatureelement_t = 0;
uint16_t temphumidity_t  = 0;
uint16_t light_t = 0;
uint16_t pulse_t = 0;
uint16_t weight_t = 0;
uint16_t oxygen_t = 0;
uint16_t carbondiaoxide_t = 0;

const byte T3_Pin = 0;

void loop() {
	readTemptratureHumidity();
	float T2 = temperature;
	float T3 = readTemperature(T3_Pin);

	if(T3 > 37 || T3 < 36) {
		// Alarm ON
	}

	if(T2 < 37) {
		// Heater ON

		// Reduce Fan Speed
	}

	if(T2 > 39) {
		// Heater OFF

		// Increase Fan Speed
	}
}
