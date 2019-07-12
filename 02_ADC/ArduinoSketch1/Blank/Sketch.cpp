/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */

/************************************************************************/
/* Read LM35 Temperature                                                */
/************************************************************************/
double LM35_Read(uint8_t ADC_PIN) {
	return analogRead(ADC_PIN) * 0.488;
}

void setup() {
	// put your setup code here, to run once:
	DDRA |= (1 << PORTA0);		// Decleared as an output
	PORTA &= ~(1 << PORTA0);	// Set GPIO to LOW
}

void loop() {
	// put your main code here, to run repeatedly:
	if(LM35_Read(A3) > 35.) {
		PORTA |= (1 << PORTA0);
	} else {
		PORTA &= ~(1 << PORTA0);
	}
}
