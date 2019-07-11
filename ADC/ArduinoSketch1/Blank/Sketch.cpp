/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

uint16_t temperature = 0;

void setup() {
	// put your setup code here, to run once:
	temperature = analogRead(A3) * 0.488;
}

void loop() {
	// put your main code here, to run repeatedly:

}
