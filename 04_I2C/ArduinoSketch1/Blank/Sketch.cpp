/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */
#include <Wire.h>

#define eeddr(a) (0xA0 | ((a) << 1))
#define eepromaddr eeddr(0)

//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

TwoWire exteeprom;

void setup() {
	// put your setup code here, to run once:
	exteeprom.setClock(400000);
}

void loop() {
	// put your main code here, to run repeatedly:

}
