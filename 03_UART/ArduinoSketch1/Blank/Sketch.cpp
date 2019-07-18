/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

void setup() {
	// Initialize serial with baud 9600 bit per second
	Serial.begin(9600);
	// Print hello world on terminal
	Serial.println("Hello World");
	// Print shell sign
	Serial.print(">");

	DDRA |= (1 << PORTA0);
	PORTA &= ~(1 << PORTA0);
}

char in[4];
char k, u = 0;

void loop() {
	// Wait for serial input from device
	while(!Serial.available());
	// Read input data
	u = Serial.read();
	// Compare input data if it is enter key pressed
	if(u != '\r') {
		in[k] = u;
		Serial.print(in[k]);
		k++;
	}

	// Compare buffer elements to check if it is ON
	if(in[0] == 'O' && in[1] == 'N') {
		// Turn motor on
		PORTA |= (1 << PORTA0);
		// Clear buffer
		memset(in, 0x00, sizeof(in));
		k = 0;
		Serial.print(">");
	}
	// Compare buffer elements to check if it is OFF
	else if(in[0] == 'O' && in[1] == 'F' && in[2] == 'F') {
		// Turn motor off
		PORTA &= ~(1 << PORTA0);
		// Clear buffer
		memset(in, 0x00, sizeof(in));
		k = 0;
		Serial.print(">");
	}
}
