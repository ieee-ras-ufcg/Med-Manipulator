#include <Stepper.h>
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
void setup()
{
	// velocidade em 60 rpm:
	myStepper.setSpeed(60);
	// inicializa a porta serial:
	Serial.begin(9600);
}
void loop() 
{
	// movimento em uma direção:
	Serial.println("clockwise");
	myStepper.step(stepsPerRevolution);
	delay(500);

	// movimento em outra direção:
	Serial.println("counterclockwise");
	myStepper.step(-stepsPerRevolution);
	delay(500);
}
