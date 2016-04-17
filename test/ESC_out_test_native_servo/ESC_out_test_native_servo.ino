//Testing ESC
// do not connect 5V to Arduino (only GND and Signal)
//only connect ESC to power AFTER Arduino is powered up

#include <Servo.h>

Servo myESC;  // create servo object to control ESC
String text;
int value;

void setup() 
{
  Serial.begin(9600);
  myESC.attach(9);  // attaches the servo on pin 9 to the servo object
  myESC.writeMicroseconds(1000); //write lowest value to ESC, to avoid it going into calibration or programming mode and to arm it
}

void loop() 
{
  while(Serial.available())
  {
    text=Serial.readString();
    value=text.toInt(); // =0 if string is not a number
    Serial.print("received: ");
    Serial.print(value);
    Serial.print("\n");

    //Change PWM output
    myESC.writeMicroseconds(value);
  }

}
