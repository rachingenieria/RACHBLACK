#include <arduino.h>
#include <Servo.h>

Servo myservo;

int pos = 0;    // variable to store the servo position

void Turbina_Init(int pint) 
{
  myservo.attach(pint);  // attaches the servo on pin 13 to the servo object
  
  myservo.write(180);
  delay(1000);
  myservo.write(0);
  delay(1000);
  myservo.write(90);
  delay(1000);
}


void Turbina_set(int pos)
{
  int turbina = pos * 180/100; 
  
  if(turbina>180)
     turbina = 180;

  if(turbina<0)
     turbina = 0;

  myservo.write(turbina);
}