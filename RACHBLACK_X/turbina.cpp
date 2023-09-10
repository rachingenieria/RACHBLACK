#include <arduino.h>
//#include <Servo.h>

//Servo myservo;

int pos = 0;    // variable to store the servo position

const int minDutyMicros = 1000; // 1060 µs
const int medDutyMicros = 1500; // 1060 µs
const int maxDutyMicros = 2000; // 1860 µs

// 490HZ Mas periodo 2.04 ms  255

void Turbina_Init(int pint) 
{
  pinMode(pint, OUTPUT);

  ledcSetup(0, 50, 8); // Configurar el canal 0 para una frecuencia de 490 Hz y resolución de 8 bits
  ledcAttachPin(pint, 0); // Asociar el pin al canal 0

  int dutyCycle = map(medDutyMicros, 0, 20000, 0, 255); // Mapear el rango a la resolución de 8 bits
  ledcWrite(0, dutyCycle); // Configurar el ciclo de trabajo
  delay(2000);
  dutyCycle = map(minDutyMicros, 0, 20000, 0, 255); // Mapear el rango a la resolución de 8 bits
  ledcWrite(0, dutyCycle); // Configurar el ciclo de trabajo
  delay(2000);
  dutyCycle = map(medDutyMicros, 0, 20000, 0, 255); // Mapear el rango a la resolución de 8 bits
  ledcWrite(0, dutyCycle); // Configurar el ciclo de trabajo
 
}


void Turbina_set(int pos)
{
  int posd = map(pos, 0, 99, 0, 500); // Mapear el rango a la resolución de 16 bits

  int dutyCycle = map(medDutyMicros - posd, 0, 20000, 0, 255); // Mapear el rango a la resolución de 16 bits 
  ledcWrite(0, dutyCycle); // Configurar el ciclo de trabajo

}