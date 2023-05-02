#include <arduino.h>
#include <stdlib.h>     /* abs */
#include "motores.h"

extern Motores motor;

// Setting PWM properties
const int freq = 1000;
const int resolution = 8;


const int pwmChannelD = 2;
const int pwmChannelI = 3;


Motores::Motores(void)
{
  
}

void Motores::Motor_Init(int MI_AIN1,int MI_AIN2, int MI_PWM ,int  MD_AIN1, int MD_AIN2, int MD_PWM )
{

  motor.MOTORD_AIN1 = MD_AIN1;
  motor.MOTORD_AIN2 = MD_AIN2;
  motor.MOTORD_PWM = MD_PWM;

  motor.MOTORI_AIN1 = MI_AIN1;
  motor.MOTORI_AIN2 = MI_AIN2;
  motor.MOTORI_PWM = MI_PWM;

  pinMode(MD_AIN1, OUTPUT);
  pinMode(MD_AIN2, OUTPUT);
  //pinMode(MD_PWM, OUTPUT);
  
  pinMode(MI_AIN1, OUTPUT);
  pinMode(MI_AIN2, OUTPUT);
  //pinMode(MI_PWM, OUTPUT);
  

  // configure LED PWM functionalitites
  ledcSetup(pwmChannelD, freq, resolution);
  ledcSetup(pwmChannelI, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MD_PWM, pwmChannelD);
  ledcAttachPin(MI_PWM, pwmChannelI);

}

//RANGO -100 to 100
void Motores::SetSpeeds(int mi,int md)
{
 int motor_derecho = md*255/100;
 int motor_izquierdo = mi*255/100;
 
 if(motor_derecho > 0)
  {
    
     if(motor_derecho>255)
         {
            motor_derecho = 255;
         }

     digitalWrite(motor.MOTORD_AIN2,LOW);
     digitalWrite(motor.MOTORD_AIN1,HIGH);
     ledcWrite(pwmChannelD, abs(motor_derecho)); 
  }
  else if(motor_derecho < 0)
    {
      
      if(motor_derecho<-255) //LIMITE GIRO ATRAS
         {
            motor_derecho = -255;
         }
         
     digitalWrite(motor.MOTORD_AIN2,HIGH);
     digitalWrite(motor.MOTORD_AIN1,LOW);
     ledcWrite(pwmChannelD, abs(motor_derecho)); 
  }
  else //=0
  {
     digitalWrite(motor.MOTORD_AIN1,HIGH);
     digitalWrite(motor.MOTORD_AIN2,HIGH);
     ledcWrite(pwmChannelD, 0); 
  }
 
 
 
  if(motor_izquierdo > 0)
  { 
    if(motor_izquierdo>255)
    {
      motor_izquierdo = 255;
    }
     digitalWrite(motor.MOTORI_AIN1,LOW);
     digitalWrite(motor.MOTORI_AIN2,HIGH);
     ledcWrite(pwmChannelI, abs(motor_izquierdo)); 
  }
  else if (motor_izquierdo < 0)
    {
       if(motor_izquierdo<-255)
         {
            motor_izquierdo = -255;
         }
     digitalWrite(motor.MOTORI_AIN1,HIGH);
     digitalWrite(motor.MOTORI_AIN2,LOW);
     ledcWrite(pwmChannelI, abs(motor_izquierdo)); 
  }
  else
  {
     digitalWrite(motor.MOTORI_AIN1,HIGH);
     digitalWrite(motor.MOTORI_AIN2,HIGH);
     ledcWrite(pwmChannelI, 0);
  }
  
}
