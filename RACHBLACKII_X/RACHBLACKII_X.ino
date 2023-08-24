#include <Adafruit_NeoPixel.h>

#include "Wire.h"
#include <Adafruit_MPU6050.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "flash.h"
#include "motores.h"
#include "linea.h"
#include "api.h"
#include "rachblack.h"
#include "turbina.h"
#include "ACC.h"

#define FIRMWARE_VERSION     23 

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//--------------------------------------------------------------------------------------//
rachblack vel;
Motores motor;
slinea Slinea;
BluetoothSerial SerialBT;
//------------------------------------------------------------------------------------//
//Asignacion de Pines de conexion

//Sensores
//unsigned char sensorline_pins[NUM_SENSORS] = {23,21,22,19,32,33,26,15}; // SENSORES DEL 0 AL 8 QTR8
unsigned char sensorline_pins[NUM_SENSORS] = {26,25,33,32,19,21,22,23}; // SENSORES DEL 0 AL 8 QTR8

#define SDA_PIN 13
#define SCL_PIN 15

//MOTOR - Cualquier GPIO funciona
#define MOTORI_AINA    12
#define MOTORI_AINB    27
#define MOTORI_PWM     14

#define MOTORD_AINA    18   
#define MOTORD_AINB    5
#define MOTORD_PWM     16   


// LISTADO DE PINES Y CONECCIONES
#define LED1           17
#define NUMPIXELS       2 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, LED1, NEO_GRB + NEO_KHZ800);

#define SW1            39
#define ON_RF          2

#define TURBINA_PIN          4

void task_create(void);
TaskHandle_t Task1;
TaskHandle_t Task2;

//------------------------------------------------------------------------------------//
//PARAMETROS del Control del Velocista
//AQUI SE MODIFICAN LOS PARAMETROS DE COMPETENCIA
//VALORES DE CONTROL POR DEFECTO
int   VELCIDAD_MAXIMA        = 36;       //Velocidad Maxima (entre 0 y 100)
int   CTE_PROPORCIONAL       = 8;      //Constante de Control Proporcional (ente 1 y 20)
int   CTE_DERIVATIVA         = 32;      //Constante de Control Diferencia (ente 1 y 20)
int   V_TURBINA              = 35;//35;      //Constante Turbina (ente 0 y 100)                                                                                                                                                                                                                  
int   PISTACOLOR             = 0;

//------------------------------------------------------------------------------------//
//Variables para Control Adicional
#define BUFFER_ERROR 300 //Tamaño de recta a detectar

int val;
int error[BUFFER_ERROR];
float power_difference, power_difference_ext;
float power_difference_ant;


int detect_recta_ant, detect_recta;
char stat_sw = 0; 

int recta_tamano = 0;
int recta_tamano_ultimo = 0;

int flag_turbina = 0;
//------------------------------------------------------------------------------------//

// Define two tasks for Blink & AnalogRead.
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
TaskHandle_t analog_read_task_handle; // You can (don't have to) use this to be able to manipulate a task from somewhere else.

void Led_Control(int ledIR,int ledIG,int ledIB,int ledDR,int ledDG,int ledDB)
{
  pixels.setPixelColor(1, pixels.Color(ledIR, ledIG, ledIB));
  pixels.setPixelColor(0, pixels.Color(ledDR, ledDG, ledDB));
  pixels.show();
}

void setup() 
{
  Serial.begin(115200);

  Acc_Init(SDA_PIN, SCL_PIN);


  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  
  SerialBT.begin("RACHBALCKII");
  
  pinMode(SW1,INPUT_PULLUP);
  
  pinMode(ON_RF,INPUT); 
  digitalWrite(ON_RF, LOW); //PULL DOWN
   
  Slinea.Asignacion_Pines(sensorline_pins,8);
  task_create_sensor(); //Leer Linea

  motor.Motor_Init(MOTORI_AINA,MOTORI_AINB,MOTORI_PWM,MOTORD_AINA,MOTORD_AINB,MOTORD_PWM);
  motor.SetSpeeds(0,0);
  
  Turbina_Init(TURBINA_PIN);

  Eeprom_read();
  Serial.printf("vel.ver %d, vel.vavg %d, vel.kpg %d, vel.kdg %d, vel.colorlinea %d, vel.pmw_t %d\n",vel.ver, vel.vavg, vel.kpg, vel.kdg, vel.colorlinea, vel.pmw_t);

  if(vel.ver != FIRMWARE_VERSION)// DATOS CORRECTOS y CARGADOS
  {
    vel.setupconfig(VELCIDAD_MAXIMA,CTE_PROPORCIONAL,CTE_DERIVATIVA,V_TURBINA,FIRMWARE_VERSION); //valres por DEFECTO
    Eeprom_save();
    Serial.printf("Saved \n");
    Serial.printf("vel.ver %d, vel.vavg %d, vel.kpg %d, vel.kdg %d, vel.colorlinea %d, vel.pmw_t %d\n",vel.ver, vel.vavg, vel.kpg, vel.kdg, vel.colorlinea, vel.pmw_t);
  }
  
  

  int cursorz = 0;
  int menuactivo = 0;
  
  // MENU DE CONFIGURACION
  //Serial_send_variables();

  Led_Control(150,0,0,150,0,0);
  delay(500);
  Led_Control(0,0,150,0,0,150);
  

  motor.SetSpeeds(0, 0); 
  val = 1;
  int mx = 0;
  int my = 0;
  vel.mx = 0;
  vel.my = 0;
  
  do{
      //RECIBIR DATOS POR BLUETOOTH
      Serial_command();
      val = digitalRead(SW1);  
      
         mx =  -(vel.vavg * vel.mx)/10;
         my =   (vel.vavg * vel.my)/10;
      
      motor.SetSpeeds(mx + my, mx - my);

       if(flag_turbina)
       {
          Turbina_set(vel.pmw_t);
          delay(2000);
          Turbina_set(0);
          delay(2000);
          flag_turbina = 0;
       }
      
  }while (val);
  

  motor.SetSpeeds(0, 0); 

  Led_Control(0,0,0,150,0,0);
  delay(200);
  Led_Control(150,0,0,0,0,0);
  delay(200);
  Led_Control(0,0,0,150,0,0);
  delay(200);                  
  Led_Control(150,0,0,0,0,0);   
  delay(200);
  Led_Control(0,0,0,0,0,0);
  

//-------------Instrucciones para Empezar a hacer la Calibracion de Sensores--------------------------------------//
  
  Slinea.Reset_Calibracion(); //ROBOT EN MEDIO DE LA LINEAS
  
  Led_Control(0,0,150,0,0,150);
  
  //GIRA MIENTRA CALIBRA
  motor.SetSpeeds(-25, 25);
  int tiempo_cal = NUM_MUESTRAS + 1;
  while(tiempo_cal--)
  {
      Slinea.Calibrar_Sensores();
      delay(1);
  }
  
  Slinea.calculate_Discriminat();
  Serial_Report_Calibration();

  vel.colorlinea = Slinea.Calibrar_Color_Linea();


  Led_Control(150,0,0,150,0,0);
  motor.SetSpeeds(0, 0);

  val = digitalRead(SW1);  
  vel.position_line = 60;
  
  while (val == HIGH )
          {
             val = digitalRead(SW1);  
             vel.position_line = Slinea.Leer_linea(vel.position_line ,vel.colorlinea ); // leemos posicion de la linea en la variable position
           
             
             for(int x=0; x<NUM_SENSORS; x++)
              {
                 Serial.print(Slinea.sensorValuesp[x]);
                 Serial.print(',');
              }
              Serial.println(vel.position_line);

               if (vel.position_line < -20)
               {
                   Led_Control(0,0,0,150,0,0);
                }
                else  if (vel.position_line > 20 )
                {
                  Led_Control(150,0,0,0,0,0);    
                }
          
                error[5]=error[4];
                error[4]=error[3];
                error[3]=error[2];
                error[2]=error[1];
                error[1]=error[0];
                error[0]=vel.position_line;
          
                float kpg = (float) vel.kpg/10.0;
                float kdg = (float) vel.kdg/10.0;

                power_difference = (error[0] * kpg) + ((error[0] - error[4]) * kdg);
                motor.SetSpeeds( - power_difference,  power_difference);
              
                delay(1);
}
  
   //---------------------------FIN DE PRUEBA DE CALIBRACION----------------------------------------------------//
   //stop Motors
  motor.SetSpeeds(0,0);
  
  Led_Control(0,150,0,0,150,0);   
  delay(200);
  Led_Control(0,0,0,0,0,0);
  delay(200);
  Led_Control(0,150,0,0,150,0);   
  delay(200);                  
  Led_Control(0,0,0,0,0,0);   
  delay(200);
  Led_Control(0,150,0,0,150,0);


 //---------------------------Verificacion de Sensores----------------------------------------------------//
  val = digitalRead(SW1);  
  while (val == HIGH )
  {    
     //Serial_send_variables();
     vel.position_line = Slinea.Leer_linea(vel.position_line ,vel.colorlinea); 
     delay(100);
     val = digitalRead(SW1); 
  }

 //---------------------------LISTO PARA COMPETIR----------------------------------------------------// 
 //---------------------------SELECIONAR METODO DE INICO----------------------------------------------------//
  
  Led_Control(0,0,0,150,0,0);
  delay(200);
  Led_Control(150,0,0,0,0,0);
  delay(200);
  Led_Control(0,0,0,150,0,0);
  delay(200);                  
  Led_Control(150,0,0,0,0,0);   
  delay(200);
  Led_Control(0,0,0,0,0,0); 
 

// INICIO CON MODULO REMOTO
  
   vel.start = 0; //STOP BLUETOOTH
   int rf_control = digitalRead(ON_RF);

  Led_Control(150,0,0,150,0,0); 
  val = 0;  

   Turbina_set(vel.pmw_t);
  
   while(rf_control == 0 && vel.start == 0 && stat_sw == 0)
   {
      rf_control = digitalRead(ON_RF);
      Serial_command();
      delay(2);
      val = digitalRead(SW1); 
      if(val == LOW)
      {
          Led_Control(0,0,0,150,0,0);
          delay(1000);
          Led_Control(150,0,0,0,0,0);
          delay(1000);
          Led_Control(150,0,0,150,0,0);
          delay(1000);
          Led_Control(0,0,0,0,0,0);
          delay(1000);
          Led_Control(150,0,0,150,0,0);
          delay(999);
          stat_sw =  1;
      }
   }


  if(vel.start == 0xFF) // MODULO CONECTADO PERO ARRANCA POR SW
  {
      stat_sw =  1; //NO PARA POR MODULO
  }
  
  if(rf_control) // MODULO NO CONECTADO, ARRANCA POR SW
  {
    stat_sw =  0; //NO PARA POR MODULO
  }

   vel.start = 0xFF; //START BLUETOOTH

   Led_Control(0,0,0,0,0,0);

   detect_recta_ant = 1;
   detect_recta = 1;

  task_create_loop();
  
}
 
void loop()
{
   delay(10000);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task2loop(void *pvParameters)
{ 
  while(1)
  {
    //APAGADO POR MODULO REMOTO
   int rf_control = digitalRead(ON_RF);
   if (rf_control == 0 && stat_sw == 0)
   {
       motor.SetSpeeds(0, 0);
       Turbina_set( 0);
       while(1)
       {
          Led_Control(0,0,0,150,0,0);
          delay(500);
          Led_Control(150,0,0,0,0,0);
          delay(500);
        
        }
   } // STOP ROBOT

   if (vel.start == 0 && stat_sw == 1)
   {
       motor.SetSpeeds(0, 0);
   }

 if( vel.start )
  {
    vel.position_line = Slinea.Leer_linea(vel.position_line ,vel.colorlinea); // leemos posicion de la linea en la variable position

       /*
       //led para curva
       if (vel.position_line < -20)
       {
           digitalWrite(LED2, LOW);
           digitalWrite(LED1, HIGH); 
        }
        else  if (vel.position_line > 20 )
        {
           digitalWrite(LED2, HIGH);
           digitalWrite(LED1, LOW);    
        }
      */

   
   //Valores anteriores
    for(int ind = BUFFER_ERROR-1; ind > 0 ; ind --)
    {
      error[ind] = error[ind-1];
    }
     error[0] = vel.position_line;
    

    int vavg= vel.vavg;
    float kpg = (float) vel.kpg/10.0;
    float kdg = (float) vel.kdg/10.0;

    power_difference = (error[0] * kpg) + ((error[0] - error[5]) * kdg);

    //Calculo de Recta  
    int suma_recta = 0;
    int sensor_curva = 0;
    
    detect_recta_ant = detect_recta;
    detect_recta = 0;
   
     for(int i=0 ; i<BUFFER_ERROR; i++)
      {
        suma_recta = error[i];
        if( error[i] > 20 || error[i] < -20)
        {
          sensor_curva = 1;
        }
      }

    suma_recta = suma_recta/BUFFER_ERROR;

    if(sensor_curva == 0)
    {
      if(abs(suma_recta) < 8)
      {
        detect_recta = 1;
      }
    }
    

      /*
      if(detect_recta == 0 && detect_recta_ant == 1) //CAmbio de Recta a curva
      {
           Led_Control(150,0,0,150,0,0); 
           motor.SetSpeeds(-50,-50); //Freno
           delay(5); //tiempo proporcional a la longitud de la recta
           //motor.SetSpeeds(vavg,vavg); //Freno
           //delay(100); //tiempo proporcional a la longitud de la recta
           Led_Control(0,0,0,0,0,0); 
      }
      */
 
     //motor.SetSpeeds(vavg  - power_difference, vavg +  power_difference);
     if(power_difference > 0)
     {
       motor.SetSpeeds(vavg  - power_difference,  vavg );
     }
     else if(power_difference < 0)
     {
        motor.SetSpeeds(vavg, vavg +  power_difference);

     }
     else
     {
       motor.SetSpeeds(vavg,vavg);
     }
  }
  else
  {
     motor.SetSpeeds(0, 0);
  }
  
  //SERIAL STOP
   Serial_command();
   Serial_send_variables();

   //delay(1);
  }

}

void Task1code(void *pvParameters)
{ 
  while(1)
  {
    Slinea.Leer_sensores();
    //Acc_read();
    delay(1);
  }

}

void task_create_sensor(void)
{
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                      Task1code,   /* Task function. */
                      "Task1",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      3,           /* priority of the task */
                      &Task1,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */                  
}


void task_create_loop(void)
{
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                      Task2loop,   /* Task function. */
                      "Task2",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      3,           /* priority of the task */
                      &Task1,      /* Task handle to keep track of created task */
                      1);          /* pin task to core 0 */                  
}