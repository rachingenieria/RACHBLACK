#include <arduino.h>
#include "BluetoothSerial.h"

#include "rachblack.h"
#include "linea.h"
#include "flash.h"
#include "api.h"

extern rachblack vel;
extern slinea Slinea;
extern BluetoothSerial SerialBT;

extern int flag_turbina;

int API_CONFIG(char* uart_buffer_rx)
{
int respuesta = 0;
          if( (uart_buffer_rx[0] == 'V' || uart_buffer_rx[0] == 'D' || uart_buffer_rx[0] == 'P' || uart_buffer_rx[0] == 'I'|| uart_buffer_rx[0] == 'T' || uart_buffer_rx[0] == 'S' || uart_buffer_rx[0] == 'X') 
              && (uart_buffer_rx[1] >= '0'  && uart_buffer_rx[1] <= '9')
              && (uart_buffer_rx[2] >= '0'  && uart_buffer_rx[2] <= '9'))
          {
                  if(uart_buffer_rx[0] == 'V')
                  {
                     vel.vavg = (uart_buffer_rx[1] - '0' )*10 + (uart_buffer_rx[2]- '0' );
                     respuesta = 1;
                  }
                  else if(uart_buffer_rx[0] == 'P')
                  {
                     vel.kpg = ((uart_buffer_rx[1] - '0' )*10 + (uart_buffer_rx[2]- '0' ));
                     respuesta = 1;
                  }
                  else if(uart_buffer_rx[0] == 'D')
                  {
                     vel.kdg = ((uart_buffer_rx[1] - '0' )*10 + (uart_buffer_rx[2]- '0' ));
                     respuesta = 1;
                  }
                  else if(uart_buffer_rx[0] == 'T')
                  {
                     vel.pmw_t = ( uart_buffer_rx[1] - '0' )*10 + (uart_buffer_rx[2] - '0' );
                     flag_turbina = 1;
                     respuesta = 1;
                  }
                  else if(uart_buffer_rx[0] == 'X')
                  {
                     vel.mx = 20 *  ((uart_buffer_rx[2] - '0' )-  5);
                     vel.my = 20 *  ((uart_buffer_rx[1] - '0' ) - 5);
                  }

                  else if(uart_buffer_rx[0] == 'S')
                  {
                     if((uart_buffer_rx[1] == '1'  && uart_buffer_rx[2] == '5'))
                     {
                         Serial.printf("Saved \n");
                          Serial.printf("vel.ver %d, vel.vavg %d, vel.kpg %d, vel.kdg %d, vel.colorlinea %d, vel.pmw_t %d\n",vel.ver, vel.vavg, vel.kpg, vel.kdg, vel.colorlinea, vel.pmw_t);
                         Eeprom_save();
                         respuesta = 1;
                     }

                     if((uart_buffer_rx[1] == '2'  && uart_buffer_rx[2] == '0'))
                     {
                         vel.start = 0XFF;
                         respuesta = 1;
                     }

                     if((uart_buffer_rx[1] == '2'  && uart_buffer_rx[2] == '5'))
                     {
                         vel.start = 0;
                         respuesta = 1;
                     }

                     if((uart_buffer_rx[1] == '3'  && uart_buffer_rx[2] == '0'))
                     {
                         //vel.remoto_enable = 1;
                     }

                     if((uart_buffer_rx[1] == '3'  && uart_buffer_rx[2] == '5'))
                     {
                         //vel.remoto_enable = 0;
                     }

                     if((uart_buffer_rx[1] == '4'  && uart_buffer_rx[2] == '0'))
                     {
                         //vel.sw_enable = 1;
                     }

                     if((uart_buffer_rx[1] == '4'  && uart_buffer_rx[2] == '5'))
                     {
                         //vel.sw_enable = 0;
                     }

                     //PISO
                      if((uart_buffer_rx[1] == '5'  && uart_buffer_rx[2] == '0'))
                     {
                         vel.colorlinea = 0;
                     }

                     if((uart_buffer_rx[1] == '5'  && uart_buffer_rx[2] == '5'))
                     {
                         vel.colorlinea = 1;
                     }
                  }
                  
          }
          else
          {
            uart_buffer_rx[0] = 0;
            uart_buffer_rx[1] = 0;
            uart_buffer_rx[2] = 0;
          }

return respuesta;
}


int API_BUFFER(char uart_buffer_rx)
{
int respuesta = 0;
          if( uart_buffer_rx== 'V' || uart_buffer_rx == 'D' || uart_buffer_rx == 'P' || uart_buffer_rx == 'I'|| uart_buffer_rx == 'T' || uart_buffer_rx == 'S' || uart_buffer_rx == 'X') 
          {
            respuesta = 1;
          }
  return respuesta;
}


int count;
char uart_buffer_tx[32];
char uart_buffer_rx[32];

void Serial_command(void)
{
while (SerialBT.available()) {
      char datau = SerialBT.read();
      Serial.print(datau);
      if( API_BUFFER(datau))
       {
         count = 0;
       }
      uart_buffer_rx[count] = datau;
      count ++;
      if( count == 3)
      {
        if(API_CONFIG(uart_buffer_rx))
        {
           Serial_send_variables();
        }
      }
      if(count > 3)
        count = 0;
    }
}

void Serial_send_variables(void)
{
  SerialBT.print("#");
  SerialBT.print(",");
  SerialBT.print(vel.vavg);
  SerialBT.print(",");
  SerialBT.print(vel.kpg);
  SerialBT.print(",");
  SerialBT.print(vel.kdg);
  SerialBT.print(",");
  SerialBT.print(vel.pmw_t);
  SerialBT.print(",");
  SerialBT.print(vel.colorlinea);
  SerialBT.print(",");
  SerialBT.print(vel.position_line);
  for(int s=0; s<8; s++)
  {
     SerialBT.print(",");
     SerialBT.print(Slinea.S[s]);
  }
  SerialBT.println(",");

/*
  Serial.print("#");
  Serial.print(",");
  Serial.print(vel.vavg);
  Serial.print(",");
  Serial.print(vel.kpg);
  Serial.print(",");
  Serial.print(vel.kdg);
  Serial.print(",");
  Serial.print(vel.pmw_t);
  Serial.print(",");
  Serial.print(vel.colorlinea);
  Serial.print(",");
  Serial.print(vel.position_line);
  for(int s=0; s<8; s++)
  {
     Serial.print(",");
     Serial.print(Slinea.S[s]);
  }
  Serial.println(",");
*/
  //delay(2);
  
}

void Serial_Report_Calibration(void)
{

 SerialBT.print("#");
 SerialBT.print(",");
 SerialBT.print(Slinea.num_muestras);

 for(int x=0; x<NUM_SENSORS; x++)
     {
        SerialBT.print("discriminate:");
        SerialBT.print(Slinea.discriminate[x]); 
        SerialBT.print(", negro:"); 
        SerialBT.print(Slinea.negro[x]);
        SerialBT.print(", blanco:");
        SerialBT.print(Slinea.blanco[x]);
        SerialBT.print(", num_negro:");
        SerialBT.print(Slinea.num_negro[x]);
        SerialBT.print(", num_blanco:");
        SerialBT.print(Slinea.num_blanco[x]);
        SerialBT.print(", sensorValues_max:");
        SerialBT.print(Slinea.sensorValues_max[x]);
        SerialBT.print(", sensorValues_min:");
        SerialBT.println(Slinea.sensorValues_min[x]);
     }

 for(int x=0; x<NUM_SENSORS; x++)
     {
        SerialBT.print(" suma_clase:");
        SerialBT.print(Slinea.suma_clase[x][0]); 
        SerialBT.print(",");
        SerialBT.print(Slinea.suma_clase[x][1]); 

        SerialBT.print(" num_samples_per_class:");
        SerialBT.print(Slinea.num_samples_per_class[x][0]); 
        SerialBT.print(",");
        SerialBT.print(Slinea.num_samples_per_class[x][1]); 

        SerialBT.print(" media:");
        SerialBT.print(Slinea.media[x][0]); 
        SerialBT.print(",");
        SerialBT.print(Slinea.media[x][1]); 


        SerialBT.print(", scatter_between:"); 
        SerialBT.print(Slinea.scatter_between[x]); 
        SerialBT.print(", scatter_within:");
        SerialBT.print(Slinea.scatter_within[x]);

        SerialBT.print(", centroids:"); 
        SerialBT.print(Slinea.centroides_Sensors[x][0]); 
        SerialBT.print(",");
        SerialBT.print(Slinea.centroides_Sensors[x][1]);

        
        SerialBT.print(", discriminate_LDA:");
        SerialBT.println(Slinea.discriminate_LDA[x]);
        
     }


 SerialBT.println(",");

}
