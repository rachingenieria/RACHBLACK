#include <arduino.h>

#include "linea.h"

extern slinea Slinea;

slinea::slinea(void)
{
  
}

void slinea::Asignacion_Pines(unsigned char* sensors_pins, int sensor_num)
{
   for(int x=0; x<NUM_SENSORS && x<sensor_num; x++)
   {
      pins[x]=sensors_pins[x];
   }
}

void slinea::Reset_Calibracion(void)
{
   for(int x=0; x<NUM_SENSORS; x++)
   {
     sensorValues_max[x] = 0; 
     sensorValues_min[x] = MAX_VALUE;

     negro[x] = 0;
     blanco[x]= 0;
     num_negro[x]= 0;
     num_blanco[x]= 0;
     num_muestras = 0;
   }
}


int slinea::Calibrar_Color_Linea(void)
{  
   int color_fondo = 4;

   Calibrar_Sensores ();

    for(int x=0; x<NUM_SENSORS; x++)
    {
      if(minimo_general > sensorValues_min[x])
          minimo_general = sensorValues_min[x];

      if(maximo_general < sensorValues_max[x])
          maximo_general = sensorValues_max[x];
    }
   
   int rango_comparacion = (maximo_general + minimo_general)/2; //RANGO TEMPORAL
   
  int timeuot_fondo = 10;

 while(color_fondo == 4 && timeuot_fondo--)
 {
     //Leer_sensores ();
     delay(500);
     //Leer_sensores ();
     
    //fondo NEGRO - LINEA BLANCA
    if(sensorValuesp[0] > rango_comparacion && sensorValuesp[NUM_SENSORS-1] > rango_comparacion  )
    {
      color_fondo = 1;
    }
     //fondo BLANCO - LINEA NEGRA 
    if(sensorValuesp[0] < rango_comparacion && sensorValuesp[NUM_SENSORS-1] < rango_comparacion  )
    {
      color_fondo = 0;
    }
 }
     
return color_fondo;
}


void slinea::Calibrar_Sensores(void)
{   

    //Leer_sensores ();
 
    for(int x=0; x<NUM_SENSORS; x++)
     {
         if(num_muestras == NUM_MUESTRAS/2)
         {
             discriminate[x] = (sensorValues_max[x] + sensorValues_min[x])/2; //DISCRIMINANTE TEMPORAL
         }

         if(num_muestras > NUM_MUESTRAS/2)
         {
           if(sensorValuesp[x] < discriminate[x])
             {
                    blanco[x] += sensorValuesp[x];
                    num_blanco[x] ++;
             }
             if(sensorValuesp[x]> discriminate[x])
             {
                    negro[x] += sensorValuesp[x];
                    num_negro[x] ++;
             }
          }

          if(num_muestras == NUM_MUESTRAS -1 && num_blanco[x] && num_negro[x])
          {
              blanco[x] =  blanco[x]/num_blanco[x]; //MEDIA BLANCOS
              negro[x]  =  negro[x]/num_negro[x];   //MEDIA NEGROS

              discriminate[x] = (blanco[x] + negro[x])/2; //DISCRIMINANTE FINAL
          }
       

       sensores_valor[x][num_muestras] = sensorValuesp[x];  //valor de la muestra


       if(sensorValuesp[x] > sensorValues_max[x])
       {
          sensorValues_max[x] = sensorValuesp[x];
       }
       
       if(sensorValuesp[x] < sensorValues_min[x])
       {
          sensorValues_min[x] = sensorValuesp[x];
       }
       
     }

    num_muestras ++;
}


 
int slinea::Leer_linea(int linea_anterior, int colorlinea )
{   
   int linea, suma, activos;
   int rango_comparacion;
   
    //Leer_sensores ();
    
    suma = 0;
    linea = 0;
    activos = 0;
    sensores_b = 0;
    
    
    
    for(int x=0; x<NUM_SENSORS; x++)
     {
        rango_comparacion = discriminate[x];
        //rango_comparacion = (sensorValues_max[x] + sensorValues_min[x])/2; //DISCRIMINANTE TEMPORAL
        
       if(colorlinea)
       {
             if(sensorValuesp[x] < rango_comparacion) // NEGRA LINEA NEGRA - FONDO BLANCO
             {
                suma += pesos[x];
                sensores_b += 1 << x;
                activos ++;
                S[x] = 1;
             }
             else
             {
               S[x] = 0;
             }
       }
       else
       {
             if(sensorValuesp[x] > rango_comparacion) // ES MENOR ES POR QUE ES BLANCO LINEA-- FONDO NEGRO
             {
                suma += pesos[x];
                sensores_b += 1 << x;
                activos ++;
                S[x] = 1;
             }       
             else
             {
                 S[x] = 0;
             }
       }
     }
     
     if(activos > 0)
     {
         linea = suma/activos;
     }
     else
     {
         
          if(linea_anterior >= 10)
          {
            linea = 45;
          }
          else if(linea_anterior <= -10)
          {
            linea = -45;
          }
          else
          {
            linea = 0;
            }
     
     }
     return linea;
     
}


void slinea::Leer_sensores (void)
{
  int sensor_time = 0;
  int val;
  int sensorValuespaux[NUM_SENSORS];
  
      for(int x=0; x<NUM_SENSORS; x++)
       {
              sensorValuespaux[x] = TIMEOUT;
       }
   
     for(int x=0; x<NUM_SENSORS; x++)
     { 
       pinMode(pins[x], INPUT);
     }

     do{
            for(int x=0; x<NUM_SENSORS; x++)
             {
                  val = digitalRead(pins[x]);
                  if( (val==LOW) && (sensorValuespaux[x] == TIMEOUT) )
                  {
                    
                    sensorValuespaux[x]=sensor_time;
                  }
             }
             sensor_time++;
     }while(sensor_time < TIMEOUT); 

     for(int x=0; x<NUM_SENSORS; x++)
             {
                  sensorValuesp[x]=sensorValuespaux[x];
             }

      for(int x=0; x<NUM_SENSORS; x++)
       {
              pinMode(pins[x], OUTPUT);
              digitalWrite(pins[x], HIGH);
       }
}

void slinea::calculate_Discriminat(void) 
{

  for (int x = 0; x < NUM_SENSORS; x++) 
        {
            num_negros[x] = 0;
            num_blancos[x] = 0;

            suma_negros[x] = 0.0;
            suma_blancos[x] = 0.0;

            media_blancos[x] = 0.0;
            media_negros[x] = 0.0;

            var_blancos[x] = 0.0;
            var_negros[x] = 0.0;
        }

  for (int n = 0; n < NUM_MUESTRAS; n++) 
    {
      for (int x = 0; x < NUM_SENSORS; x++) 
        {
          if(sensores_valor[x][n] < discriminate[x])
          {
                sensores_tipo[x][n] = 0; // blanco refleja
                num_blancos[x] ++;
                suma_blancos[x] += sensores_valor[x][n];
          }
          else
          {
                sensores_tipo[x][n] = 1; // negro no descarga
                num_negros[x] ++;
                suma_negros[x] += sensores_valor[x][n]; 
          }
        }
    }

    //calculo de media
    for (int x = 0; x < NUM_SENSORS; x++) 
        {
          media_negros[x] = suma_negros[x]/ num_negros[x];
          media_blancos[x] = suma_blancos[x]/ num_blancos[x];
        }

    //calculo de Varianza
    for (int n = 0; n < NUM_MUESTRAS; n++) 
    {
      for (int x = 0; x < NUM_SENSORS; x++) 
        {
             if(sensores_tipo[x][n] == 0)
             {
              var_blancos[x] += pow(sensores_valor[x][n] - media_blancos[x], 2);
             }

            if(sensores_tipo[x][n] == 1)
             {
              var_negros[x] += pow(sensores_valor[x][n] - media_negros[x], 2);
             }
        }
    }

    for (int x = 0; x < NUM_SENSORS; x++) 
    {
      var_blancos[x] = var_blancos[x]/ num_blancos[x];
      var_negros[x] = var_negros[x]/ num_negros[x];
    }

    //definir el umbral
    for (int x = 0; x < NUM_SENSORS; x++) 
    {
      discriminate[x] = (media_blancos[x] + var_blancos[x]) + (media_negros[x] - var_negros[x])/2;
    }
}

// Función para realizar el algoritmo K-Means
void slinea::kMeansClustering(float data[], int dataSize, int numClusters) 
{
    float centroids[numClusters];
    // Inicializar los centroides de manera aleatoria
    for (int i = 0; i < numClusters; i++) {
        centroids[i] = random(0, 1001);  // Valores entre 0 y 1000
    }

    // Iteraciones del algoritmo K-Means (puedes ajustar el número de iteraciones)
    for (int iter = 0; iter < 100; iter++) {
        // Asignar cada muestra al centroide más cercano
        int clusterCounts[numClusters] = {0};
        float clusterSums[numClusters] = {0.0};

        for (int i = 0; i < dataSize; i++) {
            float minDistance = 1000.0;
            int closestCluster = -1;

            for (int cluster = 0; cluster < numClusters; cluster++) {
                float distance = abs(data[i] - centroids[cluster]);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestCluster = cluster;
                }
            }

            clusterCounts[closestCluster]++;
            clusterSums[closestCluster] += data[i];
        }

        // Actualizar los centroides
        for (int cluster = 0; cluster < numClusters; cluster++) {
            if (clusterCounts[cluster] > 0) {
                centroids[cluster] = clusterSums[cluster] / clusterCounts[cluster];
            }
        }
    }

    // Imprimir los centroides finales encontrados por K-Means
    for (int cluster = 0; cluster < numClusters; cluster++) {
        Serial.print("Cluster ");
        Serial.print(cluster);
        Serial.print(" - Centroid: ");
        Serial.println(centroids[cluster]);
    }
}
