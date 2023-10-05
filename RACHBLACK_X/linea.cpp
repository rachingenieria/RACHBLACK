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
  num_muestras = 0;
   for(int x=0; x<NUM_SENSORS; x++)
   {
     sensorValues_max[x] = 0; 
     sensorValues_min[x] = MAX_VALUE;

     negro[x] = 0;
     blanco[x]= 0;
     num_negro[x]= 0;
     num_blanco[x]= 0;
   }
}


int slinea::Calibrar_Color_Linea(void)
{  
     int color_fondo = 0;
     
    //fondo NEGRO - LINEA BLANCA
    if(num_negro[4] > num_blanco[4])
    {
      color_fondo = 1;
    }
     //fondo BLANCO - LINEA NEGRA 
    if(num_blanco[4] > num_negro[4])
    {
      color_fondo = 0;
    }
     
return color_fondo;
}


void slinea::Calibrar_Sensores(void)
{   

    //Leer_sensores ();
 
    for(int x=0; x<NUM_SENSORS; x++)
     {

       sensores_valor[x][num_muestras] = sensorValuesp[x];  //valor de la muestra

       if(sensorValuesp[x] > sensorValues_max[x])
       {
          sensorValues_max[x] = sensorValuesp[x];
       }
       
       if(sensorValuesp[x] < sensorValues_min[x])
       {
          sensorValues_min[x] = sensorValuesp[x];
       }

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
              pinMode(pins[x], OUTPUT);
              digitalWrite(pins[x], HIGH);
       }

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
}

void slinea::calculate_Discriminat(void) 
{
  for (int x = 0; x < NUM_SENSORS; x++) 
    {

        suma_clase[x][0] = 0.0; // Suma de valores para cada clase
        suma_clase[x][1] = 0.0; // Suma de valores para cada clase

        num_samples_per_class[x][0] = 0; // Número de muestras por clase
        num_samples_per_class[x][1] = 0; // Número de muestras por clase

        media[x][0] = 0.0; // Media para cada clase
        media[x][1] = 0.0; // Media para cada clase

        scatter_between[x] = 0.0; // Scatter entre clases
        scatter_within[x] = 0.0; // Scatter dentro de clases


      for (int n = 0; n < NUM_MUESTRAS; n++) 
        {

            if(sensores_valor[x][n] < discriminate[x])
            {
                  sensores_tipo[x][n] = 0; // blanco refleja
            }
            else
            {
                  sensores_tipo[x][n] = 1; // negro no descarga
            }

        }

        // Cálculos de medias y sumas por clase
        for (int n = 0; n < NUM_MUESTRAS; n++) 
        {
          suma_clase[x][sensores_tipo[x][n]] += sensores_valor[x][n];
          num_samples_per_class[x][sensores_tipo[x][n]]++;
        }

        for (int c = 0; c < 2; c++) 
        {
          media[x][c] = suma_clase[x][c] / num_samples_per_class[x][c];
        }

        // Cálculo de Scatter Between y Scatter Within
        for (int n = 0; n < NUM_MUESTRAS; n++) {
          double diff = sensores_valor[x][n] - media[x][sensores_tipo[x][n]];
          scatter_within[x] += diff * diff;
        }

         scatter_between[x] = scatter_within[x] - scatter_within[x] / NUM_MUESTRAS;

         centroides_Sensors[x][0] = 50;//media[x][0];
         centroides_Sensors[x][1] = 200;//media[x][1];

         kMeansClustering(&sensores_valor[x][0], NUM_MUESTRAS, 2, &centroides_Sensors[x][0]) ;

        // Cálculo de discriminante (umbral)
        discriminate_LDA[x] = scatter_within[x] / scatter_between[x];
        discriminate[x] = (centroides_Sensors[x][0] + centroides_Sensors[x][1])/2;
    }
}

// Función para realizar el algoritmo K-Means
void slinea::kMeansClustering(int* data, int dataSize, int numClusters, int* centroides) 
{
    float centroids[numClusters];
    // Inicializar los centroides de manera aleatoria
    for (int i = 0; i < numClusters; i++) {
        centroids[i] = centroides[i];  // Valores entre 0 y 1000
    }


    // Iteraciones del algoritmo K-Means (puedes ajustar el número de iteraciones)
    for (int iter = 0; iter < 100; iter++) 
    {
        // Asignar cada muestra al centroide más cercano
        int clusterCounts[numClusters] = {0};
        float clusterSums[numClusters] = {0.0};

        for (int i = 0; i < dataSize; i++) 
        {
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
            if (clusterCounts[cluster] > 0) 
            {
                centroids[cluster] = clusterSums[cluster] / clusterCounts[cluster];
            }
        }
    }

    for (int i = 0; i < numClusters; i++) 
    {
        centroides[i] = centroids[i];  // Valores entre 0 y 1000
    }
    
}
