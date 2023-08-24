#ifndef LINEA_h

#define TIMEOUT                   1024

#define MAX_VALUE                 1024
#define NUM_MUESTRAS              1000

//Sensores de Linea PD
#define NUM_SENSORS             8  // Numero de sensores que usa

class slinea
{
  public:
  
    slinea (void);
    void Asignacion_Pines(unsigned char* sensors_pins, int sensor_num);
    void Leer_sensores (void);
    void Reset_Calibracion(void);
    int Calibrar_Color_Linea(void);
    void Calibrar_Sensores(void);
    int Leer_linea(int linea_anterior, int colorlinea);
    void calculate_Discriminat(void);
    void kMeansClustering(float data[], int dataSize, int numClusters);
    

    int colorlinea;
    int position_line;
    int S[8];
    int discriminate[NUM_SENSORS];  
    //int sensorValues[NUM_SENSORS];  
    int sensorValuesp[NUM_SENSORS];  

    int minimo_general = 1024;
    int maximo_general = 0;
    int num_muestras = 0;
    int sensores_b;
    int negro[NUM_SENSORS], blanco[NUM_SENSORS];
    int num_negro[NUM_SENSORS], num_blanco[NUM_SENSORS];
    unsigned int sensorValues_max[NUM_SENSORS];
    unsigned int sensorValues_min[NUM_SENSORS];

    // Definir las muestras de las dos clases para 8 sensores y 1000 muestras cada una
    int sensores_valor[NUM_SENSORS][NUM_MUESTRAS];  //valor de la muestra
    int sensores_tipo[NUM_SENSORS][NUM_MUESTRAS];   //valor clasificacion de la muestra

    int numSamples = NUM_MUESTRAS;  // Número de muestras

    int num_negros[NUM_SENSORS];
    int num_blancos[NUM_SENSORS];

    float suma_negros[NUM_SENSORS];
    float suma_blancos[NUM_SENSORS];

    float media_blancos[NUM_SENSORS];
    float media_negros[NUM_SENSORS];

    float var_blancos[NUM_SENSORS];
    float var_negros[NUM_SENSORS];
        
  private:
        //variables de control
        int sensor_num;
        unsigned char pins[NUM_SENSORS];
        int pesos[NUM_SENSORS*2+2] = {-35,-20,-10,-5,5,10,20,35}; // SENSORES DEL 0 AL 8 QTR8
        
};

#endif
