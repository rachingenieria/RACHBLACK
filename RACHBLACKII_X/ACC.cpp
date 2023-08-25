#include "Wire.h"
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
TwoWire I2Cone = TwoWire(0);

sensors_event_t a, g, temp;

  float accelX[4];
  float accelY[4];
  float accelZ[4];
  float gyroX[4];
  float gyroY[4];
  float gyroZ[4];


  const float accelThreshold = 0.1;
  const float gyroThreshold = 0.01;


  // Variables para el cálculo del ángulo
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;


const float alpha = 0.98;  // Factor de filtro complementario

// Variables para el cálculo del desplazamiento
float velocityX = 0.0;
float velocityY = 0.0;
float displacementX = 0.0;
float displacementY = 0.0;
unsigned long prevTime = 0;


float gyroXangle_grados;
float gyroYangle_grados;
float gyroZangle_grados;


  // Calibrar el acelerómetro
 float accelXOffset = 0;
 float accelYOffset = 0;
 float accelZOffset = 0;

 float gyroXOffset = 0;
 float gyroYOffset = 0;
 float gyroZOffset = 0;




void Acc_Init(int SDA_1, int SCL_1)
{

    I2Cone.begin(SDA_1, SCL_1, 100000); 

  if (!mpu.begin(0x68, &I2Cone)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");

  switch (mpu.getAccelerometerRange()) 
  {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) 
      {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
      }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) 
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void resetAndCalculate() 
{
  velocityX = 0.0;
  velocityY = 0.0;
  displacementX = 0.0;
  displacementY = 0.0;
  prevTime = millis();
}


void calibrateSensors() 
{

  // Calibrar el giroscopio
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;

  // Calibrar el acelerómetro
  accelXOffset = 0;
  accelYOffset = 0;
  accelZOffset = 0;

  for (int i = 0; i < 200; i++) 
  {
     mpu.getEvent(&a, &g, &temp);

     gyroXOffset += g.gyro.x;
     gyroYOffset += g.gyro.y;
     gyroZOffset += g.gyro.z;

    accelXOffset += a.acceleration.x;
    accelYOffset += a.acceleration.y;
    accelZOffset += a.acceleration.z;
    delay(1);
  }

    gyroXOffset /= 200;
    gyroYOffset /= 200;
    gyroZOffset /= 200;

    accelXOffset /= 200;
    accelYOffset /= 200;
    accelZOffset /= 200;
}

void Acc_read(void)
{
/* Get new sensor events with the readings */
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Intervalo de tiempo en segundos
  prevTime = currentTime;

  mpu.getEvent(&a, &g, &temp);

  for(int n=3; n>0; n--)
  {
    accelX[n] = accelX[n-1];
    accelY[n] = accelY[n-1];
    accelZ[n] = accelZ[n-1];
    gyroX[n] = gyroX[n-1];
    gyroY[n] = gyroY[n-1];
    gyroZ[n] = gyroZ[n-1];
  }

  accelX[0] = a.acceleration.x;
  accelY[0]  = a.acceleration.y;
  accelZ[0]  = a.acceleration.z;

  gyroX[0]  = g.gyro.x - gyroXOffset;
  gyroY[0]  = g.gyro.y - gyroYOffset;
  gyroZ[0]  = g.gyro.z - gyroZOffset;

  
    // Calcular el cambio absoluto en aceleración
  float accelXDelta = abs(accelX[0] - accelX[3]);
  float accelYDelta = abs(accelY[0] - accelY[3]);

  // Verificar si el cambio en aceleración es significativo
  bool isMoving = ((int)(accelXDelta*100) > (int)(accelThreshold*100)) || ((int)(accelYDelta*100) > (int)(accelThreshold*100));

  if (!isMoving)
   {
    // Si no hay movimiento significativo, reiniciar velocidad y desplazamiento
     velocityX = 0.0;
     velocityY = 0.0;

     accelXOffset = accelX[0];
     accelYOffset = accelY[0];
     accelZOffset = accelZ[0];

     //Serial.println("CALIBRACION ACC");
  } 
  else 
  {
      // Cálculo del desplazamiento basado en lecturas del acelerómetro
      velocityX = velocityX + ((accelX[0] - accelXOffset) * dt);
      velocityY = velocityY + ((accelY[0] - accelYOffset)  * dt);

      displacementX = displacementX + velocityX * dt;
      displacementY = displacementY + velocityY * dt;
  }

  float gyroXDelta = abs(gyroX[0] - gyroX[3]);
  float gyroYDelta = abs(gyroY[0] - gyroY[3]);

  isMoving = (  ((int)(gyroXDelta*100) > (int)(gyroThreshold*100)) || ((int)(gyroYDelta*100) > (int)(gyroThreshold*100)));
  if (isMoving)
  {

      //Serial.println(dt*1000);
      // Cálculo del ángulo basado en lecturas del giroscopio
      // Calcular ángulo en el eje Z basado en lecturas del giroscopio
      gyroZangle = (gyroZangle + (gyroZ[0] * dt));// * 180.0 / 3.14159;
      

      float gyroXangle_temp = (gyroXangle + (gyroX[0] * dt));// * 180.0 / 3.14159;
      float gyroYangle_temp = (gyroYangle + (gyroY[0] * dt));// * 180.0 / 3.14159;

      // Aplicar filtro complementario a los ángulos
      float accelXangle = atan(accelY[0]  / sqrt(pow(accelX[0] , 2) + pow(accelY[0] , 2)));
      float accelYangle = atan(-1 * accelX[0]  / sqrt(pow(accelY[0] , 2) + pow(accelX[0] , 2)));

      gyroXangle = alpha * gyroXangle_temp + (1 - alpha) * accelXangle;
      gyroYangle = alpha * gyroYangle_temp + (1 - alpha) * accelYangle;

      gyroXangle_grados = (gyroXangle * 180.0) / 3.14159;
      gyroYangle_grados = (gyroYangle * 180.0) / 3.14159;
      gyroZangle_grados = (gyroZangle * 180.0) / 3.14159;
  }

/*
  Serial.print("Ángulo X: ");
  Serial.print(gyroXangle);
  Serial.print("\t Ángulo Y: ");
  Serial.print(gyroYangle);
  Serial.print("\t Desplazamiento X: ");
  Serial.print(displacementX);
  Serial.print("\t Desplazamiento Y: ");
  Serial.println(displacementY);
*/

  /* Print out the values */
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
 */
}