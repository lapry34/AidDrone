#include <Wire.h>
#include "I2Cdev.h"


// Configurazione dei filtri passa basso
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3
#define MPU6050_DLPF_BW_256         0x00 //256Hz
#define MPU6050_DLPF_BW_188         0x01 //188Hz
#define MPU6050_DLPF_BW_98          0x02 //98Hz
#define MPU6050_DLPF_BW_42          0x03 //42Hz
#define MPU6050_DLPF_BW_20          0x04 //20Hz
#define MPU6050_DLPF_BW_10          0x05 //10Hz
#define MPU6050_DLPF_BW_5           0x06 //5Hz

#define MPU 0x68

// Valori Gyro e Accel
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;



// Valori Pitch e Roll
double pitch = 0, roll = 0;

void setup(){
  
  Serial.begin(9600);
  Serial.print("asd");
  init_MPU();
   Serial.print("asd");
}
void loop(){
  FunctionsMPU(); // Acquisisco AcX, AcY, AcZ.
  
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.println();
}


double FunctionsPitchRoll(double A, double B, double C){
  //Funzione per il calcolo degli angoli Pitch e Roll
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B*B) + (C*C);
  DatoB = sqrt(DatoB);
  
  Value = atan2(DatoA, DatoB);
  Value = Value * 180/3.14;
  
  return Value;
}


void init_MPU(void){
  // Funzione di inizializzazione dell'MPU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(1000);
  
  setDLPFMode(MPU6050_DLPF_BW_42); //Filtro Passa Basso
  delay(1000);
  return;
}

void FunctionsMPU(void){
  //Funzione per l'acquisizione degli assi X,Y,Z del MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  roll = FunctionsPitchRoll(AcX, AcY, AcZ);   // Calcolo angolo Roll
  pitch = FunctionsPitchRoll(AcY, AcX, AcZ);  // Calcolo angolo Pitch
  
  return;
}

/* Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 */
void setDLPFMode(uint8_t mode) {
    I2Cdev::writeBits(MPU, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
    return;
}
