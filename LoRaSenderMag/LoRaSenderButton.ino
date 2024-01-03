/*
 *  LoRaSenderButton.ino
 *  
 *  Codice presente nel controller custom
 * 
 *  Gabriele Onorato & Federico Gerardi
 */

#include <QMC5883LCompass.h>
#include <SPI.h>
#include <LoRa.h>
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

// Pin pulsanti
#define pedalLeft A0
#define pedalRight A2

#define MPU 0x68

QMC5883LCompass compass;

// Valori PPM da inviare
unsigned throttlePPM = 1000, yawPPM = 1500, pitchPPM = 1500, rollPPM = 1500,
aux1PPM = 1500, aux2PPM = 1500, aux3PPM = 1500, aux4PPM = 1500;
bool flagPedals = false, flagArm = false;

// Valori del Trim
short pitchTrim = 23,//-5,//-11,//-25,//-15,
      rollTrim = 14,//2,
      yawTrim = -25;//-45;   

// Timer
unsigned long timer = 0;
unsigned long pedalTimer = 0;

// Valori Gyro e Accel
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;



// Valori Pitch, Roll e Yaw
double yaw = 0, pitch = 0, roll = 0;

double azimuth = 0, yawOffset = 0;


void setup() {
  Serial.begin(115200);

  init_MPU();
  init_QMC();
  
  pinMode(pedalLeft, INPUT);
  pinMode(pedalRight, INPUT);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    while (true) Serial.println("LoRa non trovata!");
  }
}

void loop() {
  timer = millis();
  
  FunctionsMPU(); // Acquisisco AcX, AcY, AcZ.
  FunctionsQMC(); // Acquisisco bussola.
  
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.println(yaw);


 // ------------------- Gestione Roll, Pitch e Yaw ---------------------
 
  if((roll > -15 && roll < 15) && (pitch > -15 && pitch < 15) && (yaw < -175 || yaw > 175)){
    Serial.println("Non faccio nulla");
    pitchPPM =  1500 + pitchTrim;// map(analogRead(pitchPIN), 0, 1024, 1000, 2000);
    yawPPM = 1500 + yawTrim; // map(analogRead(yawPIN), 0, 1024, 1000, 2000);
    rollPPM = 1500 + rollTrim;//1500;//1453;// map(analogRead(rollPIN), 0, 1024, 1000, 2000);
  } else if(!(roll > -15 && roll < 15) && (pitch > -15 && pitch < 15) && (yaw < -175 || yaw > 175)) { //Muove roll
    pitchPPM =  1500 + pitchTrim; 
    yawPPM = 1500 + yawTrim;
    rollPPM = map(roll, 90, -90, 1000, 2000);
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -15 && roll < 15) && !(pitch > -15 && pitch < 15) && (yaw < -175 || yaw > 175)) { //Muove pitch
    pitchPPM = map(pitch, 90, -90, 1000, 2000);
    yawPPM = 1500 + yawTrim;
    rollPPM = 1500 + rollTrim;
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -15 && roll < 15) && !(pitch > -15 && pitch < 15) && (yaw < -175 || yaw > 175)) { //Muove pitch e roll
    pitchPPM = map(pitch, 90, -90, 1000, 2000);
    yawPPM = 1500 + yawTrim;
    rollPPM = map(roll, 90, -90, 1000, 2000);
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -15 && roll < 15) && (pitch > -15 && pitch < 15) && !(yaw < -175 || yaw > 175)) { //Muove yaw
    pitchPPM = 1500 + pitchTrim;
    if (yaw > 0) yawPPM = map(yaw, 0, 180, 1600, 1200);
    else yawPPM = map(yaw, -180, 0, 1500, 1700);
    rollPPM = 1500 + rollTrim;
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -15 && roll < 15) && (pitch > -15 && pitch < 15) && !(yaw < -175 || yaw > 175)) { //Muove yaw e roll
    pitchPPM = 1500 + pitchTrim;
    if (yaw > 0) yawPPM = map(yaw, 0, 180, 1600, 1200);
    else yawPPM = map(yaw, -180, 0, 1500, 1700);
    rollPPM = map(roll, 90, -90, 1000, 2000);
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -15 && roll < 15) && !(pitch > -15 && pitch < 15) && !(yaw < -175 || yaw > 175)) { //Muove yaw e pitch
    pitchPPM = map(pitch, 90, -90, 1000, 2000);
    if (yaw > 0) yawPPM = map(yaw, 0, 180, 1600, 1200);
    else yawPPM = map(yaw, -180, 0, 1500, 1700);
    rollPPM = 1500 + rollTrim;
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -15 && roll < 15) && !(pitch > -15 && pitch < 15) && !(yaw < -175 || yaw > 175)) { //Muove yaw e pitch e roll
    pitchPPM = map(pitch, 90, -90, 1000, 2000);
    if (yaw > 0) yawPPM = map(yaw, 0, 180, 1600, 1200);
    else yawPPM = map(yaw, -180, 0, 1500, 1700);
    rollPPM = map(roll, 90, -90, 1000, 2000);
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } 
  Serial.print(analogRead(pedalLeft));
  Serial.print('\t');
  Serial.println(analogRead(pedalRight));
  // ----------------------- Gestione Velocità, Armamento e Disarmo -----------------------
  /*
   * Attraverso la pressione dei pulsanti possiamo gestire la velocità, l'armamento e il disarmo
   */
  if (analogRead(pedalLeft) > 350 && analogRead(pedalRight) > 350 && !flagPedals) {
    // Prima pressione del pulsante
    pedalTimer = timer;
    flagPedals = true;
    Serial.println("Pedali premuti!");
  } 
  else if (analogRead(pedalLeft) > 350 && analogRead(pedalRight) > 350 && flagPedals && (pedalTimer + 850 < timer) && !flagArm) {
    // Armamento
    Serial.println("Armamento in corso!");
    pedalTimer = 0;
    flagPedals = false;
    flagArm = true;
    arm();
  }
  else if (analogRead(pedalLeft) > 350 && analogRead(pedalRight) > 350 && flagPedals && (pedalTimer + 850 < timer) && flagArm) {
    // Disarmamento
    Serial.println("Disarmamento in corso!");
    pedalTimer = 0;
    flagPedals = false;
    flagArm = false;
    disarm();
  } 
  else if (!analogRead(pedalLeft) > 350 && !analogRead(pedalRight) > 350) {
    // Eseguito quando i pulsanti non sono premuti
    Serial.print("Pulsanti non premuti,");
    Serial.print(" stato: ");
    Serial.print((flagArm ? "armato," : "disarmato,"));
    Serial.print(" velocità attuale: ");
    Serial.println(throttlePPM);
    pedalTimer = 0;
    flagPedals = false;
  }
  else if (analogRead(pedalLeft) > 350) {
    // Freno premuto
    if (throttlePPM > 1000) throttlePPM -= map(analogRead(pedalLeft), 350, 1000, 0, 100);
    Serial.print("Diminuzione velocità: \t");
    Serial.println(throttlePPM);
  }
  else if (analogRead(pedalRight) > 350) {
    // Acceleratore premuto  
    if (throttlePPM < 2000) throttlePPM += map(analogRead(pedalRight), 350, 1000, 0, 100);
    Serial.print("Aumento velocità: \t");
    Serial.println(throttlePPM);
  }

  sendData();
 
}


void arm(void) {
  // Funzione di armamento
  throttlePPM = 1000;
  sendData();
  delay(333);
  aux4PPM = 2000;
  sendData();

  Serial.println("Armato");

  delay(1000);

  timer = millis();
  return;
}


void disarm(void) {
  // Funzione di disarmo
  throttlePPM = 1000;
  sendData();
  delay(333);
  aux4PPM = 1000;
  sendData();

  delay(1000);

  Serial.println("Disarmato!");

  throttlePPM = 1000;

  sendData();
  timer = millis();
  return;
}

void init_QMC(void){
  compass.init();

  delay(333);
   
  compass.read();
  yawOffset = compass.getAzimuth(); //genero offset per porre l'accensione come 0
  
  return;
}

void FunctionsQMC(void){
  
  compass.read(); //Acquisisco dati bussola
  azimuth = compass.getAzimuth() - yawOffset; //prendo gradi bussola fissati a 0
  if(azimuth < 0) azimuth += 360; //sistemo l'offset
  
  yaw = map(azimuth, 0, 360, -180, 180); //mapppo in modo che il futuro mapping torni più facile

  return;
}

void sendData(void) {
  // Funzione di invio dati
  LoRa.beginPacket();
  LoRa.print(rollPPM);
  LoRa.print("-");
  LoRa.print(pitchPPM);
  LoRa.print("-");
  LoRa.print(throttlePPM);
  LoRa.print("-");
  LoRa.print(yawPPM);
  LoRa.print("-");
  LoRa.print(aux1PPM);
  LoRa.print("-");
  LoRa.print(aux2PPM);
  LoRa.print("-");
  LoRa.print(aux3PPM);
  LoRa.print("-");
  LoRa.print(aux4PPM);
  LoRa.print("-1500-1500-1500-1500");
  LoRa.endPacket();
  return;
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
