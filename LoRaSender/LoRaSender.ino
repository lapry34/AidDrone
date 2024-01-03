/*
 *  LoRaSender.ino
 *  
 *  Codice presente nel controller custom
 * 
 *  Gabriele Onorato & Federico Gerardi
 */

#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


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

// pin antenna HC12 (corrispondenti sulla scritta)
#define HC12RXD 5
#define HC12TXD 4
#define HC12SET 3
// variabili dati antenna

char HC12ByteIn;               // Temporary variable
String HC12ReadBuffer = "";    // Read/Write Buffer 1 for HC12
boolean HC12End = false;       // Flag to indiacte End of HC12 String
boolean commandMode = false;   // Send AT commands

SoftwareSerial HC12(HC12TXD, HC12RXD);

// valori pedali
int pedalLeft = 0, pedalRight = 0;

// indirizzo MPU6050
#define MPU 0x68
MPU6050 mpu;

//pin e capienza batteria
#define BAT A2   
float vBat = 0;

//pin buzzer e led
#define BUZZER 7
#define ledGREEN 9
#define ledBLUE 8
#define ledRED 6

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
 
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// Valori PPM da inviare
unsigned throttlePPM = 1000, yawPPM = 1500, pitchPPM = 1500, rollPPM = 1500,
aux1PPM = 1500, aux2PPM = 1500, aux3PPM = 1500, aux4PPM = 1500;
bool flagPedals = false, flagArm = false;

// Valori del Trim
short pitchTrim = 0,//-5,//-11,//-25,//-15,
      rollTrim = 25,//2,
      yawTrim = -35;//-45;   

// Timer
uint64_t timer = 0, pedalTimer = 0;


// Valori Pitch, Roll e Yaw
float yaw = 0, pitch = 0, roll = 0;

void setup(void) {
  Serial.begin(115200);
  HC12ReadBuffer.reserve(128);  // Reserve 128 bytes for HC12 data

  init_MPU();
  
  pinMode(HC12SET, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ledRED, OUTPUT);
  pinMode(ledGREEN, OUTPUT);
  pinMode(ledBLUE, OUTPUT);
  pinMode(vBat, INPUT);

  init_HC12();
  
  Serial.println("Inizializzazione LoRa");

  if (!LoRa.begin(433E6)) {
    while (true) Serial.println("LoRa non trovata!");
  }
}

void loop(void) {
  timer = millis();
  FunctionsHC12();
  FunctionsMPU(); // Acquisisco Yaw, Pitch e Roll.

  vBat = 2 * analogRead(BAT) * (5.0 / 1023.0);

  if(vBat >= 7.5) {
    digitalWrite(ledGREEN, HIGH);
    digitalWrite(ledRED, LOW);
    digitalWrite(ledBLUE, LOW);

    digitalWrite(BUZZER, LOW);
  }
  else if(vBat > 7 && vBat < 7.5) {
    digitalWrite(ledGREEN, HIGH);
    digitalWrite(ledRED, HIGH);
    digitalWrite(ledBLUE, LOW);

    digitalWrite(BUZZER, LOW);
  }  
  else if(vBat <= 7) { 
    digitalWrite(ledGREEN, LOW);
    digitalWrite(ledRED, HIGH);
    digitalWrite(ledBLUE, LOW);
    
  // digitalWrite(BUZZER, HIGH);
  } 
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.print(yaw); 
  Serial.print(" PedalLeft: "); Serial.print(pedalLeft);
  Serial.print(" PedalRight: "); Serial.println(pedalRight);
  Serial.print(" vBat: "); Serial.println(vBat); 


 // ------------------- Gestione Roll, Pitch e Yaw ---------------------
 
  if((roll > -10 && roll < 10) && (pitch > -10 && pitch < 10) && (yaw > -10 && yaw < 10)){
    Serial.println("Non faccio nulla");
    pitchPPM =  1500 + pitchTrim;
    yawPPM = 1500 + yawTrim; 
    rollPPM = 1500 + rollTrim;
  } else if(!(roll > -10 && roll < 10) && (pitch > -10 && pitch < 10) && (yaw > -10 && yaw < 10)) { //Muove roll
    pitchPPM =  1500 + pitchTrim; 
    yawPPM = 1500 + yawTrim;
    if(roll > 0) rollPPM = map(roll, 90, 0, 1200, 1500 + rollTrim);
    else rollPPM = map(roll, 0, -90, 1500 + rollTrim, 1800);
    // rollPPM = map(roll, 90, -90, 1300, 1700);
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -10 && roll < 10) && !(pitch > -10 && pitch < 10) && (yaw > -10 && yaw < 10)) { //Muove pitch

    if(pitch > 0) pitchPPM = map(pitch, 90, 0, 1200, 1500 + pitchTrim);
    else pitchPPM = map(pitch, 0, -90, 1500 + pitchTrim, 1800);
    
    //pitchPPM = map(pitch, 90, -90, 1300, 1700);
    
    yawPPM = 1500 + yawTrim;
    rollPPM = 1500 + rollTrim;
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -10 && roll < 10) && !(pitch > -10 && pitch < 10) && (yaw > -10 && yaw < 10)) { //Muove pitch e roll
    
    if(pitch > 0) pitchPPM = map(pitch, 90, 0, 1200, 1500 + pitchTrim);
    else pitchPPM = map(pitch, 0, -90, 1500 + pitchTrim, 1800);
    //pitchPPM = map(pitch, 90, -90, 1300, 1700);
    yawPPM = 1500 + yawTrim;
    if(roll > 0) rollPPM = map(roll, 90, 0, 1200, 1500 + rollTrim);
    else rollPPM = map(roll, 0, -90, 1500 + rollTrim, 1800);
    //rollPPM = map(roll, 90, -90, 1300, 1700);
    
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -10 && roll < 10) && (pitch > -10 && pitch < 10) && !(yaw > -10 && yaw < 10)) { //Muove yaw
    
    pitchPPM = 1500 + pitchTrim;
    if(yaw > 0) yawPPM = map(yaw, 0, 180, 1500 + yawTrim, 2000);
    else yawPPM = map(yaw, 0, -180, 1500 + yawTrim, 1000);
    //yawPPM = map(yaw, -180, 180, 1000, 2000);
    rollPPM = 1500 + rollTrim;
    
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -10 && roll < 10) && (pitch > -10 && pitch < 10) && !(yaw > -10 && yaw < 10)) { //Muove yaw e roll
    
    pitchPPM = 1500 + pitchTrim;
    if(yaw > 0) yawPPM = map(yaw, 0, 180, 1500 + yawTrim, 2000);
    else yawPPM = map(yaw, 0, -180, 1500 + yawTrim, 1000);
    //yawPPM = map(yaw, -180, 180, 1000, 2000);
    if(roll > 0) rollPPM = map(roll, 90, 0, 1200, 1500 + rollTrim);
    else rollPPM = map(roll, 0, -90, 1500 + rollTrim, 1800);
    //rollPPM = map(roll, 90, -90, 1300, 1700);
    
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if((roll > -10 && roll < 10) && !(pitch > -10 && pitch < 10) && !(yaw > -10 && yaw < 10)) { //Muove yaw e pitch
    
    if(pitch > 0) pitchPPM = map(pitch, 90, 0, 1200, 1500 + pitchTrim);
    else pitchPPM = map(pitch, 0, -90, 1500 + pitchTrim, 1800);
    //pitchPPM = map(pitch, 90, -90, 1300, 1700);
    if(yaw > 0) yawPPM = map(yaw, 0, 180, 1500 + yawTrim, 2000);
    else yawPPM = map(yaw, 0, -180, 1500 + yawTrim, 1000);
    //yawPPM = map(yaw, -180, 180, 1000, 2000);
    rollPPM = 1500 + rollTrim;
    
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } else if(!(roll > -10 && roll < 10) && !(pitch > -10 && pitch < 10) && !(yaw > -10 && yaw < 10)) { //Muove yaw e pitch e roll
    
    if(pitch > 0) pitchPPM = map(pitch, 90, 0, 1200, 1500 + pitchTrim);
    else pitchPPM = map(pitch, 0, -90, 1500 + pitchTrim, 1800);
    //pitchPPM = map(pitch, 90, -90, 1300, 1700);
    if(yaw > 0) yawPPM = map(yaw, 0, 180, 1500 + yawTrim, 2000);
    else yawPPM = map(yaw, 0, -180, 1500 + yawTrim, 1000);
    //yawPPM = map(yaw, -180, 180, 1000, 2000);
    if(roll > 0) rollPPM = map(roll, 90, 0, 1200, 1500 + rollTrim);
    else rollPPM = map(roll, 0, -90, 1500 + rollTrim, 1800);
    //rollPPM = map(roll, 90, -90, 1300, 1700);
    
    Serial.print("PitchPPM: ");
    Serial.print(pitchPPM);
    Serial.print(" RollPPM: "); 
    Serial.print(rollPPM);
    Serial.print(" YawPPM: ");
    Serial.println(yawPPM);
  } 
  // ----------------------- Gestione Velocità, Armamento e Disarmo -----------------------
  /*
   * Attraverso la pressione dei pulsanti possiamo gestire la velocità, l'armamento e il disarmo
   */
  if (pedalLeft > 700 && pedalRight > 700 && !flagPedals) {
    // Prima pressione del pulsante
    pedalTimer = timer;
    flagPedals = true;
    Serial.println("Pedali premuti!");
  } 
  else if (pedalLeft > 700 && pedalRight > 700 && flagPedals && (pedalTimer + 850 < timer) && !flagArm) {
    // Armamento
    Serial.println("Armamento in corso!");
    pedalTimer = 0;
    flagPedals = false;
    flagArm = true;
    arm();
  }
  else if (pedalLeft > 700 && pedalRight > 700 && flagPedals && (pedalTimer + 850 < timer) && flagArm) {
    // Disarmamento
    Serial.println("Disarmamento in corso!");
    pedalTimer = 0;
    flagPedals = false;
    flagArm = false;
    disarm();
  } 
  else if (pedalLeft < 350 && pedalRight < 350) {
    // Eseguito quando i pulsanti non sono premuti
    Serial.print("Pulsanti non premuti,");
    Serial.print(" stato: ");
    Serial.print((flagArm ? "armato," : "disarmato,"));
    Serial.print(" velocità attuale: ");
    Serial.println(throttlePPM);
    pedalTimer = 0;
    flagPedals = false;
  }
  else if (pedalLeft > 350) {
    // Freno premuto
    if (throttlePPM > 1000) throttlePPM -= map(analogRead(pedalLeft), 350, 1000, 0, 60);
    Serial.print("Diminuzione velocità: \t");
    Serial.println(throttlePPM);
  }
  else if (pedalRight > 350) {
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

void init_MPU(void) {
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) 
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
   
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-2);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(-8);
  mpu.setXAccelOffset(-519);
  mpu.setYAccelOffset(339);
  mpu.setZAccelOffset(1133); 
 
// make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // impostiamo il filtro passa basso a 42Hz
    setDLPFMode(MPU6050_DLPF_BW_42);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  return;  
}

void FunctionsMPU(void) {
  while (!mpuInterrupt && fifoCount < packetSize);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/M_PI; roll = ypr[1] * 180/M_PI; pitch = ypr[2] * 180/M_PI;
  }
  
  return;  
}
void init_HC12(void) {
  
  Serial.println("Inizializzazione HC12");  
  digitalWrite(HC12SET, HIGH); // Enter Transparent mode
  delay(80);                      // 80 ms delay before operation per datasheet
  HC12.begin(9600);               // Open software serial port to HC12
  delay(80);
  digitalWrite(HC12SET, LOW);
  delay(80);
  HC12.write("AT+C100");
  delay(80);
  digitalWrite(HC12SET, HIGH);
  delay(80);
  
  return;  
}
void FunctionsHC12(void) {
  
  while (HC12.available()) {                    // While Arduino's HC12 soft serial rx buffer has data
    HC12ByteIn = HC12.read();                   // Store each character from rx buffer in byteIn
    if (HC12ByteIn == '\n') {                   // At the end of the line
      HC12ReadBuffer += char(HC12ByteIn);
      HC12End = true;                           // Set HC12End flag to true
      break;
    }
    HC12ReadBuffer += char(HC12ByteIn);         // Write each character of byteIn to HC12ReadBuffer  
  }

  if(HC12End) {
    Serial.print(HC12ReadBuffer);
    Serial.print("\t");
    Serial.println(HC12ReadBuffer.length());

    if (HC12ReadBuffer.length() == 9){
          sscanf(HC12ReadBuffer.c_str(), "%d-%d\n", &pedalLeft, &pedalRight);
    //  pedalLeft = (String(HC12ReadBuffer[0]).toInt() * 100) + (String(HC12ReadBuffer[1]).toInt() * 10) + (String(HC12ReadBuffer[2]).toInt());
    //  pedalRight = (String(HC12ReadBuffer[4]).toInt() * 100) + (String(HC12ReadBuffer[5]).toInt() * 10) + (String(HC12ReadBuffer[6]).toInt());
    }
    else {
      if(pedalLeft >= 270) pedalLeft -= 100;
      if(pedalRight >= 270) pedalRight -= 100;
    }
    
    HC12ReadBuffer = "";
    HC12End = false;
  }
  
  return;  
}

void dmpDataReady()
{
  mpuInterrupt = true;
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
