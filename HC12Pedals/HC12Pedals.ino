

#include <SoftwareSerial.h>

#define HC12RXD 5                     // "RXD" Pin on HC12
#define HC12TXD 4                     // "TXD" Pin on HC12
#define HC12SET 3                     // "SET" Pin on HC12

#define BAT A2
float vBat = 0;

#define ledRED 6
#define ledBLUE 8 
#define ledGREEN 9

#define BUZZER 7


// Software Serial ports Rx and Tx are opposite the HC12 Rx and Tx
// Create Software Serial Port for HC12
SoftwareSerial HC12(HC12TXD, HC12RXD);

#define pedalLeftPin A0
#define pedalRightPin A1

String pedalLeft = "", pedalRight = "", msg = "";

void setup(void) {

  pedalLeft.reserve(128);                   // Reserve 64 bytes for Serial message input
  pedalRight.reserve(128);                  // Reserve 64 bytes for HC12 message input
  msg.reserve(128);
  pinMode(HC12SET, OUTPUT);                  // Output High for Transparent / Low for Command
  pinMode(ledRED, OUTPUT);
  pinMode(ledBLUE, OUTPUT);
  pinMode(ledGREEN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(pedalLeftPin, INPUT);
  pinMode(pedalRightPin, INPUT);
  pinMode(BAT, INPUT);

  Serial.begin(115200);                           // Open serial port to computer
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
}

void loop(void) {



  vBat = (2 * analogRead(BAT) * (5.0 / 1023.0)) + 1;
  if(vBat > 8.2) vBat = 8.2;
  
  if(vBat < 7.25) {
  digitalWrite(ledRED, HIGH);
  digitalWrite(ledBLUE, LOW);  
  digitalWrite(ledGREEN, LOW);
  //digitalWrite(BUZZER, HIGH);
  }
  else if(vBat >= 7.25) {
    digitalWrite(ledGREEN, HIGH);
    digitalWrite(ledBLUE, LOW);
    digitalWrite(ledRED, LOW); 
    digitalWrite(BUZZER, LOW);
    
  }
  Serial.print("vBat: "); Serial.print(vBat);  Serial.print("\t");

  msg = String(analogRead(pedalRightPin)) + "-" + String(analogRead(pedalLeftPin));
  HC12.println(msg);
  Serial.println(msg);

  delay(80);

}
