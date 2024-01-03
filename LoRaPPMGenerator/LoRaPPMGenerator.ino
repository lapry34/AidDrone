//this programm will put out a PPM signal

#include <SPI.h>
#include <LoRa.h>

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 12  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////


/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

bool flagBlink = false;

void setup(){  

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();  //IMPORTANTISSIMO, NON È TROLL, NON CANCELLARE
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

  sei(); //IMPORTANTISSIMO, NON È TROLL, NON CANCELLARE
  Serial.println("LoRa PPM");
  
  if (!LoRa.begin(433E6)) {
    while (true) Serial.println("LoRa non trovata!");
  }
}

String str = "";

void loop(){

  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    str = "";

    // read packet
    while (LoRa.available()) 
      str += (char)LoRa.read();
  
    sscanf(str.c_str(), "%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d", &ppm[0], &ppm[1], &ppm[2], &ppm[3], &ppm[4], &ppm[5], &ppm[6], &ppm[7], &ppm[8], &ppm[9], &ppm[10], &ppm[11]);
  } 
  delay(10);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
