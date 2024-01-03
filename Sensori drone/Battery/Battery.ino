
#define BAT A2
float vBat= 0;
String msg = "";

void setup(void){
  pinMode(BAT, INPUT);
  Serial.begin(115200); 
}



void loop(void) {
   vBat = 2 * analogRead(BAT) * (5.0 / 1023.0);
   Serial.print("vBat: "); Serial.print(vBat);
}
