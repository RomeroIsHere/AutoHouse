#include <Arduino.h>
//No operator
#define RXp2 26
#define TXp2 27
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);  
  Serial.println("iniciando monitor serial comandos at: ");
delay(2000);
}
void loop() {
   
  Serial.println("inicializando el sim800c");
  Serial2.print("at+cmgf=1\r");
  delay(300);
  Serial.println(Serial2.readString());
  Serial2.print("at+cmgs=\"+524741422095\"\r");
  delay(300);
  Serial2.print("aqui manda el mensaje a fastidiar");
  Serial.println(Serial2.readString());
  Serial2.print((char)26);
    delay(300);
  Serial.println(Serial2.readString());
  
    delay(1000);
    Serial.println("finalizo el mensaje");
}
