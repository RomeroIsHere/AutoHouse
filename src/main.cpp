#include <Arduino.h>

//#define SCREEN 
#ifdef SCREEN
//The One with a Screen, GSM, LED Strip and IrMovement Detector
#include <TFT_eSPI.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

#else
#define GARAGE
#ifdef GARAGE
//NEMA, Ultrasonico, Beeper
#include <AccelStepper.h>
#define TRIGGERPIN 2
#define ECHOPIN 4
#define STEPPIN 13
#define DIRPIN 15
#define ENABLEMOTORPIN 12
#else
#define VENTILATOR
#ifdef VENTILATOR
  //Hbridge, LED, TEmp, Gyro, Tachometer SErvoCuna
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

#else
#define TOPFLOOR
#ifdef TOPFLOOR
  //Infrared Existence Sensor, Photoresistor, SErvoPersianas, Ventilador
#include <ESP32Servo.h>

#else
  #define SENDER
    #ifdef SENDER

    #else
    #define RECEIVER

    #endif
#endif
#endif
#endif
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#if defined SCREEN

#elif defined GARAGE

typedef struct gateStruct {
  bool Open;
} gateStruct;

gateStruct GateData;
AccelStepper StepperMotor(1,STEPPIN,DIRPIN);
int triggerRead();
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void setup(){
  Serial.begin(115200); // Starts the serial communication
  pinMode(TRIGGERPIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHOPIN, INPUT); // Sets the echoPin as an Input
  StepperMotor.setAcceleration(100);
  StepperMotor.setMaxSpeed(1000);
}
void loop(){
  
  Serial.print("Distance (CM): ");
  Serial.println(triggerRead());
  if(triggerRead()<5){
    StepperMotor.moveTo(0);
  }
  StepperMotor.run();
}
int triggerRead(){
  // Clears the trigPin
  digitalWrite(TRIGGERPIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGERPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGERPIN, LOW);
  // Reads the ECHOPIN, returns the sound wave travel time in microseconds
  // Calculating the distance
  return pulseIn(ECHOPIN, HIGH) * 0.034 / 2;//Divide the Echo due to being There and Back Distance(Ignores Dopple Effect)
  // Prints the distance on the Serial Monitor
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&GateData, incomingData, sizeof(GateData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(GateData.Open);
  StepperMotor.moveTo(100);
}

#elif defined VENTILATOR

#elif defined TOPFLOOR

#endif

//The Existence Sensor Needs to talk to the Baby Loof

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
#if defined SENDER
typedef struct test_struct {
  int x;
  int y;
} test_struct;
uint8_t broadcastAddress1[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
{0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};
//88:13:BF:68:B3:D4
test_struct test;
esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
#elif defined RECEIVER

test_struct myData;
bool higlowflip;
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(myData.x);
  Serial.print("y: ");
  Serial.println(myData.y);
  Serial.println();
  digitalWrite(2,higlowflip);
  higlowflip=(!higlowflip);
}


void setup() {
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  higlowflip=true;
  pinMode(2,OUTPUT);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  test.x = random(0,20);
  test.y = random(0,20);
 
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);    
}
#endif