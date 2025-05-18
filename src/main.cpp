#include <Arduino.h>
//#define SCREEN 
#ifdef SCREEN
  //The One with a Screen, GSM, LED Strip and IrMovement Detector
  #include <SPI.h>

  #include <TFT_eSPI.h>
  #include <HardwareSerial.h>
  #include <Adafruit_NeoPixel.h>
  #include <ESP32Servo.h>
  #define RX
  #define TX 
  #define NEOPIN
  #define MOVEMENTSENSOR
  #define DOORSERVOPIN 
  #define XPT2046_IRQ 36   // T_IRQ
  #define XPT2046_MOSI 32  // T_DIN
  #define XPT2046_MISO 39  // T_OUT
  #define XPT2046_CLK 25   // T_CLK
  #define XPT2046_CS 33    // T_CS

#else
//#define GARAGE
  #ifdef GARAGE
    //NEMA, Ultrasonico, Beeper
    #include <AccelStepper.h>
    #define TRIGGERPIN 33
    #define ECHOPIN 25
    #define STEPPIN 27
    #define DIRPIN 15
    #define ENABLEMOTORPIN 26
    #define BUZZPIN 14
  #else
    #define VENTILATOR
    #ifdef VENTILATOR
      //Hbridge, LED, TEmp, Gyro, Tachometer, Fan, SErvoCuna
    #include <Wire.h>
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BME280.h>
    #include <Adafruit_NeoPixel.h>
    #include <ESP32Servo.h>
    #define SCL 22
    #define SDA 21
    #define SERVOCUNAPIN 19
    #define TACHOPIN 18
    #define MOSFETPIN 23
    //Infrared Existence Sensor, Photoresistor, SErvoPersianas

    #define IRPIN 36 //VP
    #define LIGHTPIN 39 //VN
    #define SERVOPERSIANASPIN 13
    #endif
  #endif
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#if defined SCREEN
typedef struct gateStruct {
  bool Open;
} gateStruct;

SPIClass touchscreenSPI = SPIClass(SPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);


void setup(){

}
void loop(){

}
/*
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  strcpy(myData.a, "THIS IS A CHAR");
  myData.b = random(1,20);
  myData.c = 1.2;
  myData.d = false;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
} */

#elif defined GARAGE

  typedef struct gateStruct {
    bool Open;
  } gateStruct;

  gateStruct GateData;
  AccelStepper StepperMotor(1,STEPPIN,DIRPIN);

  int CurrDistance;
  int Timeout;
  int PreviousTime;
  bool BuzzState;
  double triggerRead();
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);
  void setup(){
    Serial.begin(115200); // Starts the serial communication
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Once ESPNow is successfully Init, we will register for recv CallBack to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));
    
    pinMode(TRIGGERPIN, OUTPUT); // Sets the trigPin as an Output
    pinMode(ECHOPIN, INPUT); // Sets the echoPin as an Input
    
    pinMode(BUZZPIN, OUTPUT); // Sets the trigPin as an Output
    StepperMotor.setAcceleration(100);
    StepperMotor.setMaxSpeed(1000);
    BuzzState=true;
  }
  void loop(){
    CurrDistance=triggerRead();
    triggerRead();
    Serial.print("Distance (CM): ");
    Serial.println(CurrDistance);
    if(CurrDistance<20&&CurrDistance>2){
      if ((millis() - PreviousTime) > 200){
        
        digitalWrite(BUZZPIN, BuzzState);
        Serial.print("Buzz:");
        Serial.println(BuzzState);
        
        BuzzState= !BuzzState;
        PreviousTime=millis();
      }
    }else {
      digitalWrite(BUZZPIN, LOW);
    }
    if(CurrDistance<15){
      StepperMotor.moveTo(0);
    }
    StepperMotor.run();
  }
  double triggerRead(){
    // Clears the trigPin
    digitalWrite(TRIGGERPIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(TRIGGERPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGERPIN, LOW);
    // Reads the ECHOPIN, returns the sound wave travel time in microseconds
    // Calculating the distance
    return pulseIn(ECHOPIN, HIGH) * 0.034/ 2;//Divide the Echo due to being There and Back Distance(Ignores Dopple Effect)
    
    // Prints the distance on the Serial Monitor
  }

  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&GateData, incomingData, sizeof(GateData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("x: ");
    Serial.println(GateData.Open);
    StepperMotor.moveTo(GateData.Open?100:0);
  }

#elif defined VENTILATOR

void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);

typedef struct fasnStruct {
  bool Open;
} fasnStruct;

fasnStruct FanData;
double Frequency;
Servo SeverCuna,SeverCortina;
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
bool i2cGyro, i2ctemper;
int PreviousTime;
volatile int Count;
void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);

/*Interrupts*/
void CountTacho();
void LightFalls();
void LightRise();
void IRRise();
void IRFall();
/*Logic Code */
void setup(){
Serial.begin(115200); // Starts the serial communication
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Wire.begin();
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));
  SeverCuna.attach(SERVOCUNAPIN);
  SeverCortina.attach(SERVOPERSIANASPIN);
  i2cGyro=mpu.begin();
  if (!i2cGyro) {
    Serial.println("Sensor init failed");
  }else{
  Serial.println("Found a MPU-6050 sensor");
  }
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  i2ctemper=bme.begin(0x76);
  if (!i2ctemper) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }else{
    Serial.println("Found BME280");
  }
  pinMode(MOSFETPIN,OUTPUT);
  pinMode(IRPIN,INPUT);
  pinMode(LIGHTPIN,INPUT);
  
  pinMode(TACHOPIN, INPUT_PULLUP);
  attachInterrupt(IRPIN,IRRise,RISING);
  attachInterrupt(IRPIN,IRFall,FALLING);
  attachInterrupt(LIGHTPIN,LightRise,RISING);
  attachInterrupt(LIGHTPIN,LightFalls,FALLING);

}

void loop(){
  if(i2ctemper){
      //int temperature=bme.readTemperature();
    digitalWrite(MOSFETPIN,bme.readTemperature()>30);//Change Depending on High-Low Activation, or P-N Gate  
  }else{
    digitalWrite(MOSFETPIN,HIGH);
  }

  if(i2cGyro){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      int y=map(a.acceleration.y,0,10,0,70);
      int x=map(a.acceleration.x,0,10,0,70);
      int z=map(a.acceleration.z,0,10,0,70);
      SeverCuna.write(10+x+y+z);
    }else{
      SeverCuna.write(90);
    }
  
  if ((millis() - PreviousTime) > 200){
    int StartTime=millis();
    attachInterrupt(TACHOPIN,CountTacho,RISING);//use a PullUp to 3.3v or pullup Input
    while(Count!=-1);//block
    StartTime=millis()-StartTime;
    Frequency=1000.0/StartTime;
    PreviousTime=millis();
  }
}

void IRAM_ATTR CountTacho(){
  Count++;
  if(Count>=2){
    detachInterrupt(TACHOPIN);
    Count=-1;
  }
}

void IRAM_ATTR LightFalls(){
  SeverCortina.write(10);
}

void IRAM_ATTR LightRise(){
  SeverCortina.write(170);
}

void IRAM_ATTR IRRise(){
  
}

void IRAM_ATTR IRFall(){
  
}
void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&FanData, incomingData, sizeof(FanData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(FanData.Open);
  digitalWrite(MOSFETPIN,FanData.Open);
}

#elif defined TOPFLOOR

void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);
typedef struct fasnStruct {
  bool Open;
} fasnStruct;

fasnStruct FanData;

Servo Sever;
int IRNow, IRBefore;
int LIGHTNow, LIGHTBefore;
void poll();
void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);


void setup(){
Serial.begin(115200); // Starts the serial communication
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));
  
  Sever.attach(SERVOPERSIANAS);
  pinMode(LIGHTPIN,INPUT);
  pinMode(IRPIN,INPUT);
  pinMode(MOSFETPIN,OUTPUT);
}

void loop(){
  poll();
  if((IRBefore^IRNow)&&IRNow){
    //JustPressed IR
  }
  if((LIGHTBefore^LIGHTNow)){
    if(LIGHTNow){
      //JustPressed LIght
      Sever.write(170);
    }else{
      //JustUnpressed LIght
      Sever.write(10);
    }
    
  }

}

void poll(){
  IRBefore=IRNow;
  LIGHTBefore=LIGHTNow;
  IRNow=digitalRead(IRPIN);
  LIGHTNow=digitalRead(LIGHTPIN);
}

void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&FanData, incomingData, sizeof(FanData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(FanData.Open);
  digitalWrite(MOSFETPIN,FanData.Open);
}


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