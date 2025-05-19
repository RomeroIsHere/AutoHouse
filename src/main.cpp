#include <Arduino.h>
#define SCREEN 
#ifdef SCREEN
  //The One with a Screen, GSM, LED Strip and IrMovement Detector
  #include <SPI.h>

  #include <TFT_eSPI.h>
  #include <HardwareSerial.h>
  #include <Adafruit_NeoPixel.h>
  #include <ESP32Servo.h>
  #define RX 26
  #define TX 27
  #define NEOPIN 16
  #define MOVEMENTSENSOR 15
  #define DOORSERVOPIN 13
  #define XPT2046_IRQ 36   // T_IRQ
  #define XPT2046_MOSI 32  // T_DIN
  #define XPT2046_MISO 39  // T_OUT
  #define XPT2046_CLK 25   // T_CLK
  #define XPT2046_CS 33    // T_CS
  #define TOUCHDOORSWITCH 14
  #define THRESHOLD 30
  /*Screen Definitions*/
  #define SCREEN_WIDTH 320 // 320/(4)=80 -> S{0,80,160,240}, F(x)=x+5, F(S)={5,85,165,245}, W=75
  #define SCREEN_HEIGHT 240 // 240/(3)=80 
  #define FONT_SIZE 2
  #define FONT 4
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

typedef struct fasnStruct {
  double Frequency;
  double Temperature;
} fasnStruct;

enum ScreenMode{
  NULLMODE=-1,
PASSWORDMODE,
MONITORMODE,
MANUALCONTROLMODE
};

TFT_eSPI tft = TFT_eSPI();
Adafruit_NeoPixel ws2812;

HardwareSerial GSMSerial(1);
Servo SeverDoor;


int Coordinates[12][4]={
  {5,5,75,75},
  {5,85,75,75},
  {5,165,75,75},

  {85,5,75,75},
  {85,85,75,75},
  {85,165,75,75},

  {165,5,75,75},
  {165,85,75,75},
  {165,165,75,75},
  
  {245,5,75,75},
  {245,85,75,75},
  {245,165,75,75}
};

uint8_t broadcastAddress[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
{0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};



gateStruct gateData{true};
fasnStruct FanData{0.0};

esp_now_peer_info_t peerInfo;

uint16_t xTouch, yTouch;
uint16_t PreviousXTouch, PreviousYTouch;
bool TouchNow, TouchPrevious, touchedBefore, GarageDoor;


String PasswordInsert="";

int PreviousTime;
enum ScreenMode mode;
void updateSerial();
void fillPasswordMenu();
bool inBoundingBox(int ah, int aw,int bh,int bw, int x, int y);

void PassWordScreen();
void MonitorScreen();
void ManualScreen();

void PassWordSetup();
void MonitorSetup();
void ManualSetup();

void OpenDoor();
void CloseDoor();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void setup(){
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  
  GSMSerial.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Initializing...");
  delay(1000);
  GSMSerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  GSMSerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  GSMSerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  GSMSerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));


  tft.init();
  tft.setRotation(3);
  tft.fillScreen(0xFFFF);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  ws2812=Adafruit_NeoPixel(8, NEOPIN, NEO_GRB + NEO_KHZ800);
  ws2812.begin();
  SeverDoor.attach(DOORSERVOPIN);
  pinMode(MOVEMENTSENSOR,INPUT);
  pinMode(TOUCHDOORSWITCH,INPUT);
  attachInterrupt(MOVEMENTSENSOR,MovementRiseInterrupt,RISING);
  mode=NULLMODE;
  GarageDoor=false;
}

void loop(){
  poll();
  if ((millis() - PreviousTime) > 2000){
    ws2812.fill(0xFFFF);
    ws2812.show();
    PreviousTime=millis();
  }
  switch(mode){
    case PASSWORDMODE:
      PassWordScreen();
    break;
    case MONITORMODE:
      MonitorScreen();
    break;
    case MANUALCONTROLMODE:
      ManualScreen();
    break;
    default:
    
    PassWordSetup();
    break;
  }
}
void PassWordScreen(){

  if(tft.getTouch(&xTouch,&yTouch)){
    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 9:
            //Try the Password
              if(PasswordInsert.equals("159483726")){
                OpenDoor();
                MonitorSetup();
              }else{
                tft.fillScreen(0x1800);
                delay(100);
                tft.fillScreen(0xF800);
                delay(100);
                tft.fillScreen(0xFFFF);
                delay(100);
                tft.fillScreen(0xF800);
                delay(100);
                tft.fillScreen(0xFFFF);
                delay(100);
                PassWordSetup();
              }
            break;
            case 10:
            //Try the Delete
            if(PasswordInsert.length()!=0){
              PasswordInsert=PasswordInsert.substring(0,PasswordInsert.length()-1);
            }else{
              PasswordInsert.clear();
            }
              
            break;
            case 11:
            //Try the Reset
              
              PasswordInsert.clear();
            break;
            default:
            PasswordInsert.concat(ii+1);
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
  
}

void MonitorScreen(){
if(tft.getTouch(&xTouch,&yTouch)){
    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){//Find Quadrant Collided
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 9:
            //Try the Other Mode
            ManualSetup();         
            break;
            case 11:
            //Try the Locking Routine
              
              
            CloseDoor();
            break;
            default:
            //No Operation;
            break;
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
}

void ManualScreen(){
if(tft.getTouch(&xTouch,&yTouch)){
    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){//Find Quadrant Collided
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 9:
            //Try the Other Mode
            ManualSetup();         
            break;
            case 11:
            //Try the Locking Routine
              
              
            CloseDoor();
            break;
            default:
            //No Operation;
            break;
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
}

void PassWordSetup(){
  mode=PASSWORDMODE;
  fillPasswordMenu();
  PasswordInsert.clear();
  CloseDoor();
}

void MonitorSetup(){
mode=MONITORMODE;
  fillMonitorMenu();
}
void ManualSetup(){
  mode=MANUALCONTROLMODE;
  fillControlMenu();
}


void OpenDoor(){
  if(GarageDoor){
    gateData.Open=true;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gateData, sizeof(gateData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  }else{
    SeverDoor.write(10);
  }
}
void CloseDoor(){
  if(GarageDoor){
    gateData.Open=false;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gateData, sizeof(gateData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    } 
  }else{
    SeverDoor.write(170);
  }
}
void poll(){
  TouchPrevious=TouchNow;
  TouchNow=touchRead(TOUCHDOORSWITCH)<THRESHOLD;
  if(TouchNow&&(TouchPrevious^TouchNow)){
    GarageDoor=!GarageDoor;
  }
}

void MovementRiseInterrupt(){
  ws2812.fill(0xFFFF);
  ws2812.show();
  PreviousTime=millis();
}

void fillMonitorMenu(){
  int ii;
  ii=9;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("CONTROL",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);
  
  ii=11;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("LOCK",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);  
}

void fillControlMenu(){
  int ii;
  ii=9;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("MONITOR",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);
  
  ii=11;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("LOCK",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);  
}

void fillPasswordMenu(){
  for(int ii=0;ii<12;ii++){
    tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
    if(ii<9){
      tft.drawNumber(ii+1,  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);
    }else{
      switch(ii){
        case 9:
          tft.drawCentreString("Enter",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);
        break;
        case 10:
          tft.drawCentreString("Delete",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);
        break;
        case 11:
          tft.drawCentreString("Reset",  (Coordinates[ii][0]+Coordinates[ii][2])/2, (Coordinates[ii][1]+Coordinates[ii][3])/2, FONT);  
        break;
        default:
      }
    }
  }
}
bool inBoundingBox(int ah, int aw,int bh,int bw, int x, int y){
  return (ah<=x&&aw<=y&&(ah+bh)>=x&&(aw+bw)>=y);
}

void updateSerial(){
  while (Serial.available()){
    GSMSerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(GSMSerial.available()){
  Serial.write(GSMSerial.read());//Forward what Software Serial received to Serial Port
  }
}

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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&FanData, incomingData, sizeof(FanData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Float: ");
  Serial.println(FanData.Frequency);
  Serial.print("Float: ");
  Serial.println(FanData.Temperature);
}

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
  uint8_t broadcastAddress[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
  {0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};
  esp_now_peer_info_t peerInfo;
  
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);

  typedef struct fasnStruct {
    double Frequency;
    double Temperature;
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
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
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

    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));

    Wire.begin();
    
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
      FanData.Temperature=bme.readTemperature();
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
      FanData.Frequency=1000.0/StartTime;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &FanData, sizeof(FanData));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
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
    //WHAT?S SUPPOSED TO BE HERE HELP ME
  }

  void IRAM_ATTR IRFall(){
    //Just For Shits and Giggles, this will probably go unused
  }
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&FanData, incomingData, sizeof(FanData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("x: ");
    Serial.println(FanData.Frequency);
    //digitalWrite(MOSFETPIN,FanData.Frequency);
  }

  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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