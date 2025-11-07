#include <GyverStepper.h>
//#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "Arduino.h"

GStepper<STEPPER2WIRE> stepX(1280000L, 26, 27, 13); // steps/rev, step, dir, en
GStepper<STEPPER2WIRE> stepY(1280000L, 32, 33, 25);

/* 
 *  X motor axis - W/E (bottom motor)
 *  Y motor axis - N/S (top motor)
 * 12800 steps
 * 1:100 recucer
 * ESP32-WROOM-DA Module
 * Arduino & Events on core 1
 * Stepper tick on core 0
 */

const char* ssid = "iPlanumXZ";
const char* password = "4edbarklox";

#define server_port 4533

#define SerialPort Serial
#define ledPin  2
#define numLeds 8
#define brightness 50

#define XYmode true // XY or AZ/EL rotator

// endstop pin //
#define Xend 34 // elevation / X axis
#define Yend 35 // azimuth / Y axis

// offset for endstops //
#define X_offset -8.5        //  EL/X
#define Y_offset -5          //  AZ/Y

// motor speed in steps/s //
#define X_motor_speed 80000UL  //  EL/X
#define Y_motor_speed 80000UL  //  AZ/Y

// reverse motor //
#define Reverse_X_motor false  //  EL/X
#define Reverse_Y_motor false //  EL/X

#define minEL  -0.5
#define minAZ  -90
#define maxAZ  450

#define Cal_on_start true    // Caibrate motors on startup

#define Test_mode false       // don't return home after calibration
#define Parking   true        // disable motors in parked position
#define Auto_Pwr  false       // enable autoPower

WiFiServer server(server_port);

uint32_t tI;
uint32_t t;
float azSet;
float elSet;

struct AzEl {
  float az;
  float el;
};

struct XY {
  float x;
  float y;
};

TaskHandle_t Task1;

//Adafruit_NeoPixel pix(numLeds, ledPin, NEO_GRB + NEO_KHZ800);

struct XY AE2XY(float azimuth, float elevation){
  struct XY pos;
  azimuth = radians(azimuth);
  elevation = radians(elevation);

  pos.x = degrees(asin(sin(azimuth) * cos(elevation)));
  pos.y = degrees(-atan2(cos(azimuth) * cos(elevation), sin(elevation)));
  
  return pos;
};

struct AzEl XY2AE(float x, float y){
  struct AzEl pos;
  x = radians(x);
  y = radians(y);

  float x0 = sin(x);
  float y0 = -sin(y)*cos(x);
  float z0 = cos(y)*cos(x);
  
  pos.el = degrees(asin(z0));
  pos.az = degrees(- atan2(y0, x0) + HALF_PI);
  
  return pos;
};

void cal(){
  fillStrip(255,0,0); // red, started

  // find X axis endstop //
  stepY.setRunMode(KEEP_SPEED); // set keep speed run mode
  stepY.setSpeedDeg(10);        // run motor until endstop
  while(digitalRead(Xend)==0 and stepY.getCurrentDeg()<=180) {
    stepY.tick();
    if (stepY.getCurrentDeg()>180) while(1){
      calError();
    }
  }

  // return home //
  stepY.setCurrentDeg((90 + Y_offset)); // set current position with offset
  stepY.setRunMode(FOLLOW_POS);       // set follow pos run mode
  stepY.setTargetDeg(0, ABSOLUTE);    // return motor to 0
  while(stepY.tick() and !Test_mode) {stepY.tick();}
  
  fillStrip(255,70,0); // yellow, half complete

  // find Y axis endstop //
  stepX.setRunMode(KEEP_SPEED); // set keep speed run mode
  stepX.setSpeedDeg(10);        // run motor until endstop
  while(digitalRead(Yend)==0 and stepX.getCurrentDeg()<=180) {
    stepX.tick();
    if (stepX.getCurrentDeg()>180) while(1){
        calError();
    }
  }

  // return home  //
  stepX.setCurrentDeg((90 + X_offset)); // set current position with offset
  stepX.setRunMode(FOLLOW_POS);       // set follow pos run mode
  stepX.setTargetDeg(0,ABSOLUTE);     // return motor to 0
  while(stepX.tick() and !Test_mode) {
    stepX.tick();
  }

  fillStrip(0,255,0); // green, completed
}

void fillStrip(int r, int g, int b){
  for(int i=0; i<numLeds; i++) {
//    pix.setPixelColor(i, pix.Color(r, g, b));
//    pix.show();
  }
}

void calError(){
  fillStrip(255,0,0);
  delay(200);
  fillStrip(0,0,0);
  delay(200);
  vTaskDelay(1);
}

void printAzEl(WiFiClient client, float az, float el) {
  client.print(String(az,3)+" "+String(el,3)+" \n");
}

void printAzElSerial(float az, float el) {
  SerialPort.print("AZ");
  SerialPort.print(az, 3);
  SerialPort.print(" EL");
  SerialPort.print(el, 3);
  SerialPort.print("\n");
}

void parseComm(WiFiClient client, String resp) {
  String param;                                
  int ISpace;
  int IISpace;
  float az;
  float el;
  float x, y;
  
  //// parse TCP rotctl ////
  if (resp.startsWith("P") or resp.startsWith("p")) {

    if (resp.indexOf("p") != -1){       //// get pos ////
      y = stepY.getCurrentDeg();
      x = stepX.getCurrentDeg();

      if (XYmode){
        struct AzEl pos = XY2AE(x, y);
        az = pos.az;
        el = pos.el;
      }else{
        az = y;
        el = x;
      }

      printAzEl(client, az, el);

    }else if(resp.indexOf("P") != -1){  //// set pos ////
      ISpace=resp.indexOf(' ');
      IISpace=resp.indexOf(' ',ISpace+1);

      azSet = resp.substring(ISpace, IISpace).toFloat();
      elSet = resp.substring(IISpace+1, resp.length()).toFloat();

      if (azSet > maxAZ){azSet = azSet - 360;}
        
      if (azSet < minAZ){azSet = azSet + 360;}
        
      if (elSet < minEL){elSet = minEL;}
        
      if (XYmode){
        struct XY pos = AE2XY(azSet, elSet);
        azSet = pos.y;
        elSet = pos.x;
      }
      client.print("RPRT 0 \n");
    }
  
  } else if (resp.startsWith("AZ")) {//// parse serial easycomm ////

    if (resp.indexOf("AZ EL")!=-1) {
      y = stepY.getCurrentDeg();
      x = stepX.getCurrentDeg();

      if (XYmode){
        struct AzEl pos = XY2AE(x, y);
        az = pos.az;
        el = pos.el;
      }else{
        az = y;
        el = x;
      }

      printAzElSerial(az, el);                     //Send the current Azimuth and Elevation

    } else if (resp.startsWith("AZ")) {            //Position command received: Parse the line.
      ISpace = resp.indexOf(' ');                  //Get the position of the first space
      IISpace = resp.indexOf(' ', ISpace + 1);     //Get the position of the second space
      param = resp.substring(2, ISpace);           //Get the first parameter
      azSet = param.toFloat();                     //Set the azSet value
      param = resp.substring(ISpace + 3, IISpace); //Get the second parameter
      elSet = param.toFloat();                     //Set the elSet value

      if (XYmode){
        struct XY pos = AE2XY(azSet, elSet);
        azSet = pos.y;
        elSet = pos.x;
      }
    } else if (resp.indexOf("calibration")!=-1) {
      cal();
    }
  }
  
  stepX.setTargetDeg(elSet,ABSOLUTE);
  stepY.setTargetDeg(azSet,ABSOLUTE);

  if (x == 0.0 and y == 0.0 and Parking){
     stepY.disable();
     stepX.disable();
  }else{
     stepY.enable();
     stepX.enable();
  }
}

void Task1code( void * Parameter ){ 
  while(1){
    t = millis();
    while((millis() - t) < 400){//infinite loop
      stepY.tick();
      stepX.tick();
    }
    vTaskDelay(1);
  }
}

void setup() { 
  SerialPort.begin(115200);
  Serial.setTimeout(4);
  pinMode(Xend, INPUT);
  pinMode(Yend, INPUT);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  server.begin();

//  pix.begin();
//  pix.clear();
//  pix.setBrightness(brightness);

  stepY.setAcceleration(320000UL);
  stepX.setAcceleration(320000UL);
  
  stepY.setMaxSpeed(Y_motor_speed);
  stepX.setMaxSpeed(X_motor_speed);

  stepY.reverse(Reverse_Y_motor);
  stepX.reverse(Reverse_X_motor);

  if (Cal_on_start) cal(); // Calibrate
  
  stepY.autoPower(Auto_Pwr);
  stepX.autoPower(Auto_Pwr);

  delay(500); 
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
}

void loop() {
  WiFiClient client = server.available();
  //// TCP handling ////
  if (client) {
    Serial.println("Client connected!");
    while (client.connected()) {
       if (client.available()) {
          String Data = client.readStringUntil('\n');
          parseComm(client, Data);
       }
    }
    client.stop();
  }

  if (Serial.available() and !client.connected()) {
    String Data = Serial.readString();
    parseComm(client, Data);
  }
  
  vTaskDelay(1);
 
  tI = millis()+100;

  while((stepY.tick() or stepX.tick()) and ((millis()+200) - tI) <= 400){
    stepY.tick();
    stepX.tick();
    
    if (Serial.available()) {
      String Data = Serial.readString();
      parseComm(client, Data);
    }
  }
}
