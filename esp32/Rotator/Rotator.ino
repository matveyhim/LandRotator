#include <GyverStepper.h>
#include <Adafruit_NeoPixel.h>

GStepper<STEPPER2WIRE> stepX(320000, 26, 27, 13); // steps/rev, step, dir, en
GStepper<STEPPER2WIRE> stepY(320000, 32, 33, 25);

#define SerialPort Serial
#define ledPin  2
#define numLeds 8
#define brightness 50

// endstop pin //
#define Xend 34 // elevation / X axis
#define Yend 35 // azimuth / Y axis

// offset for endstops //
#define X_offset 5   //  EL/X
#define Y_offset -10 //  AZ/Y

#define Cal_on_start false

bool testMode = false;
bool parking = true;
bool autoPwr = false;

float az;
float el;
String line;
float azSet;
float elSet;
bool ifser = false;
uint32_t t;

TaskHandle_t Task1;

Adafruit_NeoPixel pix(numLeds, ledPin, NEO_GRB + NEO_KHZ800);

void printAzEl() {
  SerialPort.print("AZ");
  SerialPort.print(az, 3);
  SerialPort.print(" EL");
  SerialPort.print(el, 3);
  SerialPort.print("\n");
}

void cal(){
  fillStrip(255,0,0); // red, started
  
  stepY.setRunMode(KEEP_SPEED);       // find X axis endstop
  stepY.setSpeedDeg(10);
  while(digitalRead(Xend)==0 and stepY.getCurrentDeg()<=180) {
    stepY.tick();
    if (stepY.getCurrentDeg()>180) while(1){
      calError();
    }
  }

  stepY.setCurrentDeg(90 + X_offset);            // return home
  stepY.setRunMode(FOLLOW_POS);
  stepY.setTargetDeg(0, ABSOLUTE);
  while(stepY.tick() and !testMode) {stepY.tick();}
  
  fillStrip(255,70,0); // yellow, half complete
  
  stepX.setRunMode(KEEP_SPEED);      // find Y axis endstop
  stepX.setSpeedDeg(10);
  while(digitalRead(Yend)==0 and stepX.getCurrentDeg()<=180) {
    stepX.tick();
    if (stepX.getCurrentDeg()>180) while(1){
        calError();
    }
  }

  stepX.setCurrentDeg(90 + Y_offset);          // return home
  stepX.setRunMode(FOLLOW_POS);
  stepX.setTargetDeg(0,ABSOLUTE);
  while(stepX.tick() and !testMode) {
    stepX.tick();
  }

  fillStrip(0,255,0); // green, completed
}

void calError(){
  fillStrip(255,0,0);
  delay(200);
  fillStrip(0,0,0);
  delay(200);
  vTaskDelay(1);
}

void fillStrip(int r, int g, int b){
  for(int i=0; i<numLeds; i++) {
    pix.setPixelColor(i, pix.Color(r, g, b));
    pix.show();
  }
}

void Task1code( void * Parameter ){ 
  while(1){
    t = millis();
//    Serial.println(millis() - t);
    while(ifser){//infinite loop
      if ((millis() - t) > 400) ifser = false;
      stepY.tick();
      stepX.tick();
    }
      delay(1);
  }
}


void setup() {
//  pinMode(led, OUTPUT);
  
  SerialPort.begin(115200);
  Serial.setTimeout(4);
  pinMode(Xend, INPUT);
  pinMode(Yend, INPUT);

  pix.begin();
  pix.clear();
  pix.setBrightness(brightness);

  stepY.setAcceleration(9000);
  stepX.setAcceleration(9000);
  stepY.setMaxSpeed(8000);    //1500
  stepX.setMaxSpeed(8000);    //1000

  if (Cal_on_start) cal(); // Calibrate
  
  stepY.autoPower(autoPwr);
  stepX.autoPower(autoPwr);

  delay(500); 
 xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 1);
}

void loop() {
  az = stepY.getCurrentDeg();
  el = stepX.getCurrentDeg();
  
  if (Serial.available()) {
    line = Serial.readString();
    String param;                                          
    int firstSpace;                                         
    int secondSpace;                                        
    
    if (line.indexOf("AZ EL")!=-1) {                        
      printAzEl();                                         
    } else if (line.startsWith("AZ")) {
      firstSpace = line.indexOf(' ');
      secondSpace = line.indexOf(' ', firstSpace + 1);
      param = line.substring(2, firstSpace);
      azSet = param.toFloat();
      param = line.substring(firstSpace + 3, secondSpace);
      elSet = param.toFloat();
    } else if (line.indexOf("calibration")!=-1) {
      cal();
    }
  }
  
  if (azSet==0.0 and elSet==0.0 and parking){
    stepY.disable();
    stepX.disable();
  }else{
    stepY.enable();
    stepX.enable();
  }

  stepX.setTargetDeg(elSet,ABSOLUTE);
  stepY.setTargetDeg(azSet,ABSOLUTE);

  vTaskDelay(1);
  while(stepY.tick() or stepX.tick()){
    vTaskDelay(1);
    ifser = true;
    stepY.tick();
    stepX.tick();
    az = stepY.getCurrentDeg();
    el = stepX.getCurrentDeg();
    
    if (Serial.available()) {
//      Serial.println("fser = true ");
      ifser = true;
      line = Serial.readString();
      String param;                                           //Parameter value
      int firstSpace;                                         //Position of the first space in the command line
      int secondSpace;                                        //Position of the second space in the command line
      
      if (line.indexOf("AZ EL")!=-1) {                         //Query command received
        printAzEl();                                          //Send the current Azimuth and Elevation
      } else if (line.startsWith("AZ")) {                          //Position command received: Parse the line.
        firstSpace = line.indexOf(' ');                     //Get the position of the first space
        secondSpace = line.indexOf(' ', firstSpace + 1);    //Get the position of the second space
        param = line.substring(2, firstSpace);              //Get the first parameter
        azSet = param.toFloat();                            //Set the azSet value
        param = line.substring(firstSpace + 3, secondSpace);//Get the second parameter
        elSet = param.toFloat();                            //Set the elSet value
      } else if (line.indexOf("calibration")!=-1) {
        cal();
      }
    }
  }
}
