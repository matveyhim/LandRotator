#include <GyverStepper.h>
#include <Adafruit_NeoPixel.h>

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

#define SerialPort Serial
#define ledPin  2
#define numLeds 8
#define brightness 50

// endstop pin //
#define Xend 34 // elevation / X axis
#define Yend 35 // azimuth / Y axis

// offset for endstops //
#define X_offset 5            //  EL/X
#define Y_offset -10          //  AZ/Y

// motor speed in steps/s //
#define X_motor_speed 65000L  //  EL/X
#define Y_motor_speed 65000L  //  AZ/Y

// reverse motor //
#define Reverse_X_motor true  //  EL/X
#define Reverse_Y_motor false //  EL/X

#define Cal_on_start false    // Caibrate motors on startup

#define Test_mode false       // don't return home after calibration
#define Parking   true        // disable motors in parked position
#define Auto_Pwr  false       // enable autoPower

float az;
float el;
String line;
float azSet;
float elSet;
uint32_t tI;
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
  stepY.setCurrentDeg(90 + X_offset); // set current position with offset
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
  stepX.setCurrentDeg(90 + Y_offset); // set current position with offset
  stepX.setRunMode(FOLLOW_POS);       // set follow pos run mode
  stepX.setTargetDeg(0,ABSOLUTE);     // return motor to 0
  while(stepX.tick() and !Test_mode) {
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
    while((millis() - t) < 400){//infinite loop
      stepY.tick();
      stepX.tick();
    }
//    Serial.println("Task!");
    vTaskDelay(1);
  }
}


void setup() { 
  SerialPort.begin(115200);
  Serial.setTimeout(4);
  pinMode(Xend, INPUT);
  pinMode(Yend, INPUT);

  pix.begin();
  pix.clear();
  pix.setBrightness(brightness);

  stepY.setAcceleration(40000L);
  stepX.setAcceleration(40000L);
  
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
  
  if (az==0.0 and el==0.0 and Parking){
    stepY.disable();
    stepX.disable();
  }else{
    stepY.enable();
    stepX.enable();
  }

  stepX.setTargetDeg(elSet,ABSOLUTE);
  stepY.setTargetDeg(azSet,ABSOLUTE);
  
  vTaskDelay(1);
 
  tI = millis()+100;
//  Serial.print((millis()+100) - tI);
//  Serial.print(" loop ");
//  Serial.println(millis());
  while((stepY.tick() or stepX.tick()) and ((millis()+200) - tI) <= 400){
//    Serial.print((millis()+100) - tI);
//    Serial.println(" while");
    
    stepY.tick();
    stepX.tick();
    
    az = stepY.getCurrentDeg();
    el = stepX.getCurrentDeg();
    
    if (Serial.available()) {
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
