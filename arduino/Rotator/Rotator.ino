#include <GyverStepper.h>

GStepper<STEPPER2WIRE> stepX(32000L, 2, 5, 8); // AZ steps/rev, step, dir, en
GStepper<STEPPER2WIRE> stepY(16000L, 3, 6, 8); // EL

#define SerialPort Serial

#define EnableEndstops false

// endstop pin //
#define Xend 34 // azimuth / X axis
#define Yend 35 // elevation / Y axis

// offset for endstops //
#define X_offset -8.5          //  AZ/X
#define Y_offset -5          //  EL/Y

// motor speed in steps/s //
#define X_motor_speed 5000UL  //  AZ/X
#define Y_motor_speed 5000UL  //  EL/Y

// reverse motor //
#define Reverse_X_motor false  // AZ/X
#define Reverse_Y_motor true //  EL/Y

#define Cal_on_start false    // Caibrate motors on startup

#define Test_mode false       // don't return home after calibration
#define Parking   true        // disable motors in parked position
#define Auto_Pwr  false       // enable autoPower

float az;
float el;
String line;
float azSet;
float elSet;

void printAzEl(float az, float el) {
  SerialPort.print("AZ");
  SerialPort.print(az, 3);
  SerialPort.print(" EL");
  SerialPort.print(el, 3);
  SerialPort.print("\n");
}


void parseComm(String resp) {
  String param;                                
  int ISpace;
  int IISpace;
  float az;
  float el;
  float x, y;
  if (resp.indexOf("AZ EL")!=-1) {
    y = stepY.getCurrentDeg();
    x = stepX.getCurrentDeg();

    printAzEl(x, y);                     //Send the current Azimuth and Elevation

  } else if (resp.startsWith("AZ")) {            //Position command received: Parse the line.
    ISpace = resp.indexOf(' ');                  //Get the position of the first space
    IISpace = resp.indexOf(' ', ISpace + 1);     //Get the position of the second space
    param = resp.substring(2, ISpace);           //Get the first parameter
    azSet = param.toFloat();                     //Set the azSet value
    param = resp.substring(ISpace + 3, IISpace); //Get the second parameter
    elSet = param.toFloat();                     //Set the elSet value

  } else if (resp.indexOf("calibration")!=-1) {
    cal();
  }
  
  stepX.setTargetDeg(azSet,ABSOLUTE);
  stepY.setTargetDeg(elSet,ABSOLUTE);

  if (x == 0.0 and y == 0.0 and Parking){
     stepY.disable();
     stepX.disable();
  }else{
     stepY.enable();
     stepX.enable();
  }
}


void cal(){
  if (EnableEndstops){
    // find X axis endstop //
    stepX.setRunMode(KEEP_SPEED); // set keep speed run mode
    stepX.setSpeedDeg(10);        // run motor until endstop
    while(digitalRead(Xend)==0 and stepX.getCurrentDeg()<=180) {
      stepX.tick();
      if (stepX.getCurrentDeg()>180) while(1){
        calError();
      }
    }
  
    // return home //
    stepX.setCurrentDeg((90 + X_offset)); // set current position with offset
    stepX.setRunMode(FOLLOW_POS);       // set follow pos run mode
    stepX.setTargetDeg(0, ABSOLUTE);    // return motor to 0
    while(stepX.tick() and !Test_mode) {stepX.tick();}
  
    // find Y axis endstop //
    stepY.setRunMode(KEEP_SPEED); // set keep speed run mode
    stepY.setSpeedDeg(10);        // run motor until endstop
    while(digitalRead(Yend)==0 and stepY.getCurrentDeg()<=180) {
      stepY.tick();
      if (stepY.getCurrentDeg()>180) while(1){
          calError();
      }
    }
  
    // return home  //
    stepY.setCurrentDeg((90 + Y_offset)); // set current position with offset
    stepY.setRunMode(FOLLOW_POS);       // set follow pos run mode
    stepY.setTargetDeg(0,ABSOLUTE);     // return motor to 0
    while(stepY.tick() and !Test_mode) {
      stepY.tick();
    }
  }
}

void calError(){
//  fillStrip(255,0,0);
//  delay(200);
//  fillStrip(0,0,0);
  delay(200);
//  vTaskDelay(1);
}

void setup() { 
  SerialPort.begin(115200);
  Serial.setTimeout(4);
  if (EnableEndstops){
    pinMode(Xend, INPUT);
    pinMode(Yend, INPUT);
  }
  
  stepY.setAcceleration(3200UL);
  stepX.setAcceleration(3200UL);
  
  stepY.setMaxSpeed(Y_motor_speed);
  stepX.setMaxSpeed(X_motor_speed);

  stepY.reverse(Reverse_Y_motor);
  stepX.reverse(Reverse_X_motor);

  if (Cal_on_start) cal(); // Calibrate
  
  stepY.autoPower(Auto_Pwr);
  stepX.autoPower(Auto_Pwr);

  delay(500); 
}

void loop() {
if (Serial.available()) {
    String Data = Serial.readString();
    parseComm(Data);
  }

  while(stepY.tick() or stepX.tick()){
    stepY.tick();
    stepX.tick();
    
    if (Serial.available()) {
      String Data = Serial.readString();
      parseComm(Data);
    }
  }
}
