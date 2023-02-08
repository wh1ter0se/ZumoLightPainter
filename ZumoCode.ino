#include <Wire.h>
#include <Zumo32U4.h>

#define getXY 0
#define printing 1
#define printDone 2

// Zumo objects
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4LCD display;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

typedef struct{
  int x;
  int y;
  int z;
} xyzData;

xyzData errorGlobal;

#include "TurnSensor.h" // leave at this location- it needs Zumo objects

// Encoder constants
const double tread_diam_in = 1.53543; // total height of treads, verified in CAD
const double ticks_per_rev = 12 * 75.81; // this may be 51.45, 75.81, or 100.37
const double in_per_rev = M_PI * tread_diam_in; // circumference = 2*pi*r = pi*D
const double ticks_per_in = ticks_per_rev / in_per_rev;

// Compass heading
double init_heading;
double setpoint_heading;

// Menu vars
uint8_t x;
uint8_t y;
uint8_t state;

//Prototypes
void turnCW(double degrees, int motorSpeed = 100);
void turnCCW(double degrees, int motorSpeed = 100);
void paintLine(double inches, char pixels[54], bool leftToRight = true, int motorSpeed = 150);
void drawImage(double width_in, double height_in, int rows);
void forward(double inches, int motorSpeeds = 150);
void backward(double inches, int motorSpeeds = 150);
void turnCW_mag(double degrees, int motorSpeed = 100);
double getHeading();

// Image
char image[36][54] = {
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','W','W','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','W','W','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','B','B','W','W','W','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','B','B','B','B','W','W','W','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','B','B','B','Y','Y','Y','Y','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','Y','Y','Y','Y','Y','B','B','B','B','B','W','W','W','W','B','B','B','Y','Y','Y','Y','Y','Y','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','B','B','B','B','Y','Y','Y','Y','Y','B','B','B','B','B','B','B','B','B','W','W','B','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','B','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','B','B','B','B','B','B','B','B','B','B','B','W','W','W','B','B','B','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','B','B','Y','Y','Y','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','B','B','Y','Y','Y','B','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','W','W'},
  {'W','W','W','W','W','B','B','Y','Y','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','B','B','W','W','W','W','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','W'},
  {'W','W','W','W','W','B','B','Y','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','B','B','W','W','W','W','B','Y','Y','Y','Y','Y','Y','Y','Y','Y','B','B','W','W','W','W','W'},
  {'W','W','W','W','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','B','B','B','W','W','W','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W'},
  {'W','W','W','W','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','W','W','W','W'},
  {'W','W','W','B','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W'},
  {'W','W','W','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','B','B','B','W','W','W'},
  {'W','W','W','W','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','B','W','W','W','W','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','Y','Y','B','B','W','W','W','W'},
  {'W','W','W','W','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','Y','B','B','B','W','W','W','W'},
  {'W','W','W','W','W','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','Y','Y','B','B','W','W','W','W','W'},
  {'W','W','W','W','W','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','Y','Y','B','B','B','W','W','W','W','W'},
  {'W','W','W','W','W','W','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','B','B','B','W','W','W','B','B','B','B','B','B','B','B','W','W','W','W','W','B','Y','Y','B','B','B','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','B','Y','Y','B','B','B','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','B','B','B','B','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','B','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','B','B','B','W','W','W','W','W','W','W','B','B','B','Y','Y','Y','Y','Y','Y','Y','Y','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','B','B','W','W','W','W','W','W','B','B','B','B','Y','Y','Y','Y','Y','Y','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','B','B','W','W','W','W','W','B','B','B','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','B','W','W','W','W','B','B','B','B','B','B','B','B','B','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'},
  {'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W'}
};

/*
 * Calibrates the magnetometer to account for hard iron distortions.
 * Finds the min and max for x,y,z and calculates the error/center point of the 
 * data and stores it in errorGlobal.
 * 
 * Pass true as printData argument for method to print debugging info.
 */


void printStuff() { //Prints the menu, saves a little space to have it as a function
  display.clear();
  display.print(F("X:"));
  display.print(x);
  display.print(F(" Y:"));
  display.print(y);
  display.gotoXY(0,1);
  display.print(F("X  Y  OK"));
}


void setup() {
  // DO NOT REMOVE; needed for gyro/turning
  turnSensorSetup(); 
  turnSensorReset();
  Serial.begin(9600);

  // put your setup code here, to run once:
  x = 0;
  y = 0;
  state = 0;
  printStuff();
  Wire.begin();
}


void loop() {
  // put your main code here, to run repeatedly:
  switch (state){
    case 0:
      if(buttonA.getSingleDebouncedPress()){
        x++;
        printStuff();
      }
      if(buttonB.getSingleDebouncedPress()){
        y++;
        printStuff();
      }
      if(buttonC.getSingleDebouncedPress()){
        state++;
        display.clear();
        display.print(F("yay"));
      }
      break;
    case 1:
      delay(1000);
      drawImage(36,24,36);
      state++;
    case 2:
      delay(500);
      break;
  }
}


/**
 * Turns clockwise by running the motors in opposite direction, for
 * specified angle delta.
 * 
 * Parameters
 * degrees    : positive clockwise angle to turn, in degrees
 * motorSpeed : motor rate to travel at (0-400, unitless)
*/
void turnCW(double degrees, int motorSpeed = 100) {
  ledGreen(0);
  ledYellow(0);
  ledRed(0);
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  int setpoint = init_angle + ((((int32_t)(turnAngle1*degrees)>>16)*360)>>16);
  display.clear();
  display.print(F("CW Turn"));

  motors.setSpeeds(motorSpeed, -motorSpeed); // activate motors
  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    int error = ((((int32_t)turnAngle>>16)*360)>>16) - setpoint;
    if (error <= -180) { transit = false; }
  }

  motors.setSpeeds(0, 0); // stop motors
}

void turnCCW(double degrees, int motorSpeed = 100) {
  ledGreen(0);
  ledYellow(0);
  ledRed(0);
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  int setpoint = init_angle - ((((int32_t)(turnAngle1*degrees)>>16)*360)>>16);
  display.clear();
  display.print(F("CCW Turn"));

  motors.setSpeeds(-motorSpeed, motorSpeed);
  bool transit = true;
  while (transit){
    turnSensorUpdate();
    int error = setpoint + ((((int32_t)turnAngle>>16)*360)>>16);
    if(error>=0) { transit = false; }
  }
  motors.setSpeeds(0,0);
}


/**
 * Moves forward specified distance at constant, specified motor speed. 
 * Moves until at least one encoder hits its setpoint.
 * 
 * Parameters
 * inches     : positive distance to travel in inches (max 120in)
 * motorSpeed : motor rate to travel at (0-400, unitless)
*/
void forward(double inches, int motorSpeed = 150) {
  double setpoint = ticks_per_in * inches;
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  motors.setSpeeds(motorSpeed,motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    int errorAngle =  ((((int32_t)turnAngle>>16)*360)>>16);
    Serial.print(errorAngle);
    Serial.print(F("\n"));
    motors.setSpeeds((motorSpeed+errorAngle),(motorSpeed-errorAngle));
    int delta_L = encoders.getCountsLeft(); // ticks travelled
    int delta_R = encoders.getCountsRight();
    int error_L = setpoint - delta_L; // ticks to setpoint (negative if passed)
    int error_R = setpoint - delta_R;
    if ((error_L<=0) || (error_R<=0)) { transit = false; }
  }

  motors.setSpeeds(0,0); // stop motors
}

void backward(double inches, int motorSpeed = 150) {
  ledGreen(0);
  ledYellow(0);
  ledRed(0);
  double setpoint = ticks_per_in * inches;
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  motors.setSpeeds(-motorSpeed,-motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    int delta_L = -(encoders.getCountsLeft()); // ticks travelled
    int delta_R = -(encoders.getCountsRight());
    int error_L = (setpoint - delta_L); // ticks to setpoint (negative if passed)
    int error_R = (setpoint - delta_R);
    if ((error_L<=0) || (error_R<=0)) { transit = false; }
  }

  motors.setSpeeds(0,0); // stop motors
}


/**
 * Drives forward, while setting the LEDs to values that correspond
 * to the pixel colors given.
 * 
 * Parameters
 * inches : distance to drive forward and draw row across
 * pixels : row of 56 (px_x) characters representing color, pulled from image
*/
void paintLine(double inches, char pixels[54], bool leftToRight, int motorSpeed = 150) {
  double setpoint = ticks_per_in * inches;

  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  int setpoint2 = init_angle - ((((int32_t)turnAngle90>>16)*360)>>16);

  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  if(leftToRight)
    motors.setSpeeds(motorSpeed,motorSpeed); // activate motors
  else
    motors.setSpeeds(-motorSpeed,-motorSpeed);
  Serial.print(F("\n"));

  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    int errorAngle = ((((int32_t)turnAngle>>16)*360)>>16);
    int delta_L = encoders.getCountsLeft(); // ticks travelled
    int delta_R = encoders.getCountsRight();
    int error_L = setpoint - delta_L; // ticks to setpoint (negative if passed)
    int error_R = setpoint - delta_R;

    // LED code
    double delta_avg = (delta_L + delta_R) / 2;
    double progress = delta_avg / setpoint;
    int closestPixel_indx;
    if (!leftToRight) { closestPixel_indx = 53-int(round(progress*54)); } // invert if zagging, otherwise leave as a zig
    else{ closestPixel_indx = int(round(progress*54)); }
    char closestPixel = pixels[closestPixel_indx];
    // if (leftToRight) { closestPixel = pixels[closestPixel_indx]; } // left for debugging, but not meant to be included as code
    // else             { closestPixel = pixels[px_x-closestPixel_indx-1]; }
    if (closestPixel == 'W') {
      // white -> no LED
        ledRed(0);
        ledYellow(0);
        ledGreen(1);
    }
    else if(closestPixel == 'Y'){

      ledRed(0);
      ledYellow(1);
      ledGreen(0);
    } // yellow -> yellow LED
    else if(closestPixel == 'B'){
      ledRed(1);
      ledYellow(0);
      ledGreen(0);
    }  // black -> red LED
      
    else{

      ledRed(0);
      ledYellow(0);
      ledGreen(0);
    } // unparsable char -> no LED

    if ((error_L<=0) || (error_R<=0)) { transit = false; }
  }

  motors.setSpeeds(0,0); // stop motors
}


/**
 * Main function to call; runs in a zigzag, drawing rows of the
 * image using the topside LEDs.
 * 
 * Parameters
 * width_in  : x-axis length to drive, in inches
 * height_in : y-axis length, summing height of all rows, in inches
 * rows      : number of rows to zigzag
*/
void drawImage(double width_in, double height_in, int rows) {
  bool leftToRight = true;
  for(int i = 0;i<rows;i++){
    paintLine(width_in,image[i],leftToRight);
    backward(36);

  }
}
