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
  imu.enableDefault();
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
      drawImage(16,16,36);
    case 2:
      delay(500);
      ledGreen(1);
      ledYellow(1);
      ledRed(1);
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
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  int setpoint = init_angle + ((((int32_t)turnAngle90>>16)*360)>>16);
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

/**
 * Turns clockwise by running the motors in opposite direction, for
 * specified angle delta.
 * 
 * Parameters
 * degrees    : positive clockwise angle to turn, in degrees
 * motorSpeed : motor rate to travel at (0-400, unitless)
*/
void turnCW_mag(double degrees, int motorSpeed = 100) {
  setpoint_heading = (setpoint_heading + degrees) % 360.0;
  //void hasOverflow = (setpoint >= 360.0); 

  double kP = 1.0;
  double tolerance = 1.0; // degrees

  // motors.setSpeeds(motorSpeed, -motorSpeed); // activate motors
  bool transit = true;
  while (transit) {
    real_heading = getHeading();
    angle_error = setpoint_heading - real_heading;
    
    if (abs(angle_error) <= tolerance) { transit = false; }

    motor_diff = kP * angle_error;
    motors.setSpeeds(motorSpeed+motor_diff, -motorSpeed-motor_diff);
  }

  motors.setSpeeds(0, 0); // stop motors
}

void turnCCW(double degrees, int motorSpeed = 100) {
  turnSensorReset();
  delay(500);
  turnSensorUpdate();
  int init_angle = ((((int32_t)turnAngle>>16)*360)>>16);
  int setpoint = init_angle - ((((int32_t)turnAngle90>>16)*360)>>16);
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

  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  motors.setSpeeds(motorSpeed,motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    int delta_L = encoders.getCountsLeft(); // ticks travelled
    int delta_R = encoders.getCountsRight();
    int error_L = setpoint - delta_L; // ticks to setpoint (negative if passed)
    int error_R = setpoint - delta_R;
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

  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  motors.setSpeeds(motorSpeed,motorSpeed); // activate motors
  Serial.print(F("\n"));

  bool transit = true;
  while (transit) {
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
  bool leftToRight;
  
  init_heading = getHeading()
  setpoint_heading = init_heading;

  for(int i = 0;i<rows;i++){
    leftToRight = !(i%2);
    paintLine(width_in,image[i],leftToRight);
    if(leftToRight){
      turnCW(90);
      forward((height_in/rows));
      turnCW(90);
    }
    else{
      turnCCW(90);
      forward((height_in/rows));
      turnCCW(90);
    }
  }
}

/*
 * Returns the heading of the Zumo based on the current magnetometer reading.
 * Returns value (in degrees) from 0 to 360
 * 
 * Source:
 * RSE Drive / Projects / Spring 21 / Zumo Videos / Purple Flaming Ducks (Nathan Wolf)
 */
double getHeading() {
  //reads in data from magnetometer and calculates corrected point doing actual - error
  imu.readMag();
  xyzData correctedPoint = {imu.m.x - error.x, imu.m.y - error.y, imu.m.z - error.z};

  double heading = atan2( correctedPoint.x, correctedPoint.y) - PI / 180; //calculates the heading in radians based on the following equation
  //the following if, else if statements shift the heading to an interval of 0 to 2*pi
  if (heading > PI) 
  {
    heading -= (2 * PI);
  }
  else if (heading < -PI) 
  {
    heading += (2 * PI);
  }
  else if (heading < 0) 
  {
    heading += 2 * PI;
  }
  heading *= (180 / PI); //rad to degrees
  heading = 360 - heading; // invert
  int rotateAngle = 90;
  heading = heading + rotateAngle; //rotate to have 0 = north
  if(heading > 360)
  {
    heading -= 360;
  }
  else if(heading < 0)
  {
    heading += 360;
  }
  return heading;  
}