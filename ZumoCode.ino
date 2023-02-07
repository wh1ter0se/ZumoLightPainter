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

// Menu vars
uint8_t x;
uint8_t y;
uint8_t state;

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
  delay(500);
  turnSensorReset();

  // put your setup code here, to run once:
  x = 0;
  y = 0;
  state = 0;
  printStuff();
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
        delay(1000);
        paintLine(36,image[30]);
      }
      break;
    case 1:
      delay(1000);
  }
}


/**
 * Turns clockwise by running the motors in opposite direction, for
 * specified angle delta. Uses the TurnSensor.h file.
 * Code used from:
 * https://github.com/pvcraven/zumo_32u4_examples/blob/master/TurnExample/TurnExample.ino
 * 
 * Parameters
 * degrees    : positive clockwise angle to turn, in degrees
 * motorSpeed : motor rate to travel at (0-400, unitless)
*/
void turnCW(double degrees, int motorSpeed = 100) {
  turnSensorReset();
  turnSensorUpdate();
  // double init_angle = turnAngle;
  // double setpoint = init_angle + (turnAngle1*degrees);
  double setpoint = degrees;

  motors.setSpeeds(motorSpeed, -motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    double angle = double(((int32_t)turnAngle >> 16) * 360) >> 16;
    double error = setpoint - angle;
    if (error <= 0) { transit = false; }
  }

  motors.setSpeeds(0, 0); // stop motors
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
void paintLine(double inches, char pixels[54], bool leftToRight = true, int motorSpeed = 150) {
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

    // LED code
    double delta_avg = (delta_L + delta_R) / 2;
    double progress = delta_avg / setpoint;
    if (!leftToRight) { progress = 1 - progress; } // invert if zagging, otherwise leave as a zig
    int closestPixel_indx = int(round(progress*54));
    char closestPixel = pixels[closestPixel_indx];
    display.clear();
    display.print(closestPixel);
    // if (leftToRight) { closestPixel = pixels[closestPixel_indx]; } // left for debugging, but not meant to be included as code
    // else             { closestPixel = pixels[px_x-closestPixel_indx-1]; }
    switch (closestPixel) {
      case 'W': // white -> no LED
        ledRed(0);
        ledYellow(0);
        ledGreen(0);
      case 'Y': // yellow -> yellow LED
        ledRed(0);
        ledYellow(1);
        ledGreen(0);
      case 'B': // black -> red LED
        ledRed(1);
        ledYellow(0);
        ledGreen(0);
      default: // unparsable char -> no LED
        ledRed(0);
        ledYellow(0);
        ledGreen(0);
    }

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
  double row_height_in = height_in / rows;
  
  for (int i = 0; i < rows; i++) {
    int image_row_indx = int(round((i)*(36/rows))); // pick the closest pixel row

    char image_row[54]; // extract the row
    for (int j = 0; j < 54; j++) {
      image_row[j] = image[image_row_indx][j];
    }

    paintLine(width_in, image_row, (i%2)==0);
    turnCW(90);
    forward(row_height_in);
    turnCW(90);
  }
}