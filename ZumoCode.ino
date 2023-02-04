#include <Wire.h>
#include <Zumo32U4.h>
#include <cmath>

#define getXY 0
#define printing 1
#define printDone 2

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
int array[]

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
      }
      break;
    case 1:
      delay(1000);
  }
}

// TODO finish, once the image and paintLine() are ready
void drawImage(double width_in, double height_in, int rows) {
  double row_height_in = height_in / rows;
  for (int i = 0; i < rows; i++) {
    paintLine(width_in);
    turnCW(90);
    forward(row_height_in);
    turnCW(90);
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
  turnSensorUpdate();
  double init_angle = turnAngle;
  double setpoint = init_angle + (turnAngle1*degrees);

  motors.setSpeeds(motorSpeed, -motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    turnSensorUpdate();
    double error = setpoint - turnAngle;
    if (error <= 0) { transit = false; }
  }

  motors.setSpeeds(0, 0); // stop motors
}

/**
 * Moves forward specified distance at constant, specified motor speed. 
 * Moves until at least one encoder hits its setpoint.
 * 
 * Paramaters
 * inches     : positive distance to travel in inches (max 120in)
 * motorSpeed : motor rate to travel at (0-400, unitless)
*/
void forward(double inches, int motorSpeed = 150) {
  double setpoint = ticks_per_in * inches;

  encoders.getCountsAndResetLeft(); // set counts to 0 (overflows at 32767 ticks, >10ft)
  encoders.getCountsAndResetRight();
  
  motors.setSpeeds(motorSpeed); // activate motors

  bool transit = true;
  while (transit) {
    int delta_L = encoders.getCountsLeft(); // ticks travelled
    int delta_R = encoders.getCountsRight();
    int error_L = setpoint - delta_L; // ticks to setpoint (negative if passed)
    int error_R = setpoint - delta_R;
    if ((error_L<=0) || (error_R<=0)) { transit = false; }
  }

  motors.setSpeeds(0); // stop motors
}

void paintLine(double inches);