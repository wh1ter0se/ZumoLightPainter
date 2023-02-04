#include <Wire.h>
#include <Zumo32U4.h>

#define getXY 0
#define printing 1
#define printDone 2

Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

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
