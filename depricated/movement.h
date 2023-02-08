#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Wire.h>
#include <Zumo32U4.h>

#include <stdio.h>
#include <stdlib.h>

void turnCW(double degrees);
void turnCCW(double degrees);

void forward(double inches);
void backward(double inches);

void paintLine(double inches, );