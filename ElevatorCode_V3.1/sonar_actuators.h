#ifndef SONAR_ACTUATORS_H
#define SONAR_ACTUATORS_H

#include <Arduino.h>

int objectDetect (void); //returns 0 if no object, 1 if top, 2 if bottom
void topActLoad (float fraction); //run top actuator; receive turning distance
void botActLoad (float fraction); //run bottom actuator; receive turning distance

#endif
