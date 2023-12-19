#ifndef SENSORS_H
#define SENSORS_H
//libreria HX711
#include "HX711.h"
#include <Arduino.h>
 
//load cell 1
#define DOUT_PIN 20
#define SCK_PIN 21

void sensors_setup(void);
float read_load_cell(void);

#endif