/*
 * IRSensor.h
 *
 *  Created on: 25.11.2018
 *      Author: JeanCarlosHerrera
 */

#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct _irSensors
{
    bool leftValue;
    bool rightValue;
    bool oldLeftValue;
    bool oldRightValue;
    bool leftDetection;
    bool rightDetection;
    uint16_t freq;
} IRSensors;

IRSensors ReadIRSensors();

#endif /* IRSENSOR_H_ */
