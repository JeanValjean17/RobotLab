/*
 * IRSensor.h
 *
 *  Created on: 25.11.2018
 *      Author: JeanCarlosHerrera
 */

#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <stdint.h>

typedef struct _irSensors
{
    uint8_t Left;
    uint8_t Right;
    uint16_t freq;
} IRSensors;


IRSensors ReadIRSensors();


#endif /* IRSENSOR_H_ */
