/*
 * ObstacleSensor.H
 *
 *  Created on: 08.11.2018
 *      Author: JeanCarlosHerrera
 */

#ifndef OBSTACLESENSOR_H_
#define OBSTACLESENSOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "digital.h"
#include "adc.h"

/**
 *
 */
typedef struct _bumpers
{
    bool Left :1;
    bool Right :1;
} Bumpers;

/**
 *
 */
typedef struct _distanceSensors
{
    uint32_t LeftRawValue;
    uint32_t RightRawValue;
    uint8_t Left;
    uint8_t Right;
} DistanceSensor;


/**
 * Configures pins for sensors
 */
void SetUpSensors();

/**
 * Reads the bumpers sensors
 * @return
 */
Bumpers ReadBumperSensors();


/**
 * Obtains the distance and the raw value from the sharp sensors
 * @return
 */
DistanceSensor ReadDistanceSensors();


/**
 *
 *
 *
 *
 *
 * */

#endif /* OBSTACLESENSOR_H_ */
