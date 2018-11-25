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
 * Struct with bits for the switches/bumper sensors
 */
typedef struct _bumpers
{
    bool Left :1;
    bool Right :1;
} Bumpers;

/**
 * Struct for storing distance sensors data
 */
typedef struct _distanceSensors
{
    uint32_t LeftRawValue;
    uint32_t RightRawValue;
    uint8_t Left;
    uint8_t Right;
} DistanceSensor;



void SetUpSensors();


Bumpers ReadBumperSensors();



DistanceSensor ReadDistanceSensors();


/**
 *
 *
 *
 *
 *
 * */

#endif /* OBSTACLESENSOR_H_ */
