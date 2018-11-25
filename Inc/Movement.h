/*
 * Movement.h
 *
 *  Created on: 14.11.2018
 *      Author: JeanCarlosHerrera
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "motor.h"
#include <stdbool.h>

/**
 * Velocity of motors
 */
extern int8_t mov_velocity;

/**
 * Enum for robot movement.
 */
enum RobotMovement
{
    Mov_Straight = 4, //!< Mov_Straight
    Mov_Back = 1,     //!< Mov_Back
    Mov_Rot_Left = 2, //!< Mov_Rot_Left
    Mov_Rot_Right = 3, //!< Mov_Rot_Right
    Mov_Stop = 0,      //!< Mov_Stop
    Mov_BackRight = 5
};

/**
 * Struct that contains the direction the robot should take and the duration of the movement.
 */
typedef struct _motorControl
{
    uint8_t duration;
    enum RobotMovement direction;
    enum RobotMovement nextDirection;
} MovementControl;

#endif /* MOVEMENT_H_ */


void InitMotorControl();


int8_t BackAndRight(int8_t vel);


int8_t MoveStraight(int8_t velocity);


int8_t MoveBack(int8_t velocity);


int8_t RotateLeft(int8_t velocity);


int8_t RotateRight(int8_t velocity);

/**
 *
 * @param velocity
 * @return
 */
int8_t BackAndRight(int8_t vel);

bool WriteMovement(enum RobotMovement direction, uint8_t duration);


MovementControl ReadMovementToExecute();

int8_t StopMovement();

