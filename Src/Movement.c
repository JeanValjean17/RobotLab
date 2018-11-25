/*
 * Movement.c
 *
 *  Created on: 14.11.2018
 *      Author: JeanCarlosHerrera
 */

#include "Movement.h"

#define MAX_VEL 40
#define MAX_ROT_VEL 20
#define RATE_OF_SPEEDUP 20
#define ROT_VEL 20

static MovementControl robotMovement;


/**
 * Initialises struct which contains movement and direction order
 */
void InitMotorControl()
{
    robotMovement.direction = Mov_Straight;
    robotMovement.duration = 1;
}

/**
 *
 * @param velocity
 * @return velocity after movement
 */
int8_t MoveStraight(int8_t vel)
{
    motor_set(DM_MOTOR0, vel * -1);
    motor_set(DM_MOTOR1, vel);
    motor_set(DM_MOTOR3, 0);

    if (vel >= MAX_VEL)
        vel = MAX_VEL;
    else
        vel += RATE_OF_SPEEDUP;

    return vel;
}

/**
 *
 * @param velocity
 * @return velocity after movement
 */
int8_t MoveBack(int8_t vel)
{

    static bool switchDirection = false;

    if (!switchDirection)
    {
        motor_set(DM_MOTOR0, vel);
        motor_set(DM_MOTOR1, vel * -1);
        motor_set(DM_MOTOR3, 0);
        switchDirection = true;
    }
    else
    {
        motor_set(DM_MOTOR0, ROT_VEL);
        motor_set(DM_MOTOR1, ROT_VEL);
        motor_set(DM_MOTOR3, -ROT_VEL);
        switchDirection = false;
    }

    if (vel >= MAX_VEL)
        vel = MAX_VEL;
    else
        vel += RATE_OF_SPEEDUP;

    return vel;
}

/**
 * Moves the robot back and then turns to the right.
 * @param vel velocity of motors.
 * @return
 */
int8_t BackAndRight(int8_t vel)
{
    static bool switchDirection = false;

    if (!switchDirection)
    {
        motor_set(DM_MOTOR0, vel);
        motor_set(DM_MOTOR1, vel * -1);
        motor_set(DM_MOTOR3, 0);
        switchDirection = true;
    }
    else
    {
        motor_set(DM_MOTOR0, -ROT_VEL);
        motor_set(DM_MOTOR1, -ROT_VEL);
        motor_set(DM_MOTOR3, ROT_VEL);
        switchDirection = false;
    }

    if (vel >= MAX_VEL)
        vel = MAX_VEL;
    else
        vel += RATE_OF_SPEEDUP;

    return vel;
}

/**
 *
 * @param velocity
 * @return velocity after movement
 */
int8_t RotateLeft(int8_t vel)
{
    int8_t velRot = 20;

    motor_set(DM_MOTOR0, ROT_VEL);
    motor_set(DM_MOTOR1, ROT_VEL);
    motor_set(DM_MOTOR3, -ROT_VEL);

    if (vel >= MAX_ROT_VEL)
        vel = MAX_ROT_VEL;
    else
        vel += RATE_OF_SPEEDUP;

    return vel;
}

/**
 *
 * @param velocity
 * @return velocity after movement
 */
int8_t RotateRight(int8_t vel)
{

    motor_set(DM_MOTOR0, -ROT_VEL);
    motor_set(DM_MOTOR1, -ROT_VEL);
    motor_set(DM_MOTOR3, ROT_VEL);

    if (vel >= MAX_ROT_VEL)
        vel = MAX_ROT_VEL;
    else
        vel += RATE_OF_SPEEDUP;

    return vel;
}


/**
 *
 * @return velocity after movement
 */
int8_t StopMovement()
{
    motor_set(DM_MOTOR0, 0);
    motor_set(DM_MOTOR1, 0);
    motor_set(DM_MOTOR3, 0);

    return 0;
}

/**
 * Writes movement order
 * @param direction direction movement of the robot
 * @param duration how many consecutive times this movement will be executed
 * @return true if movement correctly written false if there's still a movement in execution. Ex: duration still > 0
 */
bool WriteMovement(enum RobotMovement direction, uint8_t duration)
{

    // validate if there's still movement being executed
    if (robotMovement.duration > 1)
    {
        return 0;
    }

    robotMovement.direction = direction;
    robotMovement.duration = duration;

    return 1;
}

/**
 * Reads current movement order
 * @return
 */
MovementControl ReadMovementToExecute()
{
    --robotMovement.duration;
    return robotMovement;
}
