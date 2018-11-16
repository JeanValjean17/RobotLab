/*
 * ObstacleSensor.c
 *
 *  Created on: 08.11.2018
 *      Author: JeanCarlosHerrera
 */

#include "ObstacleSensor.H"

void SetUpSensors()
{

    digital_configure_pin(DD_PIN_PC13, DD_CFG_INPUT_PULLUP);

    digital_configure_pin(DD_PIN_PA8, DD_CFG_INPUT_PULLUP);
}

Bumpers ReadBumperSensors()
{
    Bumpers _bumperSensors;

    _bumperSensors.Left = digital_get_pin(DD_PIN_PA8);

    _bumperSensors.Right = digital_get_pin(DD_PIN_PC13);

    return _bumperSensors;
}

DistanceSensor ReadDistanceSensors()
{
    DistanceSensor _distanceSensors;

    _distanceSensors.RightRawValue = adc_get_value(DA_ADC_CHANNEL0);
    _distanceSensors.LeftRawValue = adc_get_value(DA_ADC_CHANNEL2);

    float floatValueRight = (1789 - _distanceSensors.RightRawValue) / 19;

    float floatValueLeft = (1647 - _distanceSensors.LeftRawValue) / 19;

    _distanceSensors.Right = (uint32_t) floatValueRight;
    _distanceSensors.Left = (uint32_t) floatValueLeft;


    return _distanceSensors;
}

