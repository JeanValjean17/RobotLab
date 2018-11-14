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

    for (uint8_t i = 0; i < 10; i++)
    {
        _distanceSensors.LeftRawValue += adc_get_value(DA_ADC_CHANNEL0);
        _distanceSensors.RightRawValue += adc_get_value(DA_ADC_CHANNEL2);
    }

    //TODO: Add equation for distance. Re-calibration needed.

    return _distanceSensors;
}

