/*
 * IRSensor.c
 *
 *  Created on: 25.11.2018
 *      Author: JeanCarlosHerrera
 */

#include "IRSensor.h"
#include "fft.h"
#include "dorobo32.h"


/**
 * Reads digital signal from pins of the IR Sensor as well as the FFT from the PD14 pin. At the end,
 * the information is stored in a IRSensors struct.
 * @return
 */
IRSensors ReadIRSensors()
{
    IRSensors irSensors;

    uint8_t leftSensor = 0;
    uint8_t rightSensor = 0;


    if (ft_is_sampling_finished())
    {
        // Freq only works in one pin. Right now set up to work on PD14 pin.

        uint16_t freq = ft_get_transform(DFT_FREQ100);
        for (uint8_t i = 0; i < 100; i++)
        {
            leftSensor += digital_get_pin(DD_PIN_PD14);
            rightSensor += digital_get_pin(DD_PIN_PC8);
        }

        leftSensor /= 100;
        rightSensor /= 100;

        irSensors.Left = leftSensor;
        irSensors.Right = rightSensor;
        irSensors.freq = freq;

        //tracef(" [Target Sensor] Pin Level : %d,  Freq Read: %d PIN 2   %d\r\n", levelPin, freq, levelPin2);

        //TODO Filter maybe?
        freq > 1500 ? led_red(DD_LEVEL_LOW) : led_red(DD_LEVEL_HIGH);

    }

    return irSensors;
}
