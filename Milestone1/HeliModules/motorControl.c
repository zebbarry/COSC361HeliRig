// *******************************************************
//
// motorControl.c
//
// PI controller module for maintaining helicopter at set
// height by changing duty cycle.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   5.5.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "motorControl.h"

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
void
updateMotors(rotor_t *mainRotor, rotor_t *tailRotor, int16_t altError, int16_t yawError)
{
    int16_t newMainDuty = altError * P_GAIN_MAIN + altInt * I_GAIN_MAIN + HOVER_DUTY_MAIN;
    int16_t newTailDuty = yawError * P_GAIN_TAIL + yawInt * I_GAIN_TAIL + HOVER_DUTY_TAIL;

    // Check duty cycles are within range
    if (newMainDuty > PWM_DUTY_MAX_PER)
    {
        newMainDuty = PWM_DUTY_MAX_PER;
    }
    else if (newMainDuty < PWM_DUTY_MIN_PER)
    {
        newMainDuty = PWM_DUTY_MIN_PER;
    }
    if (newTailDuty > PWM_DUTY_MAX_PER)
    {
        newTailDuty = PWM_DUTY_MAX_PER;
    }
    else if (newTailDuty < PWM_DUTY_MIN_PER)
    {
        newTailDuty = PWM_DUTY_MIN_PER;
    }

    // Check if the error is small enough to hover.
    if (altError < 2 && yawError < 2)
    {
        mainRotor->duty = HOVER_DUTY_MAIN;
        tailRotor->duty = HOVER_DUTY_TAIL;
    } else
    {
        mainRotor->duty = newMainDuty;
        tailRotor->duty = newTailDuty;
    }

    setPWM(mainRotor);
    setPWM(tailRotor);
}

//*****************************************************************************
// Function to integrate yaw and altitude error.
//*****************************************************************************
void
integrate(int16_t altError, int16_t yawError)
{
    altInt += altError;
    yawInt += yawError;
}

//*****************************************************************************
// Function to calculate error.
//*****************************************************************************
int16_t
calcError(int16_t desired, int16_t actual)
{
    return desired - actual;
}
