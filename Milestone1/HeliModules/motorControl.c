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
updateMotors(rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    int32_t newMainDuty = (2*altError + P_GAIN_MAIN) / 2 / P_GAIN_MAIN + HOVER_DUTY_MAIN;
    //newMainDuty += (2*altErrorInt + I_GAIN_MAIN) / 2 / I_GAIN_MAIN;

    int32_t newTailDuty = (2*yawError + P_GAIN_TAIL) / 2 / P_GAIN_TAIL + HOVER_DUTY_TAIL;
    //newTailDuty += (2*yawErrorInt + I_GAIN_TAIL) / 2 / I_GAIN_TAIL;

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
    /*if (altError < 2 && yawError < 2)
    {
        mainRotor->duty = HOVER_DUTY_MAIN;
        tailRotor->duty = HOVER_DUTY_TAIL;
    } else
    {
        mainRotor->duty = newMainDuty;
        tailRotor->duty = newTailDuty;
    }*/

    mainRotor->duty = newMainDuty;
    tailRotor->duty = newTailDuty;
    setPWM(mainRotor);
    setPWM(tailRotor);
}

//*****************************************************************************
// Function to integrate yaw and altitude error.
//*****************************************************************************
void
integrate(int32_t altError, int32_t yawError)
{
    altErrorInt += altError;
    yawErrorInt += yawError;
}

//*****************************************************************************
// Function to calculate altitude error.
//*****************************************************************************
int32_t
calcAltError(int32_t desiredAlt, int32_t actualAlt)
{
    return desiredAlt - actualAlt;
}

//*****************************************************************************
// Function to calculate yaw error.
//*****************************************************************************
int32_t
calcYawError(int32_t desiredYaw, int32_t actualYaw)
{
    int16_t error = desiredYaw - actualYaw;

    // Wrap error in other direction if necessary.
    /*if (abs(error) > 180)
    {
        if (error < 0)
        {
            error += 360;
        } else
        {
            error -= 360;
        }
    }*/

    return error;
}
