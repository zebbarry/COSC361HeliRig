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
    // Calculate derivative error
    int32_t altErrorDer = (altError - altErrorPrev) * TIME_STEP;
    int32_t yawErrorDer = (yawError - yawErrorPrev) * TIME_STEP;

    altErrorPrev = altError;
    yawErrorPrev = yawError;

    int32_t newMainDuty = HOVER_DUTY_MAIN + (2*altError*P_GAIN_MAIN_MUL + P_GAIN_MAIN) / 2 / P_GAIN_MAIN;   // Proportional
    newMainDuty += (2*altErrorInt + I_GAIN_MAIN) / 2 / I_GAIN_MAIN;                         // Integral
    newMainDuty += (2*altErrorDer + D_GAIN_MAIN) / 2 / D_GAIN_MAIN;                       // Derivative

    int32_t newTailDuty = HOVER_DUTY_TAIL + (2*yawError*P_GAIN_TAIL_MUL + P_GAIN_TAIL) / 2 / P_GAIN_TAIL;   // Proportional
    //newTailDuty += (2*yawErrorInt + I_GAIN_TAIL) / 2 / I_GAIN_TAIL;                         // Integral
    newTailDuty += (2*yawErrorDer + D_GAIN_TAIL) / 2 / D_GAIN_TAIL;                       // Derivative

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

    // Set new duty cycles
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

    return error;
}

//********************************************************
// fly - Controls heli to desired position and angle
//********************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    integrate (altError, yawError);
    updateMotors (mainRotor, tailRotor, altError, yawError);
}
