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

//********************************************************
// Global Vars
//********************************************************
int32_t PWMLastMain = 0;              //"
int32_t PWMLastTail = 0;              //

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
// PID controller for the main motor
void
mainController(rotor_t *mainRotor, int32_t error)
{
    // Scales the values up by a constant so for example 1000 * 0.01 can be 1010 instead of 1 * 0.01 getting 1.01
    // because decimals are inacurate and even small changes from rounding could be a problem
    int32_t errorIntMax = 10000 * DUTYSCALER;
    int32_t errorIntMin = 0;
    float Kp = 0.5;
    float Ki = 0.1;
    float Kd = 0.3;

    // Proportional: The error times the proportional coefficent (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error integral
    altErrorInt += error / DUTYSCALER;

    // Limit the summed error to between i_max and i_min
    if (altErrorInt > errorIntMax) altErrorInt = errorIntMax;
    else if (altErrorInt < errorIntMin) altErrorInt = errorIntMin;

    // Integral: Multiply the sum by the integral coefficent (Ki)
    int32_t I = Ki * altErrorInt;

    // Derivative: Calculate change in error between now and last time through the controller
    // then multiply by the differential coefficent (Kd)
    int32_t D = Kd * (altErrorPrev - error);

    // Store error to be used to calculate the change next time
    altErrorPrev = error;

    // Combine the proportional, integral and derivative components and then scales back down.
    //      (looking at it again im not sure why this isn't just "P + I + D" as the previous
    //      duty cycle shouldn't matter, I'll test changing this)
    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit the duty cycle to between 95 and 5
    if (PWM_Duty > PWM_MAX) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 20) PWM_Duty = 20;

    PWMLastMain = PWM_Duty;

    mainRotor->duty = PWM_Duty;
    setPWM(mainRotor);
}

// PID controller for the tail motor [ See above for comments ]
void
tailController(rotor_t *tailRotor, int32_t error)
{
    int32_t errorIntMax = 10000 * DUTYSCALER;
    int32_t errorIntMin = 0;
    float Kp = 0.2;
    float Ki = 0.2;
    float Kd = 0.2;

    // Proportional
    int32_t P = error * Kp;

    // Integral
    if (tailRotor->duty < PWM_MAX && tailRotor->duty > PWM_MIN)
    {
        yawErrorInt += error / DUTYSCALER;
    }

    // Limit sum
    if (yawErrorInt > errorIntMax) yawErrorInt = errorIntMax;
    else if (yawErrorInt < errorIntMin) yawErrorInt = errorIntMin;

    // Integral
    int32_t I = Ki * yawErrorInt;

    // Derivative
    int32_t D = Kd * (yawErrorPrev - error);
    yawErrorPrev = error;

    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit PWM to specification
    if (PWM_Duty > PWM_MAX) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < PWM_MIN) PWM_Duty = PWM_MIN;

    PWMLastTail = PWM_Duty;

    tailRotor->duty = PWM_Duty;
    setPWM(tailRotor);
}


//********************************************************
// fly - Controls heli to desired position and angle
//********************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    if (!debug) {
        mainController (mainRotor, altError);
    }
    tailController (tailRotor, yawError);
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
    return (desiredAlt * DUTYSCALER) - (actualAlt * DUTYSCALER);
}

//*****************************************************************************
// Function to calculate yaw error.
//*****************************************************************************
int32_t
calcYawError(int32_t desiredYaw, int32_t actualYaw)
{
    int32_t error = (desiredYaw * DUTYSCALER) - (actualYaw * DUTYSCALER);

    return error;
}
