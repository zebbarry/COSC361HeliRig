// *******************************************************
//
// motorControl.c
//
// PID controller module for maintaining helicopter at set
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
#include "yaw.h"

//*****************************************************************************
// altController - Function to update main motor duty cycle to reduce alt error
// value to zero using PID control
//*****************************************************************************
void
altController(rotor_t *mainRotor, int32_t error)
{
    // Scales the values up by a constant so integers can be used. This removes rounding errors.
    int32_t errorIntMax = 1000 * DUTYSCALER;
    int32_t errorIntMin = 0;
    float Kp = 0.2;
    float Ki = 0.1;
    float Kd = 0.2;

    // Proportional: The error times the proportional gain (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error integral if current speed is less than maximum.
    // This removes integral windup.
    if (mainRotor->duty < PWM_MAX_MAIN)
    {
        altErrorInt += (2 * error + DUTYSCALER) / 2 / DUTYSCALER;
    }

    // Limit the summed error to between maximum and minimum values.
    if (altErrorInt > errorIntMax)
    {
        altErrorInt = errorIntMax;
    }
    else if (altErrorInt < errorIntMin)
    {
        altErrorInt = errorIntMin;
    }

    // Integral: Multiply the error sum by the integral gain (Ki)
    int32_t I = Ki * altErrorInt;

    // Derivative: Calculate change in error between now and previous operation,
    // then multiply by the differential gain (Kd)
    double D = Kd * (altErrorPrev - error);

    // Store error to be used to calculate the derivative change next time
    altErrorPrev = error;

    // Combine the proportional, integral and derivative components and then scales
    // back down using integer division. This reduces rounding error.
    int32_t PWM_Duty = (2 * (P + I + D) + DUTYSCALER) / 2 / DUTYSCALER;

    // Limit the duty cycle to between maximum and minimum values.
    if (PWM_Duty > PWM_MAX_MAIN)
    {
        PWM_Duty = PWM_MAX_MAIN;
    }
    else if (PWM_Duty < PWM_MIN_MAIN)
    {
        PWM_Duty = PWM_MIN_MAIN;
    }

    // Set the motor to calculated new duty cycle.
    mainRotor->duty = PWM_Duty;
    setPWM(mainRotor);
}

//*****************************************************************************
// yawController - Function to update tail motor duty cycle to reduce yaw error
// value to zero using PID control
//*****************************************************************************
void
yawController(rotor_t *tailRotor, int32_t error)
{
    // Scales the values up by a constant so integers can be used. This removes rounding errors.
    int32_t errorIntMax = 10000 * DUTYSCALER;
    int32_t errorIntMin = 0;
    float Kp = 0.5;
    float Ki = 0.09;
    float Kd = 0.1;

    // Proportional: The error times the proportional gain (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error integral if current speed is less than maximum
    // and more than minimum. This removes integral windup.
    if (tailRotor->duty < PWM_MAX && tailRotor->duty > PWM_MIN)
    {
        yawErrorInt += (2 * error + DUTYSCALER) / 2 / DUTYSCALER;
    }

    // Limit the summed error to between maximum and minimum values.
    if (yawErrorInt > errorIntMax)
    {
        yawErrorInt = errorIntMax;
    }
    else if (yawErrorInt < errorIntMin)
    {
        yawErrorInt = errorIntMin;
    }

    // Integral: Multiply the error sum by the integral gain (Ki)
    int32_t I = Ki * yawErrorInt;

    // Derivative: Calculate change in error between now and previous operation,
    // then multiply by the differential gain (Kd)
    double D = Kd * (yawErrorPrev - error);
    yawErrorPrev = error;

    // Combine the proportional, integral and derivative components and then scales
    // back down using integer division. This reduces rounding error.
    int32_t PWM_Duty = (2 * (P + I + D) + DUTYSCALER) / 2 / DUTYSCALER;

    // Limit the duty cycle to between maximum and minimum values.
    if (PWM_Duty > PWM_MAX)
    {
        PWM_Duty = PWM_MAX;
    }
    else if (PWM_Duty < PWM_MIN)
    {
        PWM_Duty = PWM_MIN;
    }

    // Set the motor to calculated new duty cycle.
    tailRotor->duty = PWM_Duty;
    setPWM(tailRotor);
}


//*****************************************************************************
// fly - Controls heli to desired position and angle
//*****************************************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    altController (mainRotor, altError);
    yawController (tailRotor, yawError);
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
    // Scales the values up by a constant so integers can be used. This removes rounding errors.
    return (desiredAlt * DUTYSCALER) - (actualAlt * DUTYSCALER);
}

//*****************************************************************************
// Function to calculate yaw error.
//*****************************************************************************
int32_t
calcYawError(int32_t desiredYaw, int32_t actualYaw)
{
    // Scales the values up by a constant so integers can be used. This removes rounding errors.
    int32_t error = (desiredYaw * DUTYSCALER) - (YAW_DEG(actualYaw) * DUTYSCALER);

    return error;
}
