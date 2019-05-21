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


#define DUTYSCALER 1000
#define PWM_MIN                 5
#define PWM_MAX                 70

//********************************************************
// Global Vars
//********************************************************

int32_t MainDuty = 0;                   // Main Duty:
int32_t d_lastError_main = 0;                //
int32_t i_sum_main = 0;                // Accumulated error sum
int32_t PWM_last_main = 0;              //

int32_t TailDuty = 0;                   // Tail Duty:
int32_t d_lastError_tail = 0;                //
int32_t i_sum_tail = 0;                // Accumulated error sum
int32_t PWM_last_tail = 0;              //

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
// PID controller for the main motor
void
Maincontroller(rotor_t *mainRotor, int32_t target, int32_t current){

    // Scales the values up by a constant so for example 1000 * 0.01 can be 1010 instead of 1 * 0.01 getting 1.01
    // because decimals are inacurate and even small changes from rounding could be a problem
    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.5;
    float Ki = 0.1;
    float Kd = 0.2;
    int32_t error = target - current;

    // Proportional: The error times the proportional coefficent (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error sum
    i_sum_main += error / DUTYSCALER;

    // Limit the summed error to between i_max and i_min
    if (i_sum_main > i_Max) i_sum_main = i_Max;
    else if (i_sum_main < i_Min) i_sum_main = i_Min;

    // Integral: Multiply the sum by the integral coefficent (Ki)
    int32_t I = Ki * i_sum_main;

    // Derivative: Calculate change in error between now and last time through the controller
    // then multiply by the differential coefficent (Kd)
    int32_t D = Kd * (d_lastError_main - error);

    // Store error to be used to calculate the change next time
    d_lastError_main = error;

    // Combine the proportional, integral and derivative components and then scales back down.
    //      (looking at it again im not sure why this isn't just "P + I + D" as the previous
    //      duty cycle shouldn't matter, I'll test changing this)
    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit the duty cycle to between 95 and 5
    if (PWM_Duty > 70) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 5) PWM_Duty = PWM_MIN;

    PWM_last_main = PWM_Duty;

    mainRotor->duty = PWM_Duty;
    setPWM(mainRotor);
}

// PID controller for the tail motor [ See above for comments ]
void
Tailcontroller(rotor_t *tailRotor, int32_t target, int32_t current){

    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.2;
    float Ki = 0.1;
    float Kd = 0.08;
    int32_t error = target - current;

    // Proportional
    int32_t P = error * Kp;

    // Integral
    i_sum_tail += error / DUTYSCALER;

    // Limit sum
    if (i_sum_tail > i_Max) i_sum_tail = i_Max;
    else if (i_sum_tail < i_Min) i_sum_tail = i_Min;

    // Integral
    int32_t I = Ki * i_sum_tail;

    // Derivative
    int32_t D = Kd * (d_lastError_tail - error);
    d_lastError_tail = error;

    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit PWM to specification
    if (PWM_Duty > 70) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 5) PWM_Duty = PWM_MIN;

    PWM_last_tail = PWM_Duty;

    tailRotor->duty = PWM_Duty;
    setPWM(tailRotor);
}


//********************************************************
// fly - Controls heli to desired position and angle
//********************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    Maincontroller (mainRotor, altError, yawError);
    Tailcontroller (tailRotor, altError, yawError);
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
