#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

// *******************************************************
//
// motorControl.h
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
#include "heliPWM.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define ROTATE_DUTY_TAIL    28     // Rotate duty cycle for tail
#define TIME_STEP           1000   // Time step between samples of derivative
#define DROP_ALT_STEP       4      // Altitude drop step when landing
#define DUTYSCALER 1000            // Prescaler for error so integers can be used.
#define PWM_MIN    5
#define PWM_MAX    70
#define PWM_MAX_MAIN 60
#define PWM_MIN_MAIN 25

//*****************************************************************************
// Global variables
//*****************************************************************************
static int32_t yawErrorInt;
static int32_t altErrorInt;
static int32_t yawErrorPrev;
static int32_t altErrorPrev;

//*****************************************************************************
// altController - Function to update main motor duty cycle to reduce alt error
// value to zero using PID control
//*****************************************************************************
void
altController(rotor_t *mainRotor, int32_t error);

//*****************************************************************************
// yawController - Function to update tail motor duty cycle to reduce yaw error
// value to zero using PID control
//*****************************************************************************
void
yawController(rotor_t *tailRotor, int32_t error);

//*****************************************************************************
// fly - Controls heli to desired position and angle
//*****************************************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError);

//*****************************************************************************
// Function to integrate yaw and altitude error.
//*****************************************************************************
void
integrate(int32_t altError, int32_t yawError);

//*****************************************************************************
// Function to calculate altitude error.
//*****************************************************************************
int32_t
calcAltError(int32_t desiredAlt, int32_t actualAlt);

//*****************************************************************************
// Function to calculate yaw error.
//*****************************************************************************
int32_t
calcYawError(int32_t desiredYaw, int32_t actualYaw);

#endif /* MOTORCONTROL_H_ */
