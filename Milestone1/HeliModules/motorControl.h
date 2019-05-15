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
#define P_GAIN_MAIN  2   // Proportional Gain for main rotor
#define I_GAIN_MAIN  200   // Integral Gain for main rotor
#define P_GAIN_TAIL  4   // Proportional Gain for tail rotor
#define I_GAIN_TAIL  100   // Integral Gain for tail rotor
#define HOVER_DUTY_MAIN 40
#define HOVER_DUTY_TAIL 35

//*****************************************************************************
// Global variables
//*****************************************************************************
static int32_t yawErrorInt;
static int32_t altErrorInt;

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
void
updateMotors(rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError);

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
