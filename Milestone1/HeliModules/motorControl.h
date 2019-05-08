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
#define P_GAIN_MAIN  1   // Proportional Gain for main rotor
#define I_GAIN_MAIN  1   // Integral Gain for main rotor
#define P_GAIN_TAIL  1   // Proportional Gain for tail rotor
#define I_GAIN_TAIL  1   // Integral Gain for tail rotor
#define HOVER_DUTY_MAIN 30
#define HOVER_DUTY_TAIL 30

//*****************************************************************************
// Global variables
//*****************************************************************************
static int16_t yawErrorInt;
static int16_t altErrorInt;

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
void
updateMotors(rotor_t *mainRotor, rotor_t *tailRotor, int16_t altError, int16_t yawError);

//*****************************************************************************
// Function to integrate yaw and altitude error.
//*****************************************************************************
void
integrate(int16_t altError, int16_t yawError);

//*****************************************************************************
// Function to calculate altitude error.
//*****************************************************************************
int16_t
calcAltError(int16_t desiredAlt, int16_t actualAlt);

//*****************************************************************************
// Function to calculate yaw error.
//*****************************************************************************
int16_t
calcYawError(int16_t desiredYaw, int16_t actualYaw);

#endif /* MOTORCONTROL_H_ */
