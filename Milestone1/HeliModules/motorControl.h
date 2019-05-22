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
#define HOVER_DUTY_MAIN 25     // Hover duty cycle for main
#define ROTATE_DUTY_TAIL 32     // Hover duty cycle for tail
#define TIME_STEP       1000   // Time step between samples of derivative

//*****************************************************************************
// Global variables
//*****************************************************************************
static int32_t yawErrorInt;
static int32_t altErrorInt;
static int32_t yawErrorPrev;
static int32_t altErrorPrev;
static bool debug = false;

//*****************************************************************************
// Function to update motor duty cycles to reduce error values to zero.
//*****************************************************************************
/*
void
updateMotors(rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError);
*/

void
mainController(rotor_t *mainRotor, int32_t error);

void
tailController(rotor_t *tailRotor, int32_t error);

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

//********************************************************
// fly - Controls heli to desired position and angle
//********************************************************
void
fly (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError);

#endif /* MOTORCONTROL_H_ */
