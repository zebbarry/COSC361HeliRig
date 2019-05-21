#ifndef DISPLAY_H_
#define DISPLAY_H_

// *******************************************************
//
// display.h
//
// Support for use of the OrbitOLED display
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
#include "stateMachine.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define ALT_RANGE 1000                 // Range of voltage for altitude reading
#define MAX_DISP_LEN 16

//*****************************************************************************
// Initialise OrbitOLED display.
//*****************************************************************************
void
initDisplay (void);

//*****************************************************************************
// Function to map input value from input range to output range.
//*****************************************************************************
int16_t
map(int16_t val, uint16_t min_in, uint16_t max_in, uint16_t min_out, uint16_t max_out);

//*****************************************************************************
// Function to map input ADC value to altitude range in percent.
//*****************************************************************************
int16_t
mapAlt(uint16_t meanVal, uint16_t inADCMax);

//*****************************************************************************
// Function to display the mean ADC value (10-bit value, note) and sample count.
//*****************************************************************************
void
displayMeanVal(int16_t mappedAlt, uint16_t desiredAlt);

//*****************************************************************************
// Function to display the yaw value in degrees to display
//*****************************************************************************
void
displayYaw(int16_t mappedYaw, int16_t desiredYaw);

//*****************************************************************************
// Function to display the PWM for main and tail rotors.
//*****************************************************************************
void
displayPWM(rotor_t *main, rotor_t *tail);

//*****************************************************************************
// Function to display the helicopter state
//*****************************************************************************
void
displayState(enum state heliState);

#endif /*DISPLAY_H_*/
