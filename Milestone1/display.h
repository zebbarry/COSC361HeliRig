#ifndef DISPLAY_H_
#define DISPLAY_H_

// *******************************************************
//
// display.h
//
// Support for use of the OrbitOLED display
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID:
// Author:  Jack Topliss        ID:
// Last modified:   9.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
#define ALT_RANGE 800                 // Range of voltage for altitude reading
enum state {SCALED = 0, MEAN, CLEAR}; // State variable for altitude unit

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
// Function to display the mean ADC value (10-bit value, note) and sample count.
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint32_t count, uint16_t inADC_max, uint8_t displayState);

//*****************************************************************************
// Function to display the yaw value in degrees to display
//*****************************************************************************
void
displayYaw(uint16_t yaw);

#endif /*DISPLAY_H_*/
