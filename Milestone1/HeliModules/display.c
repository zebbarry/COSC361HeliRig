// *******************************************************
//
// display.c
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
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "display.h"
#include "yaw.h"
#include "heliPWM.h"

//*****************************************************************************
// Global variables
//*****************************************************************************
const uint16_t outADC_min = 0;
const uint16_t outADC_max = 100;

//*****************************************************************************
// Initialise OrbitOLED display.
//*****************************************************************************
void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
// Function to map input value from input range to output range.
//*****************************************************************************
int16_t
map(int16_t val, uint16_t min_in, uint16_t max_in, uint16_t min_out, uint16_t max_out)
{
    return (val - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

//*****************************************************************************
// Function to map input ADC value to altitude range in percent.
//*****************************************************************************
int16_t
mapAlt(uint16_t meanVal, uint16_t inADC_max)
{
    int16_t scaledVal = ALT_RANGE - (meanVal - inADC_max);
    int16_t mappedVal = map(scaledVal, 0, ALT_RANGE, outADC_min, outADC_max);

    return mappedVal;
}

//*****************************************************************************
// Function to display the mean ADC value (10-bit value, note) and sample count.
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint16_t inADC_max)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display

    int16_t mappedVal = mapAlt(meanVal, inADC_max);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Perc ADC = %5d", mappedVal);

    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}

//*****************************************************************************
// Function to display the yaw value in degrees to display
//*****************************************************************************
void
displayYaw(int16_t mappedYaw)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display

    usnprintf (string, sizeof(string), "Yaw Deg  = %5d", mappedYaw);

    // Update line on display, first line.
    OLEDStringDraw (string, 0, 0);

}

//*****************************************************************************
// Function to display the PWM for main and tail rotors.
//*****************************************************************************
void
displayPWM(rotor_t *main, rotor_t *tail)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display

    usnprintf (string, sizeof(string), "MAIN %2d TAIL %2d\r\n", main->duty, tail->duty);

    // Update line on display, first line.
    OLEDStringDraw (string, 0, 3);

}
