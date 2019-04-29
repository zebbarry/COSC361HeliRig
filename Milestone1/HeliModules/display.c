// *******************************************************
//
// display.c
//
// Support for use of the OrbitOLED display
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID:
// Group:   Thu am 22
// Last modified:   9.4.2019
//
// *******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "display.h"
#include "yaw.h"

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
// Function to display the mean ADC value (10-bit value, note) and sample count.
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint16_t inADC_max, uint8_t displayState)
{
    char string[17];  // 16 characters across the display

    // If displaying percent, map to range 0-100.
    if (displayState == SCALED)
    {
        int16_t scaledVal = ALT_RANGE - (meanVal - inADC_max);
        int16_t mappedVal = map(scaledVal, 0, ALT_RANGE, outADC_min, outADC_max);

        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "Perc ADC = %4d", mappedVal);
    } else if (displayState == MEAN)
    {

        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
    } else if (displayState == CLEAR)
    {
        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "                ");
    }

    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}

//*****************************************************************************
// Function to display the yaw value in degrees to display
//*****************************************************************************
void
displayYaw(uint16_t yaw)
{
    char string[17];  // 16 characters across the display

    // Scale yaw into degrees
    uint16_t scaledYaw = (2*(yaw * YAW_DEG) + YAW_TABS) / 2 / YAW_TABS;
    uint16_t mappedYaw = scaledYaw - YAW_START;

    usnprintf (string, sizeof(string), "Yaw Deg = %4d", mappedYaw);

    // Update line on display, first line.
    OLEDStringDraw (string, 0, 0);

}
