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
#include "stateMachine.h"

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
mapAlt(uint16_t meanVal, uint16_t inADCMax)
{
    int16_t scaledVal = ALT_RANGE - (meanVal - inADCMax);
    int16_t mappedVal = map(scaledVal, 0, ALT_RANGE, outADC_min, outADC_max);

    return mappedVal;
}

//*****************************************************************************
// Function to display the mean ADC value (10-bit value, note) and sample count.
//*****************************************************************************
void
displayMeanVal(int16_t mappedAlt, uint16_t desiredAlt)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display

    usnprintf (string, sizeof(string), "ALT: %3d [%3d]\n", mappedAlt, desiredAlt);

    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}

//*****************************************************************************
// Function to display the yaw value in degrees to display
//*****************************************************************************
void
displayYaw(int16_t mappedYaw, int16_t desiredYaw)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display

    usnprintf (string, sizeof(string), "YAW: %3d [%3d]\n", mappedYaw, mapYaw2Deg(desiredYaw));

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

    usnprintf (string, sizeof(string), "MAIN %2d TAIL %2d", main->duty, tail->duty);

    // Update line on display, first line.
    OLEDStringDraw (string, 0, 2);

}

//*****************************************************************************
// Function to display the helicopter state
//*****************************************************************************
void
displayState(enum state heliState)
{
    char string[MAX_DISP_LEN + 1];  // 16 characters across the display
    char* state;

    if (heliState == LANDED)
    {
        state = "LD";
    } else if (heliState == TAKING_OFF)
    {
        state = "TF";
    } else if (heliState == FLYING)
    {
        state = "FL";
    } else if (heliState == LANDING)
    {
        state = "LG";
    } else
    {
        state = "ER";
    }

    usnprintf (string, sizeof(string), "Heli State: %s", state);

    // Update line on display, first line.
    OLEDStringDraw (string, 0, 3);
}
