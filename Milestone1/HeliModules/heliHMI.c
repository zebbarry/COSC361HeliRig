// *******************************************************
//
// heliHMI.h
//
// Support for the HMI of the helicopter
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   21.5.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "heliHMI.h"
#include "utils/ustdlib.h"
#include "USBUART.h"
#include "display.h"
#include "heliPWM.h"
#include "stateMachine.h"
#include "yaw.h"

//********************************************************
// handleHMI - Handle output to UART port and display.
//********************************************************
void
handleHMI (rotor_t *main, rotor_t *tail, uint16_t mappedAlt, int16_t mappedYaw,
           int16_t desiredAlt, int32_t desiredYaw)
{
    // Output data to UART
    handleUART (main, tail, mappedAlt, mappedYaw, desiredAlt, desiredYaw);

    // Update OLED display with ADC, yaw value, duty cycles and state.
    displayMeanVal (mappedAlt, desiredAlt);
    displayYaw (mappedYaw, desiredYaw);
    displayPWM (main, tail);
    displayState (heliState);
}

//********************************************************
// handleUART - Handle output to UART port
//********************************************************
void
handleUART (rotor_t *main, rotor_t *tail, uint16_t mappedAlt, int16_t mappedYaw,
           int16_t desiredAlt, int32_t desiredYaw)
{
    char statusStr[MAX_STR_LEN + 1];
    // Form and send a status message for altitude to the console
    usnprintf (statusStr, sizeof(statusStr), "ALT: %3d [%3d]\r\n", mappedAlt, desiredAlt);
    UARTSend (statusStr);

    // Form and send a status message for yaw to the console
    int16_t mappedDesiredYaw = mapYaw2Deg (desiredYaw);
    usnprintf (statusStr, sizeof(statusStr), "YAW: %4d [%4d]\r\n", mappedYaw, mappedDesiredYaw);
    UARTSend (statusStr);

    if (main->state && tail->state)
    {
        usnprintf (statusStr, sizeof(statusStr), "MAIN %2d TAIL %2d\r\n", main->duty, tail->duty);
    } else {
        usnprintf (statusStr, sizeof(statusStr), "MAIN %2d TAIL %2d\r\n", 0, 0);
    }

    // Send status message about helicopter state
    char* state;
    if (heliState == LANDED)
    {
        state = "LANDED";
    } else if (heliState == TAKING_OFF)
    {
        state = "TAKING OFF";
    } else if (heliState == FLYING)
    {
        state = "FLYING";
    } else if (heliState == LANDING)
    {
        state = "LANDING";
    } else
    {
        state = "ERROR";
    }

    // Leave enough space for the template, state and null terminator.
    usnprintf (statusStr, sizeof(statusStr), "HELI: %s\r\n\n", state);
    UARTSend (statusStr);
}
