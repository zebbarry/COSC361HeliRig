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
handleHMI (heli_t *heli)
{
    // Output data to UART
    handleUART (heli);

    // Update OLED display with ADC, yaw value, duty cycles and state.
    displayMeanVal (heli->mappedAlt, heli->desiredAlt);
    displayYaw (heli->mappedYaw, heli->desiredYaw);
    displayPWM (heli->mainRotor, heli->tailRotor);
    displayState (heli->heliState);
}

//********************************************************
// handleUART - Handle output to UART port
//********************************************************
void
handleUART (heli_t *heli)
{
    char statusStr[MAX_STR_LEN + 1];
    // Form and send a status message for altitude to the console
    usnprintf (statusStr, sizeof(statusStr), "ALT: %3d [%3d]\r\n",
               heli->mappedAlt, heli->desiredAlt);
    UARTSend (statusStr);

    // Form and send a status message for yaw to the console
    int16_t mappedDesiredYaw = mapYaw2Deg (heli->desiredYaw, true);
    usnprintf (statusStr, sizeof(statusStr), "YAW: %3d [%3d]\r\n",
               heli->mappedYaw, mappedDesiredYaw);
    UARTSend (statusStr);

    // Form and send a status message for rotor duty cycles
    if (heli->mainRotor->state && heli->tailRotor->state)
    {
        usnprintf (statusStr, sizeof(statusStr), "MAIN %2d TAIL %2d\r\n",
                   heli->mainRotor->duty, heli->tailRotor->duty);
    } else {
        usnprintf (statusStr, sizeof(statusStr), "MAIN %2d TAIL %2d\r\n", 0, 0);
    }
    UARTSend (statusStr);

    // Send status message about helicopter state
    char *state[] = {"LANDED", "TAKE OFF", "FLYING", "LANDING", "ERROR"};
    // Leave enough space for the template, state and null terminator.
    usnprintf (statusStr, sizeof(statusStr), "HELI: %s\r\n\n", state[heliState]);
    UARTSend (statusStr);
}
