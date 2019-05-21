#ifndef HELIHMI_H_
#define HELIHMI_H_

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
#include "heliPWM.h"
#include "stateMachine.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define MAX_STR_LEN         17

//********************************************************
// handleHMI - Handle output to UART port and display.
//********************************************************
void
handleHMI (rotor_t *main, rotor_t *tail, uint16_t mappedAlt, int16_t mappedYaw,
           int16_t desiredAlt, int32_t desiredYaw);

//********************************************************
// handleUART - Handle output to UART port
//********************************************************
void
handleUART (rotor_t *main, rotor_t *tail, uint16_t mappedAlt, int16_t mappedYaw,
           int16_t desiredAlt, int32_t desiredYaw);

#endif /* HELIHMI_H_ */
