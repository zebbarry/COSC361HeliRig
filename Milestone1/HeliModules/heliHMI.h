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
handleHMI (heli_t *heli);

//********************************************************
// handleUART - Handle output to UART port
//********************************************************
void
handleUART (heli_t *heli);

#endif /* HELIHMI_H_ */
