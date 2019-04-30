#ifndef YAW_H_
#define YAW_H_

// *******************************************************
//
// yaw.h
//
// Initialisation files and interrupt handlers for quadrature
// decoding of a GPIO input signal for the yaw of a helicopter.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   29.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define YAW_TABS 112   // Number of tabs in a full circle.
#define DEG_CIRC 360    // Number of degrees in full circle.

//---Yaw Pin definitions
#define YAW_PIN_A               GPIO_PIN_0      // PB0
#define YAW_PIN_B               GPIO_PIN_1      // PB1
#define YAW_PORT_BASE           GPIO_PORTB_BASE
#define YAW_SYSCTL_PERIPH       SYSCTL_PERIPH_GPIOB


// ****************************************************************************
// Globals to module
// ****************************************************************************
volatile int16_t yaw;
volatile static uint8_t stateA;
volatile static uint8_t stateB;


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void);


//*****************************************************************************
// initYaw - Initialise yaw pins
//*****************************************************************************
void
initYaw (void);

//********************************************************
// mapYaw2Deg - Maps yaw value from raw input to degrees.
//********************************************************
int16_t
mapYaw2Deg(void);

#endif /*YAW_H_*/
