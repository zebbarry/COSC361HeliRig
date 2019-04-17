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
// Author:  Mitchell Hollows    ID:
// Author:  Jack Topliss        ID:
// Last modified:   9.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define YAW_RATIO 1                   // Conversion ratio from quadrature reading to degrees.
#define YAW_START 0    // Middle point for yaw start.

//---Yaw Pin definitions
#define YAW_PIN_A               GPIO_PIN_0
#define YAW_PIN_B               GPIO_PIN_1
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

#endif /*YAW_H_*/
