#ifndef YAW_H_
#define YAW_H_

// *******************************************************
// buttons4.h
//
// Support for a set of FOUR specific buttons on the Tiva/Orbit.
// ENCE361 sample code.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
//
// P.J. Bones UCECE
// Last modified:  7.2.2018
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"


//*****************************************************************************
// Constants
//*****************************************************************************
//---Yaw Pin definitions
#define YAW_PIN_A               GPIO_PIN_0
#define YAW_PIN_B               GPIO_PIN_1
#define YAW_PORT_BASE           GPIO_PORTB_BASE
#define YAW_SYSCTL_PERIPH       SYSCTL_PERIPH_GPIOB


// *******************************************************
// Globals to module
// *******************************************************
volatile static uint16_t yaw;
volatile static uint8_t stateA;
volatile static uint8_t stateB;


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void);


//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void);

#endif /*YAW_H_*/
