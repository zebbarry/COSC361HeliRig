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
#define YAW_TABS 448      // Number of tabs in a full circle.
#define DEG_CIRC 360      // Number of degrees in full circle.
#define YAW_DEG(Y)  (2* Y * DEG_CIRC + YAW_TABS) / 2 / YAW_TABS
#define YAW_TAB(Y)  (2 * Y * YAW_TABS + DEG_CIRC) / 2 / DEG_CIRC

//---Yaw Pin definitions
#define YAW_PIN_A               GPIO_PIN_0      // PB0
#define YAW_PIN_B               GPIO_PIN_1      // PB1
#define YAW_PIN_REF             GPIO_PIN_4      // PC4
#define YAW_PORT_BASE           GPIO_PORTB_BASE
#define YAW_PORT_BASE_REF       GPIO_PORTC_BASE
#define YAW_SYSCTL_PERIPH       SYSCTL_PERIPH_GPIOB
#define YAW_SYSCTL_PERIPH_REF   SYSCTL_PERIPH_GPIOC

enum direction {CCW = -1, STATIC, CW};
enum yawState {BOTH_ZERO = 0, A_ONE, B_ONE, BOTH_ONE};


// ****************************************************************************
// Globals to module
// ****************************************************************************
volatile int32_t yaw;
volatile bool hitYawRef;
volatile static uint32_t currentState;
volatile static uint32_t previousState;
volatile static enum direction dir;


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void);

//*****************************************************************************
// The handler for the reference pin change for PC4.
//*****************************************************************************
void
yawRefIntHandler(void);

//********************************************************
// yawRefIntDisable - Disables yawRef interrrupt
//********************************************************
void
yawRefIntDisable(void);

//********************************************************
// yawRefIntEnable - Enables yawRef interrrupt
//********************************************************
void
yawRefIntEnable(void);

//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void);

//********************************************************
// mapYaw2Deg - Maps yaw value from raw input to degrees.
//********************************************************
int16_t
mapYaw2Deg(int32_t yawVal, bool deg);

#endif /*YAW_H_*/
