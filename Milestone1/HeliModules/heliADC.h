#ifndef HELIADC_H_
#define HELIADC_H_

// *******************************************************
//
// heliADC.h
//
// Initialisation files and interrupt handlers for ADC
// sampling of an input.
//
// P.J. Bones UCECE
// Last modified:   9.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "circBufT.h"


// ****************************************************************************
// Globals to module
// ****************************************************************************
extern circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
#define HELI_CHANNEL ADC_CTL_CH9    // Analogue channel input.

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void);


//*****************************************************************************
// initADC - Initialise ADC pins and sampling for sequence 3
//*****************************************************************************
void
initADC (void);

#endif /*HELIADC_H_*/
