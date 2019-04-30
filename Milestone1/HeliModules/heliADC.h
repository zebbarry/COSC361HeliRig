#ifndef ADC_H_
#define ADC_H_

// *******************************************************
//
// heliADC.h
//
// Initialisation files and interrupt handlers for ADC
// sampling of an input for measuring altitude of a
// helicopter.
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
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

#endif /*ADC_H_*/
