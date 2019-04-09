//*****************************************************************************
//
// milestone2.c - Interrupt driven program which samples with AIN9 and
//              transmits the ADC value through UART0 at 4 Hz, while detecting
//              pin changes on PB0 and PB1 for quadrature decoding.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID:
// Author:  Jack Topliss        ID:
// Last modified:   9.4.2019
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "circBufT.h"
#include "buttons4.h"
#include "USBUART.h"
#include "display.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 1000
#define SLOWTICK_RATE_HZ 4
#define MAX_STR_LEN 16
//---Yaw Pin definitions
#define YAW_PIN_A               GPIO_PIN_0
#define YAW_PIN_B               GPIO_PIN_1
#define YAW_GPIO_BASE           GPIO_PORTB_BASE


//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
volatile uint8_t slowTick = false;
char statusStr[MAX_STR_LEN + 1];
static uint16_t inADC_max = 1835;
volatile static uint16_t yaw;
volatile static uint8_t stateA;
volatile static uint8_t stateB;
static uint8_t displayState = SCALED;


//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 
    g_ulSampCnt++;

    //
    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SAMPLE_RATE_HZ / SLOWTICK_RATE_HZ;

    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }

    updateButtons();
}

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void)
{
    uint8_t newStateA = GPIOPinRead(GPIO_PORTB_BASE, YAW_PIN_A) == YAW_PIN_A;
    uint8_t newStateB = GPIOPinRead(GPIO_PORTB_BASE, YAW_PIN_B) == YAW_PIN_B;

    if (newStateA != stateA)
    {
        yaw++;      // Moving clockwise
        stateA = newStateB;
    }
    else if (newStateB != stateB)
    {
        yaw--;      //Moving counterclockwise
        stateB = newStateB;
    }
}


//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

void 
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);    
                             
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
  
    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
  
    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void)
{
    // Enable GPIO Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Congifure Pin A and Pin B for input WPD
    GPIOPadConfigSet(YAW_GPIO_BASE, YAW_PIN_A, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(YAW_GPIO_BASE, YAW_PIN_B, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);

    // Set data direction register as input
    GPIODirModeSet(YAW_GPIO_BASE, YAW_PIN_A, GPIO_DIR_MODE_IN);
    GPIODirModeSet(YAW_GPIO_BASE, YAW_PIN_B, GPIO_DIR_MODE_IN);

    stateA = GPIOPinRead(YAW_GPIO_BASE, YAW_PIN_A);
    stateB = GPIOPinRead(YAW_GPIO_BASE, YAW_PIN_B);
    yaw = 0;
}

//********************************************************
// initAltitude - Calibrate height
//********************************************************
void
initAltitude (meanVal)
{
    inADC_max = meanVal - ALT_RANGE;
}


int
main(void)
{
    uint16_t i;
    uint16_t meanVal = 0;
    int32_t sum;
    bool init_prog = true;

    initButtons ();
    initClock ();
    initADC ();
    initDisplay ();
    initUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);

    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {
        //
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number, as long
        // as the entire buffer has been written into.
        if (g_inBuffer.written)
        {
            sum = 0;
            for (i = 0; i < BUF_SIZE; i++)
                sum = sum + readCircBuf (&g_inBuffer);
            // Calculate and display the rounded mean of the buffer contents
            meanVal = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;


            // If start of program, calibrate input
            if (init_prog)
            {
                init_prog = false;
                initAltitude(readCircBuf (&g_inBuffer));
            }
        }


        // Check buttons - LEFT = Reset zero position
        //               - UP   = Switch altitude unit
        if (checkButton(LEFT) == PUSHED)
        {
            initAltitude (meanVal);

        }
        if (checkButton(UP) == PUSHED)
        {
            switch (displayState)
            {
            case SCALED: displayState = MEAN;
                break;
            case MEAN: displayState = CLEAR;
                break;
            case CLEAR: displayState = SCALED;
                break;
            }
        }


        // Time to send a message through UART at set lower frequency SLOW_TICK_RATE_HZ
        if (slowTick && !init_prog)
        {
            slowTick = false;

            // Form and send a status message for altitude to the console
            usnprintf (statusStr, sizeof(statusStr), "ADC = %4d \r\n", meanVal); // * usprintf
            UARTSend (statusStr);

            // Form and send a status message for yaw to the console
            usnprintf (statusStr, sizeof(statusStr), "YAW = %4d \r\n", yaw); // * usprintf
            UARTSend (statusStr);

            displayMeanVal (meanVal, g_ulSampCnt, inADC_max, displayState);
            displayYaw (yaw);
        }


    }
}

