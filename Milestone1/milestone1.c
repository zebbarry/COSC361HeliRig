//*****************************************************************************
//
// milestone1.c - Simple interrupt driven program which samples with AIN9 and
//              transmits the ADC value through UART0 at 4 Hz.
//
// Author:  Zeb Barry
// Last modified:   19.3.2019
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
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 1000
#define SLOWTICK_RATE_HZ 4
#define MAX_STR_LEN 16
#define MAGIC 800
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
volatile uint8_t slowTick = false;
char statusStr[MAX_STR_LEN + 1];
static uint16_t in_min = 985;
static uint16_t in_max = 1835;
const uint16_t out_min = 0;
const uint16_t out_max = 100;

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
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
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
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

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

//********************************************************
// initUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

//********************************************************
// initAltitude - Calibrate height
//********************************************************
void
initAltitude (meanVal)
{
    in_min = meanVal;
    in_max = in_min - MAGIC;
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

//*****************************************************************************
//
// Function to map ADC input to actual voltage range.
//
//*****************************************************************************
int16_t map(int16_t val, uint16_t min_in, uint16_t max_in, uint16_t min_out, uint16_t max_out)
{
    return (val - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint32_t count, uint8_t state)
{
    char string[17];  // 16 characters across the display

    // If displaying percent, map to range 0-100.
    if (state == 0)
    {
        int16_t scaledVal = MAGIC - (meanVal - in_max);
        int16_t mappedVal = map(scaledVal, 0, MAGIC, out_min, out_max);

        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "Perc ADC = %4d", mappedVal);
    } else if (state == 1)
    {

        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
    } else
    {
        // Form a new string for the line.  The maximum width specified for the
        //  number field ensures it is displayed right justified.
        usnprintf (string, sizeof(string), "                ");
    }

    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw (string, 0, 3);
}


int
main(void)
{
    uint16_t i;
    uint16_t meanVal = 0;
    int32_t sum;
    uint8_t state = 0; // State variable for altitude unit
    uint8_t init_prog = 1;

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
                init_prog = 0;
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
            state++;

            if (state > 2)
            {
                state = 0;
            }
        }


        // Time to send a message through UART at set lower frequency
        if (slowTick && !init_prog)
        {
            slowTick = false;

            // Form and send a status message to the console
            usnprintf (statusStr, sizeof(statusStr), "ADC = %4d \r\n", meanVal); // * usprintf
            UARTSend (statusStr);

            displayMeanVal (meanVal, g_ulSampCnt, state);
        }


    }
}

