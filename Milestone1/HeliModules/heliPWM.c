// *******************************************************
//
// heliPWM.h
//
// Support for use of the PWM modules for a tail and
// main rotor.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   4.5.2019
//
// *******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pwm.h"
#include "heliPWM.h"


/*********************************************************
 * initialisePWMMain
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/
void
initialisePWMMain (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setPWM (PWM_MAIN_FREQ_HZ, PWM_START_DUTY_PER);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

/*********************************************************
 * initialisePWMTail
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/
void
initialisePWMTail (void)
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setPWM (PWM_TAIL_FREQ_HZ, PWM_TAIL_DUTY_PER);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);

    // Calculate the PWM period corresponding to the freq.
    ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_TAIL_FREQ_HZ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * PWM_TAIL_DUTY_PER / 100);
}


/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM (uint32_t ui32Freq, uint32_t ui32Duty, enum motor rotor)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    if (rotor == MAIN)
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
            ui32Period * ui32Duty / 100);
    } else if (rotor == TAIL)
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
            ui32Period * ui32Duty / 100);
    }
}

void
motorState(enum motor rotor, bool on)
{
    if (rotor == MAIN)
    {
        if (on)
        {
            PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
        } else
        {
            PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
        }
    } else if (rotor == TAIL)
    {
        if (on)
        {
            PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
        } else
        {
            PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
        }
    }
}

