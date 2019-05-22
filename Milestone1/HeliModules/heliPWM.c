// *******************************************************
//
// heliPWM.c
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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "heliPWM.h"


/*********************************************************
 * initPWMMain
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/
void
initPWMMain (rotor_t *rotor)
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    rotor->type = MAIN;
    rotor->freq = PWM_MAIN_FREQ_HZ;
    rotor->duty = PWM_START_DUTY_PER;
    rotor->state = false;
    setPWM (rotor);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, rotor->state);
}

/*********************************************************
 * initPWMTail
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/
void
initPWMTail (rotor_t *rotor)
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    rotor->type = TAIL;
    rotor->freq = PWM_TAIL_FREQ_HZ;
    rotor->duty = PWM_START_DUTY_PER;
    rotor->state = false;
    setPWM (rotor);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, rotor->state);
}


/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM (rotor_t *rotor)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / rotor->freq;

    if (rotor->type == MAIN)
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
            ui32Period * rotor->duty / 100);
    } else if (rotor->type == TAIL)
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
            ui32Period * rotor->duty / 100);
    }
}

/********************************************************
 * Function to set the power for a rotor.
 ********************************************************/
void
motorPower(rotor_t *rotor, bool power)
{
    if (rotor->type == MAIN)
    {
        rotor->state = power;
        PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, rotor->state);
    } else if (rotor->type == TAIL)
    {
        rotor->state = power;
        PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, rotor->state);
    }
}

