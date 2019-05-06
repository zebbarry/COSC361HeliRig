#ifndef HELIPWM_H_
#define HELIPWM_H_

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

/**********************************************************
 * Constants
 **********************************************************/
// Systick Configuration
#define SYSTICK_RATE_HZ    100

// PWM Configuration
#define PWM_MAIN_FREQ_HZ   250
#define PWM_TAIL_FREQ_HZ   200
#define PWM_START_DUTY_PER 5
#define PWM_DUTY_STEP_PER  5
#define PWM_DUTY_MIN_PER   2
#define PWM_DUTY_MAX_PER   98
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        4

//  PWM Hardware Details
//  ---Main Rotor PWM: PC5, J4-05 (M0PWM7), gen 3
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5
//  ---Tail Rotor PWM: PF1, J3-10 (M1PWM5), gen 2
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

enum motor {MAIN = 0, TAIL};
enum control {OFF = 0, ON};

/*********************************************************
 * initialisePWMMain
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/
void
initialisePWMMain (void);

/*********************************************************
 * initialisePWMTail
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/
void
initialisePWMTail (void);

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWM (uint32_t ui32Freq, uint32_t ui32Duty);

#endif /* HELIMODULES_HELIPWM_H_ */
