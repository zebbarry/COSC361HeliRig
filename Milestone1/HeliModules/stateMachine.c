// *******************************************************
//
// stateMachine.c
//
// Finite state machine state definitions and function module
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   20.5.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "stateMachine.h"
#include "buttons4.h"
#include "motorControl.h"
#include "yaw.h"
#include "heliPWM.h"


//********************************************************
// updateDesiredAlt - Updates desired altitude value
//********************************************************
int16_t
updateDesiredAlt(int16_t desiredAlt)
{
    // Increase desired altitude if UP button pressed.
    if (checkButton(UP) == PUSHED && desiredAlt < ALT_MAX_PER)
    {
        desiredAlt += ALT_STEP_PER;
        // Reset integral
        altErrorInt = 0;
    }

    // Decrease desired altitude if DOWN button pressed.
    if (checkButton(DOWN) == PUSHED && desiredAlt > ALT_MIN_PER)
    {
        desiredAlt -= ALT_STEP_PER;
        // Reset integral
        altErrorInt = 0;
    }
    return desiredAlt;
}

//********************************************************
// updateDesiredYaw - Updates desired yaw value
//********************************************************
int32_t
updateDesiredYaw(int32_t desiredYaw)
{
    // Increase desired yaw if RIGHT button pressed.
    if (checkButton(RIGHT) == PUSHED)
    {
        desiredYaw += YAW_STEP_DEG;
        // Reset integral
        yawErrorInt = 0;
    }

    // Decrease desired yaw if LEFT button pressed.
    if (checkButton(LEFT) == PUSHED)
    {
        desiredYaw -= YAW_STEP_DEG;
        // Reset integral
        yawErrorInt = 0;
    }
    return desiredYaw;
}


//********************************************************
// landed - Resets motors to off and checks switch
//********************************************************
enum state
landed (rotor_t *mainRotor, rotor_t *tailRotor)
{
    // If motors are on, turn off.
    if (mainRotor->state || tailRotor->state)
    {
        motorPower (mainRotor, false);
        motorPower (tailRotor, false);
    }

    // Check for SW change to trigger state change
    if (checkButton(SW) == PUSHED)
    {
        return TAKING_OFF;
    } else {
        return LANDED;
    }
}

//********************************************************
// takeOff - Orientates helicopter to yawRef
//********************************************************
enum state
takeOff (rotor_t *mainRotor, rotor_t *tailRotor)
{
    // Check motors are turned on and with correct duty cycles
    if (!mainRotor->state || !tailRotor->state ||
            mainRotor->duty != PWM_MIN_MAIN || tailRotor->duty != ROTATE_DUTY_TAIL)
    {
        mainRotor->duty = PWM_MIN_MAIN;
        tailRotor->duty = ROTATE_DUTY_TAIL;
        setPWM (mainRotor);
        setPWM (tailRotor);
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    // If yaw reference flag is true, change state
    if (hitYawRef)
    {
        yawRefIntDisable();
        hitYawRef = false;
        return FLYING;
    } else {
        return TAKING_OFF;
    }
}

//********************************************************
// flight - Flys helicopter checking for switch.
//********************************************************
enum state
flight (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    // Check if motors are on.
    if (!mainRotor->state || !tailRotor->state)
    {
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    // Fly helicopter, controlling motors using PID control from error values.
    fly (mainRotor, tailRotor, altError, yawError);

    // If SW changed to down, land heli.
    if (checkButton(SW) == RELEASED)
    {
        return LANDING;
    } else {
        return FLYING;
    }
}

//********************************************************
// land - Lands helicopter at reference angle.
//********************************************************
enum state
land (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError, int16_t mappedAlt)
{
    // Check if motors are on.
    if (!mainRotor->state || !tailRotor->state)
    {
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    // Fly helicopter, controlling motors using PID control from error values.
    fly (mainRotor, tailRotor, altError, yawError);

    // If altitude falls to below 2%, change to landed state
    if (mappedAlt < 2)
    {
        yawRefIntEnable();
        return LANDED;
    } else
        return LANDING;
}
