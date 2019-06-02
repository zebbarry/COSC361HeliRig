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
    if (checkButton(UP) == PUSHED && desiredAlt < ALT_MAX_PER)
    {
        desiredAlt += ALT_STEP_PER;
        altErrorInt = 0;
    }
    if (checkButton(DOWN) == PUSHED && desiredAlt > ALT_MIN_PER)
    {
        desiredAlt -= ALT_STEP_PER;
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
    if (checkButton(RIGHT) == PUSHED)
    {
        desiredYaw += YAW_STEP_DEG;
        yawErrorInt = 0;
    }

    if (checkButton(LEFT) == PUSHED)
    {
        desiredYaw -= YAW_STEP_DEG;
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
    if (mainRotor->state || tailRotor->state)
    {
        motorPower (mainRotor, false);
        motorPower (tailRotor, false);
    }

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
    if (!mainRotor->state || !tailRotor->state || mainRotor->duty != HOVER_DUTY_MAIN || tailRotor->duty != ROTATE_DUTY_TAIL)
    {
        mainRotor->duty = HOVER_DUTY_MAIN;
        tailRotor->duty = ROTATE_DUTY_TAIL;
        setPWM (mainRotor);
        setPWM (tailRotor);
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    if (hitYawRef)
    {
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
    if (!mainRotor->state || !tailRotor->state)
    {
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    fly (mainRotor, tailRotor, altError, yawError);

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
    if (!mainRotor->state || !tailRotor->state)
    {
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    fly (mainRotor, tailRotor, altError, yawError);

    if (mappedAlt < 1)
    {
        return LANDED;
    } else
        return LANDING;
}
