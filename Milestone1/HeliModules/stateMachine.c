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
        desiredYaw += (2 * YAW_STEP_DEG * YAW_TABS + DEG_CIRC) / 2 / DEG_CIRC;
        yawErrorInt = 0;
    }
    if (checkButton(LEFT) == PUSHED)
    {
        desiredYaw -= (2 * YAW_STEP_DEG * YAW_TABS + DEG_CIRC) / 2 / DEG_CIRC;
        yawErrorInt = 0;
    }
    return desiredYaw;
}


//********************************************************
// landed - Resets motors to off and checks switch
//********************************************************
void
landed (rotor_t *mainRotor, rotor_t *tailRotor)
{
    if (mainRotor->state || tailRotor->state)
    {
       motorPower (mainRotor, false);
       motorPower (tailRotor, false);
       mainRotor->duty = 0;
       tailRotor->duty = 0;
       setPWM (mainRotor);
       setPWM (tailRotor);
    }

    if (checkButton(SW) == PUSHED)
    {
       heliState = TAKING_OFF;
    }
}

//********************************************************
// takeOff - Orientates helicopter to yawRef
//********************************************************
void
takeOff (rotor_t *mainRotor, rotor_t *tailRotor)
{
    if (!mainRotor->state || !tailRotor->state)
    {
        mainRotor->duty = HOVER_DUTY_MAIN;
        tailRotor->duty = HOVER_DUTY_TAIL + 5;
        setPWM (mainRotor);
        setPWM (tailRotor);
        motorPower (mainRotor, true);
        motorPower (tailRotor, true);
    }

    if (hitYawRef)
    {
        tailRotor->duty = HOVER_DUTY_TAIL;
        setPWM (tailRotor);
        hitYawRef = false;

        heliState = FLYING;
    }
}

//********************************************************
// flyController - Flys helicopter checking for switch.
//********************************************************
void
flight (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError)
{
    fly (mainRotor, tailRotor, altError, yawError);

    if (checkButton(SW) == RELEASED)
    {
        heliState = LANDING;
    }
}

//********************************************************
// land - Lands helicopter at reference angle.
//********************************************************
void
land (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError, int16_t mappedAlt)
{
    fly (mainRotor, tailRotor, altError, yawError);

    if (mappedAlt < 2)
    {
        heliState = LANDED;
    }
}
