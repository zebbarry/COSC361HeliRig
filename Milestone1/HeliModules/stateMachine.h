#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

// *******************************************************
//
// stateMachine.h
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
#include "heliPWM.h"

//********************************************************
// Constants
//********************************************************
#define ALT_MAX_PER         100
#define ALT_MIN_PER         0
#define ALT_STEP_PER        10
#define YAW_STEP_DEG        15

//********************************************************
// Globals
//********************************************************
enum state {LANDED = 0, TAKING_OFF, FLYING, LANDING};
enum state heliState;
typedef struct heli_struct_t
{
    rotor_t *mainRotor;
    rotor_t *tailRotor;
    bool    initProg;
    int32_t mappedAlt;
    int32_t mappedYaw;
    int16_t desiredAlt;
    int32_t desiredYaw;
    enum state heliState;
} heli_t;

//********************************************************
// updateDesiredAlt - Updates desired altitude value
//********************************************************
int16_t
updateDesiredAlt(int16_t desiredAlt);

//********************************************************
// updateDesiredYaw - Updates desired yaw value
//********************************************************
int32_t
updateDesiredYaw(int32_t desiredYaw);

//********************************************************
// landed - Resets motors to off and checks switch
//********************************************************
enum state
landed (rotor_t *mainRotor, rotor_t *tailRotor);

//********************************************************
// takeOff - Orientates helicopter to yawRef
//********************************************************
enum state
takeOff (rotor_t *mainRotor, rotor_t *tailRotor);

//********************************************************
// flight - Flys helicopter checking for switch.
//********************************************************
enum state
flight (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError);

//********************************************************
// land - Lands helicopter at reference angle.
//********************************************************
enum state
land (rotor_t *mainRotor, rotor_t *tailRotor, int32_t altError, int32_t yawError, int16_t mappedAlt);

#endif /* STATEMACHINE_H_ */
