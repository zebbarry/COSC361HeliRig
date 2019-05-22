// ************************************************************
// kernel.h
// Helicopter project
// Group: A03 Group 10
// Last edited: 02-06-18
//
// Purpose: A paced round robin scheduler for running tasks as specified frequencies.
// Different frequencies are achieved by dividing the base frequency (using counters).
// ************************************************************

#ifndef KERNEL_H_
#define KERNEL_H_

#include <stdint.h>
#include <stdbool.h>

// The mode of the helicoptor.
typedef enum {
    STATE_LANDED = 0,
    STATE_CALIBRATE_YAW,
    STATE_FLYING,
    STATE_DESCENDING,
    STATE_POWER_DOWN,

    // the value of this enum is the number of heli states defined above
    NUM_HELI_STATES
} heli_state_t;


/*// Entries which more than one task needs to know about.
typedef struct {
    heli_state_t heliMode;
    int32_t targetHeight;
    int32_t targetYaw;
    int32_t outputMainDuty;
    int32_t outputTailDuty;
} state_t;*/

typedef struct state_struct
{
    uint8_t on;
} state_t;

typedef void (* task_func_t)(void *data);

// Object to configure a handler for use in the task scheduler.
typedef struct {
    task_func_t handler;  // pointer to task handler function
    void        *data;
    uint32_t    updateFreq;  // number of ms between runs
    uint32_t    count;  // used by the kernal only
    uint32_t    triggerAt;  // used by the kernal only
} task_t;


// A simple round robin scheduler.
// Uses an infinite loop to run the tasks at specified frequencies relative to baseFreq. Make
// sure baseFreq is greater than or equal to all task frequencies otherwise the tasks will
// not be run at the correct rate. A pointer to a state object stores entries applicable to
// many tasks.
void runTasks(task_t* tasks);


#endif /* KERNEL_H_ */
