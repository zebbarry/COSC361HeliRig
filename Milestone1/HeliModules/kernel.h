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
#include "stateMachine.h"

typedef void (* task_func_t)(heli_t *data);

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
