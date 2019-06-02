// ************************************************************
// kernel.c
// Helicopter project
// Group: A03 Group 10
// Last edited: 02-06-18
//
// Purpose: A paced round robin scheduler for running tasks as specified frequencies.
// Different frequencies are achieved by dividing the base frequency (using counters).
// ************************************************************


#include "kernel.h"
#include "heliTimer.h"


// A simple round robin scheduler.
// Uses an infinite loop to run the tasks at specified frequencies relative to baseFreq. Make
// sure baseFreq is greater than or equal to all task frequencies otherwise the tasks will
// not be run at the correct rate. A pointer to a state object stores entries applicable to
// many tasks.
void runTasks(task_t* tasks, uint32_t baseFreq)
{
    // initalise the value to count up to for each task so that
    // tasks can run at different frequencies
    int32_t deltaTime = 1000 / baseFreq;  // in milliseconds, hence the 1000 factor

    // loop until empty terminator task
    uint16_t i = 0;
    while (tasks[i].handler) {
        uint32_t triggerCount = baseFreq / tasks[i].updateFreq;

        // ensure a count of zero gets triggered since the counter will skip 0 and
        // start at 1.
        if (triggerCount == 0) {
            triggerCount = 1;
        }

        // initalise all tasks
        tasks[i].count = 0;
        tasks[i].triggerAt = triggerCount;
        i++;
    }

    // begin the main loop
    while (true) {
        int32_t referenceTime = timerGet();

        int i = 0;
        while (tasks[i].handler) {
            tasks[i].count++;

            // check if task should run in this update
            if (tasks[i].count == tasks[i].triggerAt) {
                tasks[i].count = 0;

                // run the task
                tasks[i].handler(tasks[i].data);
            }
              i++;
        }

        // make sure loop runs as a consistent speed
        timerWaitFrom(deltaTime, referenceTime);
    }
}
