// *******************************************************
//
// task.c
//
// Round robin scheduler for running tasks.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   5.5.2019
//
// *******************************************************
/** @file   task.c
    @author M. P. Hayes, UCECE
    @date   17 August 2011
    @brief  Simple task scheduler.
*/

#include <stdint.h>
#include <stdbool.h>
#include "task.h"
#include "heliTimer.h"


/** With 16-bit times the maximum value is 32768.  */
#define TASK_OVERRUN_MAX TIMER_OVERSHOOT_TICKS


/** Schedule tasks
    @param tasks pointer to array of tasks (the highest priority
                 task comes first)
    @param num_tasks number of tasks to schedule
    @return this never returns.
*/
void taskSchedule (task_t *tasks, uint8_t num_tasks)
{
    uint8_t i;
    uint16_t now;
    task_t *nextTask;

    timerInit ();
    now = timerGet ();

    /* Start by scheduling the first task.  */
    nextTask = tasks;

    while (1)
    {
        uint16_t sleepMin;

        /* Wait until the next task is ready to run.  */
        timerWaitUntil (nextTask->reschedule);

        /* Schedule the task.  */
        nextTask->func (nextTask->data);

        /* Update the reschedule time.  */
        nextTask->reschedule = nextTask->period;

        sleepMin = ~0;
        now = timerGet ();

        /* Search array of tasks.  Schedule the first task (highest priority)
           that needs to run otherwise wait until first task ready.  */
        for (i = 0; i < num_tasks; i++)
        {
            task_t * task = tasks + i;
            uint16_t overrun;

            overrun = now - task->reschedule;
            if (overrun < TASK_OVERRUN_MAX)
            {
                /* Have found a task that can run immediately.  */
                nextTask = task;
                break;
            }
            else
            {
                uint16_t sleep;

                sleep = -overrun;
                if (sleep < sleepMin)
                {
                    sleepMin = sleep;
                    nextTask = task;
                }
            }
        }
    }
}
