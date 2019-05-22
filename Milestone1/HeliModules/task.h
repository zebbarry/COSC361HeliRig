#ifndef HELIMODULES_TASK_H_
#define HELIMODULES_TASK_H_

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
/** @file   task.h
    @author M. P. Hayes, UCECE
    @date   17 August 2011
    @brief  Simple task scheduler.

    @defgroup task Simple task scheduler

    This scheduler periodically calls functions specified in an array
    of tasks to schedule.  There is no pre-emption; a task can hog the CPU.
    The tasks at the start of the array have a higher priority.

    Each task must maintain its own state; either using static local
    variables, file variables, or dynamically using the generic pointer
    passed when the task is called.

    Here's an example of two LED flashing tasks that run at 2 and 3 Hz.

    @code
 #include "task.h"
 #include "led.h"
 #include "system.h"

 static void led1_task (void *data)
 {
     static uint8_t state = 0;

     led_set (LED1, state);
     state = !state;
 }


 static void led2_task (void *data)
 {
     static uint8_t state = 0;

     led_set (LED2, state);
     state = !state;
 }


 int main (void)
 {
     task_t tasks[] =
     {
         {.func = led1_task, .period = TASK_RATE / 2, .data = 0},
         {.func = led2_task, .period = TASK_RATE / 3, .data = 0},
     };

     system_init ();
     led_init ();

     task_schedule (tasks, 2);
     return 0;
 }
@endcode

   Here's another similar example.  This time there are two instances
   of the same task but with different state.

@code
 #include "task.h"
 #include "led.h"
 #include "system.h"

 typedef struct state_struct
 {
     uint8_t led;
     uint8_t on;
 } state_t;


 static void led_task (void *data)
 {
     state_t *pstate = data;

     led_set (pstate->led, pstate->on);
     pstate->on = !pstate->on;
 }


 int main (void)
 {
     state_t led1 = {.led = LED1, .on = 0};
     state_t led2 = {.led = LED2, .on = 0};
     task_t tasks[] =
     {
         {.func = led_task, .period = TASK_RATE / 2, .data = &led1},
         {.func = led_task, .period = TASK_RATE / 3, .data = &led2},
     };

     system_init ();
     led_init ();

     task_schedule (tasks, 2);
     return 0;
 }
@endcode
*/

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "heliTimer.h"


/** Task function prototype.  */
typedef void (* task_func_t)(void *data);


/** Task structure.  */
typedef struct task_struct
{
    /** Function to call.  */
    task_func_t func;
    /** Generic pointer to pass to function.  */
    void *data;
    /** How many ticks between scheduling events.  */
    uint16_t period;
    /** When to reschedule.  */
    uint16_t reschedule;
} task_t;


/** Schedule tasks
    @param tasks pointer to array of tasks (the highest priority
                 task comes first)
    @param num_tasks number of tasks to schedule
    @return this never returns.
*/
void taskSchedule (task_t *tasks, uint16_t num_tasks);



#endif /* HELIMODULES_TASK_H_ */
