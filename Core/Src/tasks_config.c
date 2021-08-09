#include <obd2_controller.h>
#include "tasks_config.h"
#include "main.h"
#include "controller.h"

/******************************************************************************
* Variable Declarations
*******************************************************************************/
/**
 * Task configuration table.  Holds the task interval, last time executed, and
 * the function to be executed.  A continuous task is defined as a task with
 * an interval of 0.  Last time executed is set to 0.
 */
static TaskType tasks[] =
{
  	{ 0             , 0, controller_task	},
	{ 0             , 0, obd2_controller_task	},
	{ 0             , 0, uart2_response_task	},
	{ 3000			, 0, k_line_coolant_query_task},
  	{ 5000			, 0, k_line_health_task }
};


TaskType *tasks_get_config(void)
{
	return tasks;
}

uint8_t get_number_of_tasks(void)
{
	return sizeof(tasks) / sizeof(*tasks);
}
