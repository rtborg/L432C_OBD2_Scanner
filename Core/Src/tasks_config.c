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
	{ 2000			, 0, obd2_coolant_query_task},
	{ 100			, 0, obd2_rpm_query_task},
	{ 50			, 0, obd2_speed_query_task},
  	{ 10000			, 0, k_line_health_task }
};


TaskType *tasks_get_config(void)
{
	return tasks;
}

uint8_t get_number_of_tasks(void)
{
	return sizeof(tasks) / sizeof(*tasks);
}
