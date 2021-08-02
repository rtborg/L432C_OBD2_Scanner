/****************************************************************************
* Title                 :   Scheduler
* Filename              :   Scheduler.h
* Author                :   JWB
* Origin Date           :   11/07/2012
* Version               :   1.0.0
* Compiler              :   IAR C/C++ Compiler for ARM v6.40.4
* Target                :   MKL25Z128VLK4
* Copyright             :   Beningo Engineering
* All Rights Reserved
*
* THIS SOFTWARE IS PROVIDED BY BENINGO ENGINEERING "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL BENINGO ENGINEERING OR ITS CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
* Notes                 :   None
*
* Revision Log:
*
* REV           Author        Date                  Changes
* 1.0.0          JWB       11/07/2012           Initial Release
*******************************************************************************/
/** \file Scheduler.h
 * \brief This module contains definitions for a task scheduler.
 */
#ifndef SCHEDULER_H_
#define SCHEDULER_H_
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				// For standard type definitions

/******************************************************************************
* Constants
*******************************************************************************/
/**
 * Define the system tick interval in milliseconds
 */
#define SYS_TICK_INTERVAL			1000UL

/**
 * The interval to call the 10 ms tasks
 */
#define INTERVAL_10MS        10

/**
 * The interval to call the 50 ms tasks
 */
#define INTERVAL_50MS        50

/**
 * The interval to call the 100 ms tasks
 */
#define INTERVAL_100MS       100

/**
 * The interval to call the 500 ms tasks
 */
#define INTERVAL_500MS       500

/**
 * The interval to call the 1000 ms tasks
 */
#define INTERVAL_1000MS       1000

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Configuration
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
/**
 * Struct TaskType
 * TaskType structure is used to define the parameters required in order to
 * configure a task.
 */
typedef struct
{
  	uint16_t Interval;			/**< Defines how often a task will run  */
  	uint32_t LastTick;			/**< Stores the last tick task was ran  */
  	void (*Func)(void);			/**< Function pointer to the task  */
}TaskType;

/******************************************************************************
* Variable Declarations
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*SCHEDULER_H_*/
/*** End of File **************************************************************/
