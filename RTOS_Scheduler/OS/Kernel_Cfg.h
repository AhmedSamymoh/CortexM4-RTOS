/*
 * @file     Kernel_Cfg.h
 * @date     Feb 13, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef KERNEL_CFG_H_
#define KERNEL_CFG_H_



/********************************************** Section : Includes ********************************************/

#include "Platform_Types.h"
#include "Std_Types.h"
#include "../MCAL/SYSTCK/SYSTICK_interface.h"


/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/
#define Max_Tasks_Number				4u

/* Queue Error Codes */
#define QUEUE_OK						0u
#define QUEUE_OVERFLOW					1u
#define QUEUE_UNDERFLOW					2u
#define QUEUE_EMPTY						3u

/* Task Priority Levels */
#define TASK_PRIORITY_IDLE				0u
#define TASK_PRIORITY_LOW				1u
#define TASK_PRIORITY_NORMAL			2u
#define TASK_PRIORITY_HIGH				3u



/************************************* Section : Global Variables Definitions **********************************/

/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/

#endif /* KERNEL_CFG_H_ */
