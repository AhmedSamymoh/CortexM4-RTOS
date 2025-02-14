/*
 * @file     main.h
 * @date     Feb 14, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef MAIN_H_
#define MAIN_H_


/********************************************** Section : Includes ********************************************/


#include <stdint.h>
#include <stdio.h>
#include "../MCAL/GPIO/GPIO_interface.h"
#include "../MCAL/SYSTCK/SYSTICK_interface.h"
#include "../OS/Kernel_interface.h"



void Task1_Handler();
void Task2_Handler();
void Task3_Handler();

/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/

/************************************* Section : Global Variables Definitions **********************************/
extern uint8 Os_Tick_Tog;
extern volatile uint32 Os_Idle_Task_Tick;
/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/



#endif /* MAIN_H_ */
