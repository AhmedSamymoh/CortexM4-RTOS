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
#include "../MCAL/GPIO/GPIO_interface.h"
#include "../MCAL/SYSTCK/SYSTICK_interface.h"
#include "../OS/Kernel_interface.h"



void Task1_Handler();
void Task2_Handler();
void Task3_Handler();
void Task4_Handler();

/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/

/************************************* Section : Global Variables Definitions **********************************/

/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/



#endif /* MAIN_H_ */
