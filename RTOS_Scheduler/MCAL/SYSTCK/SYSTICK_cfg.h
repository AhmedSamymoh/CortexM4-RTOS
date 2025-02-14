/*
 * @file     SYSTICK_cfg.h
 * @date     Oct 5, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef SYSTCK_SYSTICK_CFG_H_
#define SYSTCK_SYSTICK_CFG_H_


/********************************************** Section : Includes ********************************************/

#include "SYSTICK_interface.h"

/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/
#define SYSTICK_AHB_DIV_1			1
#define SYSTICK_AHB_DIV_8			8

#define SYSTICK_CLK_SOURCE			SYSTICK_AHB_DIV_1

#define SYSTEM_CLK					16000000UL


/************************************* Section : Global Variables Definitions **********************************/


/************************************* Section : Macro Functions Definitions ***********************************/


/**************************************** Section : Functions Declarations *************************************/


#endif /* SYSTCK_SYSTICK_CFG_H_ */
