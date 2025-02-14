/*
 * @file     Kernel_interface.h
 * @date     Feb 13, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef KERNEL_INTERFACE_H_
#define KERNEL_INTERFACE_H_

/********************************************** Section : Includes ********************************************/

#include "Platform_Types.h"
#include "Std_Types.h"
#include "../MCAL/SYSTCK/SYSTICK_interface.h"
#include "Kernel_Cfg.h"


/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/
#define SRAM_SIZE				(128*1024U)
#define SRAM_START				0X20000000U
#define SRAM_END			    ((SRAM_START) + (SRAM_SIZE))

#define TASK_STACK_SIZE 		1024U
#define SCHED_STACK_SIZE 		1024U

#define T1_STACK_START			(SRAM_END)
#define T2_STACK_START			((SRAM_END) - (1* (TASK_STACK_SIZE)) )
#define T3_STACK_START			((SRAM_END) - (2* (TASK_STACK_SIZE)) )
#define T4_STACK_START			((SRAM_END) - (3* (TASK_STACK_SIZE)) )
#define SCHED_TASK_STACK		((SRAM_END) - (4* (TASK_STACK_SIZE)) )

#define Dummy_xPSR				0x01000000
#define Dummy_LR				0xFFFFFFFD




/************************************* Section : Global Variables Definitions **********************************/

extern void Task1_Handler();
extern void Task2_Handler();
extern void Task3_Handler();
extern void Task4_Handler();

/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/



void UpdateNextTask();
void Stack_InitTasks_Stack();
Std_ReturnType Enable_FaultException();
uint32 GetCurrent_PSP_value();
void SavePSP_Value(uint32 Currnt_PSP_value);

__attribute__ ((naked)) void Stack_InitScheduler_Stack(uint32 Copy_u32SchedTOS);
__attribute__ ((naked)) void ChangeToPSP();

#endif /* KERNEL_INTERFACE_H_ */
