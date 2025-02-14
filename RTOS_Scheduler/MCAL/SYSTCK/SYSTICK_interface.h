/*
 * @file     SYSTICK_interface.h
 * @date     Oct 5, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef SYSTCK_INC_SYSTICK_INTERFACE_H_
#define SYSTCK_INC_SYSTICK_INTERFACE_H_

/********************************************** Section : Includes ********************************************/

#include "Platform_Types.h"
#include "Std_Types.h"
#include "Stm32F446xx.h"
#include "../../OS/Kernel_interface.h"

/**************************************** Section: Data Type Declarations **************************************/

/* SysTick Control and Status Register (SYST_CSR) bits */
typedef enum
{
    SYSTICK_CSR_ENABLE        = 0,  /* Counter enable */
    SYSTICK_CSR_TICKINT       = 1,  /* SysTick exception request enable */
    SYSTICK_CSR_CLKSOURCE     = 2,  /* Clock source selection */
    SYSTICK_CSR_COUNTFLAG     = 16  /* Returns 1 if timer counted to 0 since last read */
} SYSTICK_CSR_bits_t;

/* SysTick Reload Value Register (SYST_RVR) bits */
typedef enum
{
    SYSTICK_RVR_RELOAD_0      = 0,  /* Reload value bit 0 */
    SYSTICK_RVR_RELOAD_1      = 1,  /* Reload value bit 1 */
    SYSTICK_RVR_RELOAD_2      = 2,  /* Reload value bit 2 */
    /* Continue with other reload value bits as needed... */
    SYSTICK_RVR_RELOAD_23     = 23  /* Reload value bit 23 */
} SYSTICK_RVR_bits_t;

/* SysTick Current Value Register (SYST_CVR) bits */
typedef enum
{
    SYSTICK_CVR_CURRENT_0     = 0,  /* Current value bit 0 */
    SYSTICK_CVR_CURRENT_1     = 1,  /* Current value bit 1 */
    /* Continue with other current value bits as needed... */
    SYSTICK_CVR_CURRENT_23    = 23  /* Current value bit 23 */
} SYSTICK_CVR_bits_t;



/****************************************** Section: Macro Declarations ****************************************/


/************************************* Section : Global Variables Definitions **********************************/
extern uint8 Os_Tick_Tog;
extern volatile uint32 Os_Tick;


/************************************* Section : Macro Functions Definitions ***********************************/


/**************************************** Section : Functions Declarations *************************************/

Std_ReturnType SYSTICK_Init(void);

void SYSTICK_Delay_us(uint32 us);
void SYSTICK_Delay_ms(uint32 ms);

uint32 SYSTICK_Get_ElapsedTime(void);
uint32 SYSTICK_Get_RemaningTime(void);

Std_ReturnType SYSTICK_EnableInterupt(void);
void SYSTICK_StartCountMillisecondsIT(uint32 ms, void (*ptr)(void));
void SYSTICK_StartCountMicrosecondsIT(uint32 us, void (*ptr)(void));


#endif /* SYSTCK_INC_SYSTICK_INTERFACE_H_ */

