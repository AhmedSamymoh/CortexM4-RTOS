/*
 * @file     SYSTICK_program.c
 * @date     Oct 5, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

/********************************************** Section : Includes ********************************************/

#include "SYSTICK_interface.h"
#include "SYSTICK_cfg.h"


/************************************* Section : Macro Functions Definitions ************************************/

/************************************ Section : Global Variables Definitions ************************************/

static volatile uint32 uwTick = 0;
extern uint8 Os_Tick;
/*************************************** Section : Functions Definitions ***************************************/

Std_ReturnType SYSTICK_Init(void){
	Std_ReturnType retVar = E_NOT_OK;

#if SYSTICK_CLK_SOURCE == SYSTICK_AHB_DIV_1
	CLR_BIT(Systick->CSR, SYSTICK_CSR_CLKSOURCE);
#elif SYSTICK_CLK_SOURCE == SYSTICK_AHB_DIV_8
	SET_BIT(Systick->CSR, SYSTICK_CSR_CLKSOURCE);
#else
	retVar = E_NOT_OK;
#endif


    /* Set the SysTick Reload Value for 1 ms intervals*/
    Systick->RVR = (SYSTEM_CLK / (SYSTICK_AHB_DIV_8 * 1000)) - 1;


    /* Reset the Current Value Register*/
    Systick->CVR = 0;

    __asm volatile ("cpsie i");
    /* Enable SysTick and its interrupt*/
    SET_BIT(Systick->CSR, SYSTICK_CSR_ENABLE);
    SET_BIT(Systick->CSR, SYSTICK_CSR_TICKINT);

	return retVar;
}


/*
 *
 * |-------| |-----------------------------|
 * |  xPSR | |*****************************|
 * |  PC   | |*****************************|
 * |  LR   | |*****************************|
 * |  R12  | |<CPU read them automatically>|
 * |  R3   | |*****************************|
 * |  R2   | |*****************************|
 * |  R1   | |*****************************|
 * |  R0   |	<-- Current PSP		<-- PSP
 * |-------| |-----------------------------|
 * |  R4   | |*****************************|
 * |  R5   | |*****************************|
 * |  R6   | |*****************************|
 * |  R7   | |***<we read them manually>***|
 * |  R8   | |*****************************|
 * |  R9   | |*****************************|
 * |  R10  | |*****************************|
 * |  R11  | |*****************************|
 * |-------| |-----------------------------|
 */
__attribute__ ((naked))void SysTick_Handler(void) {
    uwTick++;
    TOG_BIT(Os_Tick, 1); //TOG_BIT((GPIOA->ODR), 5);

    /* --- Save the context of the current task --- */
	__asm volatile("MRS R0, PSP");

	 __asm volatile("STMDB R0!,{R4-R11}");

	/*pushing LR Value to call another function*/
	__asm volatile("PUSH {LR}");

	 __asm volatile("BL SavePSP_Value");

    /* --- Retrieve the context of the next task --- */

	 /*Decide next task to run*/
	 __asm volatile("BL UpdateNextTask");

	 /*Get its PSP Value*/
	 __asm volatile("BL GetCurrent_PSP_value"); /*Return Value Of the Function is Returned To R0*/

	 /*Popping the LR Register Value to return safely to the caller*/
	 __asm volatile("POP {LR}");

	 /*Using that psp value, retrieve remaining stack data (R4->R11)*/
	 __asm volatile("LDM R0!,{R4-R11}");

     /*so we should update the PSP Value */
	 __asm volatile("MSR PSP, R0");


	/*
	 * 4. Branch to LR to return from Interrupt handler
	 * LR --> contain EXC_RETURN Code
	 */
	__asm("BX LR");

}

void SYSTICK_Delay_ms(uint32 ms){
	/* Value of Reload Register */
	Systick->RVR = (SYSTEM_CLK/SYSTICK_AHB_DIV_8) * ms * 1000;

	/* Start the system timer */
	SET_BIT(Systick->CSR, SYSTICK_CSR_ENABLE);

	/* Wait on the count flag */
	while(! READ_BIT(Systick->CSR ,SYSTICK_CSR_COUNTFLAG));

	/* Stop the system timer */
	CLR_BIT(Systick->CSR, SYSTICK_CSR_ENABLE);
}
void SYSTICK_Delay_us(uint32 us){
	/* Value of Reload Register */
	Systick->RVR = (SYSTEM_CLK/SYSTICK_AHB_DIV_8) * us;

	/* Start the system timer */
	SET_BIT(Systick->CSR, SYSTICK_CSR_ENABLE);

	/* Wait on the count flag */
	while(! READ_BIT(Systick->CSR ,SYSTICK_CSR_COUNTFLAG));

	/* Stop the system timer */
	CLR_BIT(Systick->CSR, SYSTICK_CSR_ENABLE);
}

uint32 SYSTICK_Get_ElapsedTime(void){
    /* Read the Current Value Register to get the elapsed time */
    return (Systick->CVR / (SYSTEM_CLK / SYSTICK_CLK_SOURCE));
}

uint32 SYSTICK_Get_RemainingTime(void){
    /* Get the remaining time from the Reload Register minus the Current Value Register */
    return (Systick->RVR - Systick->CVR) / (SYSTEM_CLK / SYSTICK_CLK_SOURCE);
}

Std_ReturnType SYSTICK_EnableInterupt(void){
    /* Enable the system timer interrupt */
    SET_BIT(Systick->CSR, SYSTICK_CSR_TICKINT);
    return E_OK;
}

/* Returns the number of milliseconds since the system started */
uint32 SYSTICK_GetTick(void) {
    return Systick->CVR;
}




void SYSTICK_StartCountMillisecondsIT(uint32 ms, void (*ptr)(void)){

}

void SYSTICK_StartCountMicrosecondsIT(uint32 us, void (*ptr)(void)){

}

