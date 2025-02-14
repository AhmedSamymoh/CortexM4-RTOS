/*
 * @file     Kernel_program.c
 * @date     Feb 13, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */


/********************************************** Section : Includes ********************************************/

#include "Kernel_interface.h"

/************************************* Section : Macro Functions Definitions ************************************/

/************************************ Section : Global Variables Definitions ************************************/

uint8 Global_Current_Task = 1 ;
extern volatile uint32 Os_Tick;
volatile uint32 Os_Idle_Task_Tick;

Task_ControlBlock_t UserTasks[Max_Tasks_Number];

/*************************************** Section : Functions Definitions ***************************************/


void OS_TaskDelay(uint32 Copy_BlockCount){
	if(Global_Current_Task != 0){/*Check if it 's Idle Task */
		Enter_Critical_Section();

		/* Update Task blocking period */
		UserTasks[Global_Current_Task].BlockCount = Copy_BlockCount + Os_Tick;
		/* Update Task State */
		UserTasks[Global_Current_Task].CurrentState = TASK_BlockedState;
	    /* Triggering PendSV Exception */
	    SET_BIT(SCB->ICSR,28);

	    Exit_Critical_Section();
	}
}

void OS_UnblockTasks(void){
	uint8 Local_TaskCounter;
	for (Local_TaskCounter = 0; Local_TaskCounter < Max_Tasks_Number; Local_TaskCounter++) {
		if(UserTasks[Local_TaskCounter].CurrentState == TASK_BlockedState){
			if(UserTasks[Local_TaskCounter].BlockCount <= Os_Tick){
				/* Update Task State */
				UserTasks[Local_TaskCounter].CurrentState = TASK_ReadyState;
			}
		}
	}
}

void OS_IdleTask(void){
	for(;;){
		TOG_BIT(Os_Idle_Task_Tick,1);
	}
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
__attribute__ ((naked)) void PendSV_Handler(void){

	/* --- Save the context of the current task --- */
	/*
	* 1. Get the Current_PSP from CPU registers
	* |-------|
	* |  xPSR |
	* |  PC   |
	* |  LR   |
	* |  R12  |
	* |  R3   |
	* |  R2   |
	* |  R1   |
	* |  R0   | <-- Current PSP
	* |-------|
	*/__asm volatile("MRS R0, PSP");
	/*
	* 2. using that psp value , store remaining stack data (R4->R11)
	* |-------|
	* |  R4   |
	* |  R5   |
	* |  R6   |
	* |  R7   |
	* |  R8   |
	* |  R9   |
	* |  R10  |
	* |  R11  |
	* |-------|
	* The R0 now has the updated value of R0 after Saving GPRs*/
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

/**/
void UpdateNextTask(void){

	for (int TaskCount = 1; TaskCount < Max_Tasks_Number ; ++TaskCount) {

		Global_Current_Task++;

		Global_Current_Task = Global_Current_Task % Max_Tasks_Number;

		if ((UserTasks[Global_Current_Task].CurrentState == TASK_ReadyState) && ( Global_Current_Task !=0 )) {
			break;
		}
	}
	/*All tasks are Blocked*/
	if(UserTasks[Global_Current_Task].CurrentState == TASK_BlockedState){
		Global_Current_Task =0; /*Jump to IDLE Task*/
	}
}


__attribute__ ((naked)) void Stack_InitScheduler_Stack(uint32 Copy_u32SchedTOS){
	/* Arguments of function stored in the first 4 GPRs
	 * We use Main Stack Pointer to Organize stack of the scheduler */
	__asm volatile("MSR MSP, R0");

	/* Branching indirect */
	__asm volatile("BX LR");
}


uint32 GetCurrent_PSP_value(void){
	return UserTasks[Global_Current_Task].pspValue;
}

void SavePSP_Value(uint32 Currnt_PSP_value){
	UserTasks[Global_Current_Task].pspValue = Currnt_PSP_value;
}

/**
 * @brief Switches the stack pointer to Process Stack Pointer (PSP).
 *
 * This function changes the currently active stack pointer from the Main Stack Pointer (MSP)
 * to the Process Stack Pointer (PSP).
 *
 * @note The function is marked as "naked" to ensure no prologue or epilogue is added by the compiler.
 */
__attribute__ ((naked)) void ChangeToPSP(){

	/*pushing LR Value to call another function*/
	__asm volatile("PUSH {LR}");

	/* Initialize PSP with task1 stack address*/
	__asm volatile("BL GetCurrent_PSP_value"); /*Return Value Of the Function is Returned To R0*/

	__asm volatile("MSR PSP, R0");

	/*Popping the LR Register Value to return safely to the caller*/
	__asm volatile("POP {LR}");

	 /*Defines the currently active stack pointer to PSP using CONTROL Register
	 1 = PSP is the current stack pointer.*/
	__asm volatile("MOV R0, #0x02");
    /* Set CONTROL register to switch to PSP (bit 1 = 1) */
	__asm volatile("MSR CONTROL, R0");
    /* Return to caller */
	__asm volatile("BX LR");

}




/**
 * @brief Initializes the Process Stack Pointer (PSP) for each task.
 *
 * This function sets up the initial stack frame for each task, including:
 * - xPSR: Initializes the Thumb instruction set.
 * - PC: Sets the task entry point.
 * - LR: Stores a dummy return address.
 * - General-purpose registers (R0-R12): Initialized to zero.
 * - Saves the stack pointer for each task.
 */
void Stack_InitTasks_Stack() {
    uint32 *Local_pu32TaskPSP;
    uint32 Index = 0;
    uint8 j = 0;

    /*PSP Tasks Initialization*/
    UserTasks[0].pspValue = IDLE_Task_STACK_START;
    UserTasks[1].pspValue = T1_STACK_START;
    UserTasks[2].pspValue = T2_STACK_START;
    UserTasks[3].pspValue = T3_STACK_START;


    /*Task Handler Initialization*/
    UserTasks[0].TaskHandler = &OS_IdleTask;
    UserTasks[1].TaskHandler = &Task1_Handler;
    UserTasks[2].TaskHandler = &Task2_Handler;
    UserTasks[3].TaskHandler = &Task3_Handler;



    /* Loop through each task to initialize its stack */
    for (Index = 0u; Index < Max_Tasks_Number; ++Index) {

    	/*Initialize tasks as ReadyState*/
    	UserTasks[Index].CurrentState = TASK_ReadyState;

    	/* Set the task's PSP address */
        Local_pu32TaskPSP = (uint32 *)UserTasks[Index].pspValue;

    	/* xPSR Register (Initialize Thumb instruction set) */
    	Local_pu32TaskPSP--;
    	*Local_pu32TaskPSP = Dummy_xPSR;

        /* PC Register (Task entry point) */
        Local_pu32TaskPSP--;
        *Local_pu32TaskPSP = (uint32)(UserTasks[Index].TaskHandler);


        /* LR Register (Dummy return address) */
        Local_pu32TaskPSP--;
        *Local_pu32TaskPSP = Dummy_LR;


        /* Initialize General-Purpose Registers (R0-R12) */
        for (j = 0; j < 13; j++) {
            Local_pu32TaskPSP--;
            *Local_pu32TaskPSP = 0;
        }

        /* Save Current PSP of the task for retrieval */
        UserTasks[Index].pspValue = (uint32)Local_pu32TaskPSP;
    }
}
/**
 * @brief Enables the Cortex-M Fault Exceptions.
 *
 * This function enables the following system fault exceptions:
 * - Memory Management Fault
 * - Bus Fault
 * - Usage Fault
 *
 * @return Std_ReturnType
 *         - E_OK: Indicates successful enabling of fault exceptions.
 */
Std_ReturnType Enable_FaultException(){
    /* Enable Memory Management Fault */
    SET_BIT(SCB->SHCSR, 16);

    /* Enable Bus Fault */
    SET_BIT(SCB->SHCSR, 17);

    /* Enable Usage Fault */
    SET_BIT(SCB->SHCSR, 18);

    return E_OK;
}


/**
 * @brief Handles Memory Management Fault.
 *
 * This handler is triggered when a memory protection violation occurs.
 */
void MemManage_Handler(){
    for(;;);
}

/**
 * @brief Handles Bus Fault.
 *
 * This handler is triggered when an access to an invalid memory region occurs.
 */
void BusFault_Handler(){
    for(;;);
}

/**
 * @brief Handles Usage Fault.
 *
 * This handler is triggered when an undefined instruction or illegal state is encountered.
 */
void UsageFault_Handler(){
    for(;;);
}

