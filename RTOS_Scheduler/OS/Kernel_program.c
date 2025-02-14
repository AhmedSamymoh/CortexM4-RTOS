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
#include "main.h"

/************************************* Section : Macro Functions Definitions ************************************/

/************************************ Section : Global Variables Definitions ************************************/

uint8 Global_Current_Task = 0;

uint32 Task_PSP[Max_Tasks_Number] = {
		T1_STACK_START,
		T2_STACK_START,
		T3_STACK_START,
		T4_STACK_START,
};

/*************************************** Section : Functions Definitions ***************************************/


/**/
void UpdateNextTask(){
	Global_Current_Task++;
	Global_Current_Task = Global_Current_Task % Max_Tasks_Number;
}


__attribute__ ((naked)) void Stack_InitScheduler_Stack(uint32 Copy_u32SchedTOS){
	/* Arguments of function stored in the first 4 GPRs
	 * We use Main Stack Pointer to Organize stack of the scheduler */
	__asm volatile("MSR MSP, R0");

	/* Branching indirect */
	__asm volatile("BX LR");

}

uint32 GetCurrent_PSP_value(){
	return Task_PSP[Global_Current_Task];
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
    /* Array of task function pointers */
    void (*Local_u32TaskHandle[Max_Tasks_Number])(void) = {
        Task1_Handler,
        Task2_Handler,
        Task3_Handler,
        Task4_Handler
    };


    /* Loop through each task to initialize its stack */
    for (Index = 0u; Index < Max_Tasks_Number; ++Index) {
        /* Set the task's PSP address */
        Local_pu32TaskPSP = (uint32 *)Task_PSP[Index];

    	/* xPSR Register (Initialize Thumb instruction set) */
    	Local_pu32TaskPSP--;
    	*Local_pu32TaskPSP = Dummy_xPSR;

        /* PC Register (Task entry point) */
        Local_pu32TaskPSP--;
        *Local_pu32TaskPSP = (uint32)Local_u32TaskHandle[Index];


        /* LR Register (Dummy return address) */
        Local_pu32TaskPSP--;
        *Local_pu32TaskPSP = Dummy_LR;


        /* Initialize General-Purpose Registers (R0-R12) */
        for (j = 0; j < 13; j++) {
            Local_pu32TaskPSP--;
            *Local_pu32TaskPSP = 0;
        }

        /* Save Current PSP of the task for retrieval */
        Task_PSP[Index] = (uint32)Local_pu32TaskPSP;
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

void SavePSP_Value(uint32 Currnt_PSP_value){
	Task_PSP[Global_Current_Task] = Currnt_PSP_value;
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

