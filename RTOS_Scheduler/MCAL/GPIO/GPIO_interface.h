/*
 * @file     GPIO_interface.h
 * @date     Sep 21, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains Interfaces configurations to managing GPIO pins on STM32f466xx.
 */

#ifndef MCAL_GPIO_INC_GPIO_INTERFACE_H_
#define MCAL_GPIO_INC_GPIO_INTERFACE_H_


/********************************************** Section : Includes ********************************************/
#include "Std_Types.h"
#include "Stm32F446xx.h"

/**************************************** Section: Data Type Declarations **************************************/

typedef enum
{
	PORTA=0,
	PORTB,
	PORTC,
	PORTD,
	PORTE,
	PORTF,
	PORTG,
	PORTH,
}GPIO_Port_t;

typedef enum{
	PIN0=0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15,
}GPIO_Pin_t;

typedef enum{
	INPUT=0,
	OUTPUT,
	ALTERNATE_FUNCTION,
	ANALOG
}GPIO_Mode_t;

typedef enum{
	LOW=0,
	MEDIUM,
	FAST,
	HIGH
}GPIO_OutputSpeed_t;

typedef enum{
	PUSH_PULL=0,
	OPEN_DRAIN
}GPIO_OutputType_t;

typedef enum{
	PIN_LOW=0,
	PIN_HIGH
} GPIO_PinVal_t;

typedef enum{
	NO_PULL=0,
	PULLUP,
	PULLDOWN
}GPIO_PullUpDown_t;

typedef enum
{
	AF0=0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15,
}GPIO_AlternateFunc_t;


typedef struct
{
	GPIO_Port_t        Port;
	GPIO_Pin_t         Pin;
	GPIO_Mode_t        Mode;
	GPIO_PullUpDown_t  PullType;
	GPIO_OutputType_t  OutputType;
	GPIO_OutputSpeed_t Speed;
	GPIO_AlternateFunc_t AltFunc;
}GPIO_Pin_Config_t;



/****************************************** Section: Macro Declarations ****************************************/

/************************************* Section : Macro Functions Definitions ***********************************/
extern GPIO_Pin_Config_t BUILT_IN_LED;
extern GPIO_Pin_Config_t LED2;
/**************************************** Section : Functions Declarations *************************************/

Std_ReturnType GPIO_u8PinInit(const GPIO_Pin_Config_t * PinConfig);

Std_ReturnType GPIO_u8SetPinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber, GPIO_PinVal_t PinVal);

Std_ReturnType GPIO_u8ReadPinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber, GPIO_PinVal_t * PinVal);

Std_ReturnType GPIO_u8TogglePinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber);

#endif /* MCAL_GPIO_INC_GPIO_INTERFACE_H_ */









/********************************************** Section : Includes ********************************************/



/**************************************** Section: Data Type Declarations **************************************/


/****************************************** Section: Macro Declarations ****************************************/

/************************************* Section : Global Variables Definitions **********************************/

/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/






/********************************************** Section : Includes ********************************************/


/************************************ Section : Global Variables Definitions ************************************/


/************************************* Section : Macro Functions Definitions ************************************/

/*************************************** Section : Functions Definitions ***************************************/
