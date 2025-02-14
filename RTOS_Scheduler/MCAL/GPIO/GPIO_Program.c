/*
 * @file     GPIO_Program.c
 * @date     Sep 21, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains functions for configuring and managing GPIO pins on STM32f466xx.
 */


/********************************************** Section : Includes ********************************************/

#include "GPIO_interface.h"

/************************************* Section : Macro Functions Definitions ************************************/

/************************************ Section : Global Variables Definitions ************************************/
GPIO_Pin_Config_t BUILT_IN_LED = {
	.Port = PORTA,
	.Pin = PIN5,
	.Mode = OUTPUT,
	.OutputType = PUSH_PULL,
	.Speed = LOW,
	.PullType = NO_PULL,
	.AltFunc = 0
};
GPIO_Pin_Config_t LED2 = {
	.Port = PORTA,
	.Pin = PIN6,
	.Mode = OUTPUT,
	.OutputType = PUSH_PULL,
	.Speed = LOW,
	.PullType = NO_PULL,
	.AltFunc = 0
};

/*************************************** Section : Functions Definitions ***************************************/


Std_ReturnType GPIO_u8PinInit(const GPIO_Pin_Config_t * PinConfig){
	Std_ReturnType retVar = E_NOT_OK;
	if(PinConfig == NULL_PTR ){
		retVar = E_NOT_OK;
	}else if( (PinConfig->Port > PORTH )|| (PinConfig->Pin > PIN15 )){
		retVar = E_NOT_OK;
	}else{
		switch(PinConfig->Port){
			case PORTA:
				GPIOA_PCLK_EN();
				/*Clear the two bits in the MODER register then Set the new mode to the correct pin position*/
				GPIOA->MODER &= ~(0x3 << (PinConfig->Pin * 2));
				GPIOA->MODER |= ( PinConfig->Mode << (PinConfig->Pin * 2 ));
			
				if(ANALOG != PinConfig->Mode){
					if (INPUT != PinConfig->Mode)
					{
						/* Configure Output Type (Push-Pull or Open-Drain) */
						GPIOA->OTYPER &= ~(1 << PinConfig->Pin);
						GPIOA->OTYPER |= (PinConfig->OutputType << PinConfig->Pin);
						/* Configure Output Speed (Low, Medium, High, or Very High)*/
						GPIOA->OSPEEDR &= ~(0x3 << (PinConfig->Pin * 2));
						GPIOA->OSPEEDR |= (PinConfig->Speed << (PinConfig->Pin * 2));
					}
					/* Configure Pull-up/Pull-down (None, Pull-up, or Pull-down)*/
					GPIOA->PUPDR &= ~(0x3 << (PinConfig->Pin * 2));
					GPIOA->PUPDR |= (PinConfig->PullType << (PinConfig->Pin * 2));
				}
	            /* If Alternate Function is needed, configure it*/
	            if (PinConfig->Mode == ALTERNATE_FUNCTION) {
	                /* Alternate function register (AFR) is split into two registers
	                 * (AFRL and AFRH) for pins 0-7 and 8-15*/
	                if (PinConfig->Pin < 8){
	                    GPIOA->AFR[0] &= ~(0xF << (PinConfig->Pin * 4));
	                    GPIOA->AFR[0] |= (PinConfig->AltFunc << (PinConfig->Pin * 4));
	                } else{
	                    GPIOA->AFR[1] &= ~(0xF << ((PinConfig->Pin - 8) * 4));
	                    GPIOA->AFR[1] |= (PinConfig->AltFunc << ((PinConfig->Pin - 8) * 4));
	                }
	            }
				break;


			case PORTB:
				GPIOB_PCLK_EN();
				/*Clear the two bits in the MODER register then Set the new mode to the correct pin position*/
				GPIOB->MODER &= ~(0x3 << (PinConfig->Pin * 2));
				GPIOB->MODER |= ( PinConfig->Mode << (PinConfig->Pin * 2 ));
				if(ANALOG != PinConfig->Mode){
					if (INPUT != PinConfig->Mode)
					{
						/* Configure Output Type (Push-Pull or Open-Drain) */
						GPIOB->OTYPER &= ~(1 << PinConfig->Pin);
						GPIOB->OTYPER |= (PinConfig->OutputType << PinConfig->Pin);
						/* Configure Output Speed (Low, Medium, High, or Very High)*/
						GPIOB->OSPEEDR &= ~(0x3 << (PinConfig->Pin * 2));
						GPIOB->OSPEEDR |= (PinConfig->Speed << (PinConfig->Pin * 2));
					}
					/* Configure Pull-up/Pull-down (None, Pull-up, or Pull-down)*/
					GPIOB->PUPDR &= ~(0x3 << (PinConfig->Pin * 2));
					GPIOB->PUPDR |= (PinConfig->PullType << (PinConfig->Pin * 2));
				}
	            /* If Alternate Function is needed, configure it*/
	            if (PinConfig->Mode == ALTERNATE_FUNCTION) {
	                /* Alternate function register (AFR) is split into two registers
	                 * (AFRL and AFRH) for pins 0-7 and 8-15*/
	                if (PinConfig->Pin < 8) {
	                    GPIOB->AFR[0] &= ~(0xF << (PinConfig->Pin * 4));
	                    GPIOB->AFR[0] |= (PinConfig->AltFunc << (PinConfig->Pin * 4));
	                } else {
	                    GPIOB->AFR[1] &= ~(0xF << ((PinConfig->Pin - 8) * 4));
	                    GPIOB->AFR[1] |= (PinConfig->AltFunc << ((PinConfig->Pin - 8) * 4));
	                }
	            }
				break;

			case PORTC:
				GPIOC_PCLK_EN();
				/*Clear the two bits in the MODER register then Set the new mode to the correct pin position*/
				GPIOC->MODER &= ~(0x3 << (PinConfig->Pin * 2));
				GPIOC->MODER |= ( PinConfig->Mode << (PinConfig->Pin * 2 ));
				if(ANALOG != PinConfig->Mode){
					if (INPUT != PinConfig->Mode)
					{
						/* Configure Output Type (Push-Pull or Open-Drain) */
						GPIOC->OTYPER &= ~(1 << PinConfig->Pin);
						GPIOC->OTYPER |= (PinConfig->OutputType << PinConfig->Pin);
						/* Configure Output Speed (Low, Medium, High, or Very High)*/
						GPIOC->OSPEEDR &= ~(0x3 << (PinConfig->Pin * 2));
						GPIOC->OSPEEDR |= (PinConfig->Speed << (PinConfig->Pin * 2));
					}
					/* Configure Pull-up/Pull-down (None, Pull-up, or Pull-down)*/
					GPIOC->PUPDR &= ~(0x3 << (PinConfig->Pin * 2));
					GPIOC->PUPDR |= (PinConfig->PullType << (PinConfig->Pin * 2));
				}
	            /* If Alternate Function is needed, configure it*/
	            if (PinConfig->Mode == ALTERNATE_FUNCTION) {
	                /* Alternate function register (AFR) is split into two registers
	                 * (AFRL and AFRH) for pins 0-7 and 8-15*/
	                if (PinConfig->Pin < 8) {
	                    GPIOC->AFR[0] &= ~(0xF << (PinConfig->Pin * 4));
	                    GPIOC->AFR[0] |= (PinConfig->AltFunc << (PinConfig->Pin * 4));
	                } else {
	                    GPIOC->AFR[1] &= ~(0xF << ((PinConfig->Pin - 8) * 4));
	                    GPIOC->AFR[1] |= (PinConfig->AltFunc << ((PinConfig->Pin - 8) * 4));
	                }
	            }
				break;

			case PORTD:
				GPIOD_PCLK_EN();
				/*Clear the two bits in the MODER register then Set the new mode to the correct pin position*/
				GPIOD->MODER &= ~(0x3 << (PinConfig->Pin * 2));
				GPIOD->MODER |= ( PinConfig->Mode << (PinConfig->Pin * 2 ));
				if(ANALOG != PinConfig->Mode){
					if (INPUT != PinConfig->Mode)
					{
						/* Configure Output Type (Push-Pull or Open-Drain) */
						GPIOD->OTYPER &= ~(1 << PinConfig->Pin);
						GPIOD->OTYPER |= (PinConfig->OutputType << PinConfig->Pin);
						/* Configure Output Speed (Low, Medium, High, or Very High)*/
						GPIOD->OSPEEDR &= ~(0x3 << (PinConfig->Pin * 2));
						GPIOD->OSPEEDR |= (PinConfig->Speed << (PinConfig->Pin * 2));
					}
					/* Configure Pull-up/Pull-down (None, Pull-up, or Pull-down)*/
					GPIOD->PUPDR &= ~(0x3 << (PinConfig->Pin * 2));
					GPIOD->PUPDR |= (PinConfig->PullType << (PinConfig->Pin * 2));
				}
	            /* If Alternate Function is needed, configure it*/
	            if (PinConfig->Mode == ALTERNATE_FUNCTION) {
	                /* Alternate function register (AFR) is split into two registers
	                 * (AFRL and AFRH) for pins 0-7 and 8-15*/
	                if (PinConfig->Pin < 8) {
	                    GPIOD->AFR[0] &= ~(0xF << (PinConfig->Pin * 4));
	                    GPIOD->AFR[0] |= (PinConfig->AltFunc << (PinConfig->Pin * 4));
	                } else {
	                    GPIOD->AFR[1] &= ~(0xF << ((PinConfig->Pin - 8) * 4));
	                    GPIOD->AFR[1] |= (PinConfig->AltFunc << ((PinConfig->Pin - 8) * 4));
	                }
	            }
				break;

			case PORTE:
				GPIOE_PCLK_EN();
				/*Clear the two bits in the MODER register then Set the new mode to the correct pin position*/
				GPIOE->MODER &= ~(0x3 << (PinConfig->Pin * 2));
				GPIOE->MODER |= ( PinConfig->Mode << (PinConfig->Pin * 2 ));
				if(ANALOG != PinConfig->Mode){
					if (INPUT != PinConfig->Mode)/* Output or Alternate */
					{
						/* Configure Output Type (Push-Pull or Open-Drain) */
						GPIOE->OTYPER &= ~(1 << PinConfig->Pin);
						GPIOE->OTYPER |= (PinConfig->OutputType << PinConfig->Pin);
						/* Configure Output Speed (Low, Medium, High, or Very High)*/
						GPIOE->OSPEEDR &= ~(0x3 << (PinConfig->Pin * 2));
						GPIOE->OSPEEDR |= (PinConfig->Speed << (PinConfig->Pin * 2));
					}
					/* Configure Pull-up/Pull-down (None, Pull-up, or Pull-down)*/
					GPIOE->PUPDR &= ~(0x3 << (PinConfig->Pin * 2));
					GPIOE->PUPDR |= (PinConfig->PullType << (PinConfig->Pin * 2));
				}
	            /* If Alternate Function is needed, configure it*/
	            if (PinConfig->Mode == ALTERNATE_FUNCTION) {
	                /* Alternate function register (AFR) is split into two registers
	                 * (AFRL and AFRH) for pins 0-7 and 8-15*/
	                if (PinConfig->Pin < 8) {
	                    GPIOE->AFR[0] &= ~(0xF << (PinConfig->Pin * 4));
	                    GPIOE->AFR[0] |= (PinConfig->AltFunc << (PinConfig->Pin * 4));
	                } else {
	                    GPIOE->AFR[1] &= ~(0xF << ((PinConfig->Pin - 8) * 4));
	                    GPIOE->AFR[1] |= (PinConfig->AltFunc << ((PinConfig->Pin - 8) * 4));
	                }
	            }
				break;
			default:
				break;
		}

		retVar = E_OK;
	}
	return retVar;
}

Std_ReturnType GPIO_u8SetPinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber, GPIO_PinVal_t PinVal){
    Std_ReturnType retVar = E_NOT_OK;
    GPIO_Registers_t *GPIOx;  /* Pointer to GPIO port structure*/

    if ((Port > PORTH) || (PinNumber > PIN15)) {
        retVar = E_NOT_OK;
    } else {
        switch (Port) {
            case PORTA: GPIOx = GPIOA; break;
            case PORTB: GPIOx = GPIOB; break;
            case PORTC: GPIOx = GPIOC; break;
            case PORTD: GPIOx = GPIOD; break;
            case PORTE: GPIOx = GPIOE; break;
            case PORTF: GPIOx = GPIOF; break;
            case PORTG: GPIOx = GPIOG; break;
            case PORTH: GPIOx = GPIOH; break;

            default: GPIOx = NULL_PTR; break;
        }
        if (GPIOx != NULL_PTR) {
            if (PinVal == PIN_HIGH) {
                GPIOx->BSRR = (1 << PinNumber);
            } else if (PinVal == PIN_LOW) {
                GPIOx->BSRR = (1 << (PinNumber + 16));
            } else {
                retVar = E_NOT_OK;
            }
            retVar = E_OK;
        }else{
			retVar = E_NOT_OK;
		}
    }
    return retVar;
}


Std_ReturnType GPIO_u8ReadPinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber, GPIO_PinVal_t * PinVal){
	Std_ReturnType retVar = E_NOT_OK;

	if ((Port > PORTH) || (PinNumber > PIN15)) {
		retVar = E_NOT_OK;
	} else {
		switch (Port) {
			case PORTA : *PinVal = READ_BIT((GPIOA->IDR), PinNumber); break;
			case PORTB : *PinVal = READ_BIT((GPIOB->IDR), PinNumber); break;
			case PORTC : *PinVal = READ_BIT((GPIOC->IDR), PinNumber); break;
			case PORTD : *PinVal = READ_BIT((GPIOD->IDR), PinNumber); break;
			case PORTE : *PinVal = READ_BIT((GPIOE->IDR), PinNumber); break;
			case PORTF : *PinVal = READ_BIT((GPIOF->IDR), PinNumber); break;
			case PORTG : *PinVal = READ_BIT((GPIOG->IDR), PinNumber); break;
			case PORTH : *PinVal = READ_BIT((GPIOH->IDR), PinNumber); break;
			
			default: retVar = E_NOT_OK;
				break;
		}
		retVar = E_OK;
	}
	return retVar;
}

Std_ReturnType GPIO_u8TogglePinValue(const GPIO_Port_t Port, GPIO_Pin_t PinNumber){
	Std_ReturnType retVar = E_NOT_OK;

	if ((Port > PORTH) || (PinNumber > PIN15)) {
		retVar = E_NOT_OK;
	} else {
		switch (Port) {
			case PORTA : TOG_BIT((GPIOA->ODR), PinNumber); break;
			case PORTB : TOG_BIT((GPIOB->ODR), PinNumber); break;
			case PORTC : TOG_BIT((GPIOC->ODR), PinNumber); break;
			case PORTD : TOG_BIT((GPIOD->ODR), PinNumber); break;
			case PORTE : TOG_BIT((GPIOE->ODR), PinNumber); break;
			case PORTF : TOG_BIT((GPIOF->ODR), PinNumber); break;
			case PORTG : TOG_BIT((GPIOG->ODR), PinNumber); break;
			case PORTH : TOG_BIT((GPIOH->ODR), PinNumber); break;

			default: retVar = E_NOT_OK;
				break;
		}
		retVar = E_OK;
	}
	return retVar;
}

