/*
 * @file: Stm32F446xx.h
 * @date     Sep 21, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_



#include "Std_Types.h"


/* ===================================================================================================== */
/* 									   Various Memories Base Address                                     */
/* ===================================================================================================== */

#define FLASH_BASE_ADDRESS			0x08000000UL
#define	SRAM_BASE_ADDRESS		    0x20000000UL
#define ROM_BASE_ADDRESS			0x1FFF0000UL
/* ===================================================================================================== */
/* 							    AHBx and APBx Bus Peripheral Base Addresses                              */
/* ===================================================================================================== */

#define PERIPH_BASE			        0x40000000U		/* Peripheral base address in the alias region  */
#define APB1PERIPH_BASE		        PERIPH_BASE		/* Base address of APB1 peripheral memory */
#define APB2PERIPH_BASE		        0x40010000U		/* Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASE		        0x40020000U		/* Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASE		        0x50000000U		/* Base address of AHB2 peripheral memory */

/* ===================================================================================================== */
/* 								    Base Addresses for AHB1 Peripherals                                  */
/* ===================================================================================================== */

#define RCC_BASE			(AHB1PERIPH_BASE + 0x3800) /* RCC base address	 					*/
#define GPIOA_BASE			(AHB1PERIPH_BASE + 0x0000) /* GPIOA Base Address					*/
#define GPIOB_BASE			(AHB1PERIPH_BASE + 0x0400) /* GPIOB Base Address					*/
#define GPIOC_BASE			(AHB1PERIPH_BASE + 0x0800) /* GPIOC Base Address					*/
#define GPIOD_BASE			(AHB1PERIPH_BASE + 0x0C00) /* GPIOD Base Address					*/
#define GPIOE_BASE			(AHB1PERIPH_BASE + 0x1000) /* GPIOE Base Address					*/
#define GPIOF_BASE			(AHB1PERIPH_BASE + 0x1400) /* GPIOF Base Address					*/
#define GPIOG_BASE			(AHB1PERIPH_BASE + 0x1800) /* GPIOG Base Address					*/
#define GPIOH_BASE			(AHB1PERIPH_BASE + 0x1C00) /* GPIOH Base Address					*/


/* ===================================================================================================== */
/* 								    Base Addresses for APB1 Peripherals                                  */
/* ===================================================================================================== */

#define SPI2_BASE			(APB1PERIPH_BASE + 0x3800) /* SPI2 base address   */
#define SPI3_BASE			(APB1PERIPH_BASE + 0x3C00) /* SPI3 base address   */

/* ===================================================================================================== */
/* 								    Base Addresses for APB2 Peripherals                                  */
/* ===================================================================================================== */
#define SPI1_BASE			(APB2PERIPH_BASE + 0x3000) /* SPI1 base address   */
#define SPI4_BASE			(APB2PERIPH_BASE + 0x3400) /* SPI4 base address   */


#define RCC_APB2ENR_SPI1EN          12U
#define RCC_APB2ENR_SPI4EN          13U
#define RCC_APB1ENR_SPI2EN          14U
#define RCC_APB1ENR_SPI3EN          15U




/* ===================================================================================================== */
/* 								   	       Base Addresses for SYSTICK                                    */
/* ===================================================================================================== */
#define SYSTICK_BASE		(0xE000E010)

/* ===================================================================================================== */
/* 								            Peripheral Registers RCC                                     */
/* ===================================================================================================== */

typedef struct
{
	volatile uint32 CR;				    /* RCC clock control register, 	    							Address offset: 0x00 	*/
	volatile uint32 PLLCFGR;			/* RCC PLL configuration register, 	    						Address offset: 0x04 	*/
	volatile uint32 CFGR;				/* RCC clock configuration register, 	   						Address offset: 0x08 	*/
	volatile uint32 CIR;				/* RCC clock interrupt register, 	    						Address offset: 0x0C 	*/
	volatile uint32 AHB1RSTR;			/* RCC AHB1 peripheral reset register, 	    					Address offset: 0x10 	*/
	volatile uint32 AHB2RSTR;			/* RCC AHB2 peripheral reset register, 	    					Address offset: 0x14 	*/
	volatile uint32 AHB3RSTR;			/* RCC AHB3 peripheral reset register, 	    					Address offset: 0x18 	*/
	uint32  Reserved_0;	                /* RCC reserved register, 										Address offset: 0x1C 	*/
	volatile uint32 APB1RSTR;			/* RCC APB1 peripheral reset register, 	    					Address offset: 0x20 	*/
	volatile uint32 APB2RSTR;			/* RCC APB2 peripheral reset register, 	    					Address offset: 0x24 	*/
	uint32  Reserved_1[2];              /* RCC reserved register, 										Address offset: 0x28-2C */
	volatile uint32 AHB1ENR;			/* RCC AHB1 peripheral clock enable register, 	    			Address offset: 0x30 	*/
	volatile uint32 AHB2ENR;			/* RCC AHB2 peripheral clock enable register, 	    			Address offset: 0x34 	*/
	volatile uint32 AHB3ENR;			/* RCC AHB3 peripheral clock enable register, 	    			Address offset: 0x38 	*/
	uint32  Reserved_2;	                /* RCC reserved register, 										Address offset: 0x3C 	*/
	volatile uint32 APB1ENR;			/* RCC APB1 peripheral clock enable register, 	    			Address offset: 0x40 	*/
	volatile uint32 APB2ENR;			/* RCC APB2 peripheral clock enable register, 	    			Address offset: 0x44 	*/
    uint32 Reserved_3[2];               /* RCC reserved register, 										Address offset: 0x48-4C */
    volatile uint32 AHB1LPENR;		    /* RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50 	*/
    volatile uint32 AHB2LPENR;		    /* RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 	*/
    volatile uint32 AHB3LPENR;		    /* RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 	*/
	uint32  Reserved_4;	                /* RCC reserved register, 										Address offset: 0x5C 	*/
	volatile uint32 APB1LPENR;		    /* RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60 	*/
	volatile uint32 APB2LPENR;		    /* RCC APB2 peripheral clock enable in low power mode register,	Address offset: 0x64 	*/
	uint32  Reserved_5[2];              /* RCC reserved register, 										Address offset: 0x68-6C */
	volatile uint32 BDCR;				/* RCC back up domain control register, 						Address offset: 0x70 	*/
	volatile uint32 CSR;				/* RCC clock control & status register, 						Address offset: 0x74 	*/
	uint32  Reserved_6[2];              /* RCC reserved register, 										Address offset: 0x78-7C */
	volatile uint32 SSCGR;			    /* RCC spread spectrum clock generation register, 				Address offset: 0x80 	*/
	volatile uint32 PLLI2SCFGR;		    /* RCC PLLI2S configuration register, 							Address offset: 0x84 	*/
	volatile uint32 PLLSAICFGR;		    /* RCC PLL configuration register, 								Address offset: 0x88 	*/
	volatile uint32 DCKCFGR;			/* RCC dedicated clock configuration register, 					Address offset: 0x8C 	*/
	volatile uint32 CKGATENR;			/* RCC clock gated enable register, 							Address offset: 0x90 	*/
}RCC_Registers_t;


/* ================================================================ */
/* =================       NVIC Registers      ==================== */
/* ================================================================ */

#define NVIC_BaseAdress			(uint32)(0xE000E100)	// NVIC Base Address
typedef struct
{
	volatile uint32 ISER[8];		// Interrupt set-enable register
	volatile uint32 Reserved1[24];
	volatile uint32 ICER[8];		// Interrupt clear-enable register
	volatile uint32 Reserved2[24];
	volatile uint32 ISPR[8];		// Interrupt set-pending register
	volatile uint32 Reserved3[24];
	volatile uint32 ICPR[8];		// Interrupt clear-pending register
	volatile uint32 Reserved4[24];
	volatile uint32 IABR[8];		// Interrupt active bit register
	volatile uint32 Reserved5[56];
	volatile uint32 IPR[240];		// Interrupt priority register
	volatile uint32 Reserved6[400];
	volatile uint32 STIR;			// Software trigger interrupt register
}NVIC_MemoryMap;




#define SCB_BaseAdress			(uint32)(0xE000ED00)	// SBC Base Address Starting from CPUID Register


typedef struct
{
	uint32 CPUID;		/* CPUID base register*/
	uint32 ICSR;		/* Interrupt control and state register*/
	uint32 VTOR;		/* Vector table offset register*/
	uint32 AIRCR;		/* -> Application interrupt and reset control register <-*/
	uint32 SCR;			/* System control register*/
	uint32	CCR;		/* Configuration and control register*/
	uint32 SHPR1;		/* -> System handler priority registers 1 <-*/
	uint32 SHPR2;		/* -> System handler priority registers 2 <-*/
	uint32 SHPR3;		/* -> System handler priority registers 3 <-*/
	uint32 SHCSR;		/* System handler control and state register*/
	uint32 CFSR;		/* Configurable fault status register (CFSR; UFSR + BFSR + MMFSR)*/
	uint32 HFSR;		/* Hard fault status register*/
	uint32 Reserved;
	uint32 MMAR;		/* Memory management fault address register*/
	uint32 BFAR;		/* Bus fault address register*/
	uint32 AFSR;		/* Auxiliary fault status register*/
} SCB_MemoryMap;


/* ================================================================ */
/* ================= Peripheral Registers GPIO ==================== */
/* ================================================================ */

typedef struct
{
	uint32 MODER;  		/* GPIO port mode register, 	    			Address offset: 0x00 */
	uint32 OTYPER; 		/* GPIO port output type register,  			Address offset: 0x04 */
	uint32 OSPEEDR;  	/* GPIO port output speed register, 			Address offset: 0x08 */
	uint32 PUPDR; 		/* GPIO port pull-up/down register, 			Address offset: 0x0C */
	uint32 IDR; 		/* GPIO port input data register,    			Address offset: 0x10 */
	uint32 ODR; 		/* GPIO port output data register, 	 			Address offset: 0x14 */
	uint32 BSRR; 		/* GPIO port bit set/reset register, 			Address offset: 0x18 */
	uint32 LCKR; 		/* GPIO port configuration lock register,		Address offset: 0x1C */
	uint32 AFR[2];		/* AFR[0]: GPIO alternate function low register,
						   AFR[1]: GPIO alternate function high register,	Address offset: 0x20-24 */
}GPIO_Registers_t;

/* ================================================================ */
/* =============== SysTick Peripheral Registers =================== */
/* ================================================================ */

typedef struct
{
    uint32 CSR;      /* SysTick Control and Status Register,         Address offset: 0x00 */
    uint32 RVR;      /* SysTick Reload Value Register,               Address offset: 0x04 */
    uint32 CVR;      /* SysTick Current Value Register,              Address offset: 0x08 */
    uint32 CALIB;    /* SysTick Calibration Value Register,          Address offset: 0x0C */

}Systick_Registers_t;



/* ================================================================ */
/* =================  Flash interface registe  ==================== */
/* ================================================================ */



typedef struct
{
	uint32 ACR;
	uint32 KEYR;
	uint32 OPTKEYR;
	uint32 SR;
	uint32 CR;
	uint32 OPTCR;
} FLASH_Registers_t;



#define FLASH_INTERFACE_BASE_ADDRESS 		0x40023C00


/* ================================================================ */
typedef enum
{
	RCC_CR_HSION         =0,
	RCC_CR_HSIRDY        =1,
	RCC_CR_HSITRIM_0     =3,
	RCC_CR_HSITRIM_1     =4,
	RCC_CR_HSITRIM_2     =5,
	RCC_CR_HSITRIM_3     =6,
	RCC_CR_HSITRIM_4     =7,
	RCC_CR_HSICAL_0      =8,
	RCC_CR_HSICAL_1      =9,
	RCC_CR_HSICAL_2      =10,
	RCC_CR_HSICAL_3      =11,
	RCC_CR_HSICAL_4      =12,
	RCC_CR_HSICAL_5      =13,
	RCC_CR_HSICAL_6      =14,
	RCC_CR_HSICAL_7      =15,
	RCC_CR_HSEON  	     =16,
	RCC_CR_HSERDY        =17,
	RCC_CR_HSEBYP        =18,
	RCC_CR_CSSON  	     =19,
	RCC_CR_PLLON         =24,
	RCC_CR_PLLRDY	     =25,
    RCC_CR_PLLI2SON      =26,
	RCC_CR_PLLI2SRDY	 =27,
    RCC_CR_PLLSAION      =28,
	RCC_CR_PLLSAIRDY	 =29,
}RCC_CR_bits_t;
/*RCC_CFGR bits*/
typedef enum
{
	RCC_PLLLCFGR_PLLM_0	  =0,
	RCC_PLLLCFGR_PLLM_1	  =1,
	RCC_PLLLCFGR_PLLM_2	  =2,
	RCC_PLLLCFGR_PLLM_3	  =3,
	RCC_PLLLCFGR_PLLM_4	  =4,
	RCC_PLLLCFGR_PLLM_5	  =5,
	RCC_PLLLCFGR_PLLN_0	  =6,
	RCC_PLLLCFGR_PLLN_1	  =7,
	RCC_PLLLCFGR_PLLN_2   =8,
	RCC_PLLLCFGR_PLLN_3   =9,
	RCC_PLLLCFGR_PLLN_4   =10,
	RCC_PLLLCFGR_PLLN_5   =11,
	RCC_PLLLCFGR_PLLN_6   =12,
	RCC_PLLLCFGR_PLLN_7   =13,
	RCC_PLLLCFGR_PLLN_8   =14,
	RCC_PLLLCFGR_PLLP_0   =16,
	RCC_PLLLCFGR_PLLP_1   =17,
	RCC_PLLLCFGR_PLLSRC   =22,
	RCC_PLLLCFGR_PLLQ_0   =24,
	RCC_PLLLCFGR_PLLQ_1   =25,
	RCC_PLLLCFGR_PLLQ_2   =26,
	RCC_PLLLCFGR_PLLQ_3   =27,
	RCC_PLLLCFGR_USBPRE	  =22,
	RCC_PLLLCFGR_PLLR_0	  =28,
	RCC_PLLLCFGR_PLLR_1	  =29,
	RCC_PLLLCFGR_PLLR_2	  =30,
}RCC_PLLLCFGR_bits_t;
/*RCC_CFGR bits*/
typedef enum
{
	RCC_CFGR_SW_0	   =0,
	RCC_CFGR_SW_1	   =1,
	RCC_CFGR_SWS_0	   =2,
	RCC_CFGR_SWS_1	   =3,
	RCC_CFGR_HPRE_0	   =4,
	RCC_CFGR_HPRE_1	   =5,
	RCC_CFGR_HPRE_2	   =6,
	RCC_CFGR_HPRE_3	   =7,
	RCC_CFGR_PPRE1_0   =10,
	RCC_CFGR_PPRE1_1   =11,
	RCC_CFGR_PPRE1_2   =12,
	RCC_CFGR_PPRE2_0   =13,
	RCC_CFGR_PPRE2_1   =14,
	RCC_CFGR_PPRE2_2   =15,
	RCC_CFGR_RTCPRE_0  =16,
	RCC_CFGR_RTCPRE_1  =17,
    RCC_CFGR_RTCPRE_2  =18,
	RCC_CFGR_RTCPRE_3  =19,
    RCC_CFGR_RTCPRE_4  =20,
	RCC_CFGR_MCO1_0    =21,
	RCC_CFGR_MCO1_1    =22,
	RCC_CFGR_MCO1PRE_0 =24,
	RCC_CFGR_MCO1PRE_1 =25,
	RCC_CFGR_MCO1PRE_2 =26,
    RCC_CFGR_MCO2PRE_0 =27,
	RCC_CFGR_MCO2PRE_1 =28,
	RCC_CFGR_MCO2PRE_2 =29,
	RCC_CFGR_MCO2_0	   =30,
	RCC_CFGR_MCO2_1	   =31,
}RCC_CFGR_bits_t;


/* ================================================================ */
/* ===================         SPI          ======================= */
/* ================================================================ */
typedef struct{
    volatile uint32 CR1;
    volatile uint32 CR2;
    volatile uint32 SR;
    volatile uint32 DR;
    volatile uint32 CRCPR;
    volatile uint32 RXCRCR;
    volatile uint32 TXCRCR;
} SPI_Registers;

/* ================================================================ */
/* =================== Peripheral Instants  ======================= */
/* ================================================================ */

#define GPIOA 			((GPIO_Registers_t *) GPIOA_BASE)
#define GPIOB 			((GPIO_Registers_t *) GPIOB_BASE)
#define GPIOC 			((GPIO_Registers_t *) GPIOC_BASE)
#define GPIOD			((GPIO_Registers_t *) GPIOD_BASE)
#define GPIOE 			((GPIO_Registers_t *) GPIOE_BASE)
#define GPIOF 			((GPIO_Registers_t *) GPIOF_BASE)
#define GPIOG 			((GPIO_Registers_t *) GPIOG_BASE)
#define GPIOH 			((GPIO_Registers_t *) GPIOH_BASE)

#define RCC				((RCC_Registers_t *) RCC_BASE)

#define SPI1            ((SPI_Registers*)SPI1_BASE)
#define SPI2            ((SPI_Registers*)SPI2_BASE)
#define SPI3            ((SPI_Registers*)SPI3_BASE)
#define SPI4            ((SPI_Registers*)SPI4_BASE)

#define Systick         ((Systick_Registers_t*)SYSTICK_BASE)

#define	FMI				((FLASH_Registers_t *)(FLASH_INTERFACE_BASE_ADDRESS))

#define	SCB				( (SCB_MemoryMap *)(SCB_BaseAdress) )
#define	NVIC			( (NVIC_MemoryMap *)(NVIC_BaseAdress) )


/* ================================================================ */
/* ================================================================ */
/* ================================================================ */
/* ================================================================ */
/* ================================================================ */

/* ================================================================ */
/* ==============          Clock Enable           ================= */
/* ================================================================ */

/* ============ GPIOx peripherals ============ */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) ) /* GPIOA peripheral clock enabled */
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) ) /* GPIOB peripheral clock enabled */
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) ) /* GPIOC peripheral clock enabled */
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) ) /* GPIOD peripheral clock enabled */
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) ) /* GPIOE peripheral clock enabled */
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) ) /* GPIOF peripheral clock enabled */
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) ) /* GPIOG peripheral clock enabled */
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) ) /* GPIOH peripheral clock enabled */

#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI1EN))
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI4EN))
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI2EN))
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI3EN))



/* ================================================================ */
/* ================================================================ */
/* ================================================================ */
/* ================================================================ */
/* ================================================================ */
/* ================================================================ */

/*********************************************/
/*              SPI_CR1 Bits                 */
/*********************************************/
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR0         3
#define SPI_CR1_BR1         4
#define SPI_CR1_BR2         5
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBF        7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNXT      12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

/*********************************************/
/*              SPI_CR2 Bits                 */
/*********************************************/
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2

#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7


/*********************************************/
/*              SPI_SR Bits                 */
/*********************************************/
#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8


#endif /* STM32F446XX_H_ */
