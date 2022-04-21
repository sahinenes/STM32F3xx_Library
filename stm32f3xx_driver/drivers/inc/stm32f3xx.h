/*
 * stm32f3xx.h
 *
 *  Created on: Feb 27, 2022
 *      Author: Enes
 *
 *
 *      https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf
 */






#ifndef INC_STM3F3XX_H_
#define INC_STM3F3XX_H_

#include <stddef.h>
#include <stdint.h>

/*Generic*/
#define ENABLE 1
#define DISABLE 0
#define HIGH  ENABLE
#define LOW   DISABLE
#define SET	  ENABLE
#define RESET DISABLE

#define _vo volatile
#define __weak __attribute__((weak))

#define FLASH_BASEADDR 		0x08000000U
#define SRAM_BASEADDR  		0x20000000U
#define ROM_BASEADDR   		0x1FFFD800U
#define SRAM_CCM_BASEADDR 	0x10000000U

/*BASE ADRESS*/
#define PERIPHERAL_BASE     0x40000000U
#define APB1_BASE			PERIPHERAL_BASE
#define APB2_BASE			0x40010000U
#define AHB1_BASE			0x40020000U
#define AHB2_BASE			0x48000000U
#define AHB3_BASE			0x50000000U
#define AHB4_BASE			0x60000000U


#define SCS_BASE            (0xE000E000UL)/*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)/*!< System Control Block Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)/*!< NVIC Base Address */
/*
 * https://developer.arm.com/documentation/ddi0439/b/Introduction
 */

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (_vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (_vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (_vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (_vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((_vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((_vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((_vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((_vo uint32_t*)0xE000E400)


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI1    1
#define NVIC_IRQ_PRI2    2
#define NVIC_IRQ_PRI3    3
#define NVIC_IRQ_PRI4    4
#define NVIC_IRQ_PRI5    5
#define NVIC_IRQ_PRI6    6
#define NVIC_IRQ_PRI7    7
#define NVIC_IRQ_PRI8    8
#define NVIC_IRQ_PRI9    9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/****************************************************************************/

/*APB1*/
#define TIM2_BASEADDR		APB1_BASE
#define TIM3_BASEADDR		(APB1_BASE + 0x0400)
#define TIM4_BASEADDR		(APB1_BASE + 0x0800)
#define TIM6_BASEADDR		(APB1_BASE + 0x1000)
#define TIM7_BASEADDR		(APB1_BASE + 0x1400)
#define RTC_BASEADDR		(APB1_BASE + 0x2800)
#define WWDG_BASEADDR		(APB1_BASE + 0x2C00)
#define IWDG_BASEADDR		(APB1_BASE + 0x3000)
#define I2S2ext_BASEADDR	(APB1_BASE + 0x3400)
#define SPI2_I2S2_BASEADDR	(APB1_BASE + 0x3800)
#define SPI3_I2S3_BASEADDR	(APB1_BASE + 0x3C00)
#define I2S3ext_BASEADDR	(APB1_BASE + 0x4000)
#define USART2_BASEADDR		(APB1_BASE + 0x4400)
#define USART3_BASEADDR		(APB1_BASE + 0x4800)
#define UART4_BASEADDR		(APB1_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1_BASE + 0x5000)
#define I2C1_BASEADDR		(APB1_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1_BASE + 0x5800)
#define USB_FS_BASEADDR		(APB1_BASE + 0x5C00)
#define USB_CAN_SRAM_BASEADDR (APB1_BASE + 0x6000)
#define bxCAN_BASEADDR		(APB1_BASE + 0x6400)
#define PWR_BASEADDR		(APB1_BASE + 0x7000)
#define DAC_BASEADDR		(APB1_BASE + 0x7400)
#define I2C3_BASEADDR		(APB1_BASE + 0x7800)

/*APB2*/
#define SYSCFG_COMP_OPAMP_BASEADDR	APB2_BASE
#define EXTI_BASEADDR	    (APB2_BASE + 0x0400)
#define TIM1_BASEADDR	    (APB2_BASE + 0x2C00)
#define SPI1_BASEADDR	    (APB2_BASE + 0x3000)
#define TIM8_BASEADDR	    (APB2_BASE + 0x3400)
#define USART1_BASEADDR	    (APB2_BASE + 0x3800)
#define SPI4_BASEADDR	    (APB2_BASE + 0x3C00)
#define TIM15_BASEADDR	    (APB2_BASE + 0x4000)
#define TIM16_BASEADDR	    (APB2_BASE + 0x4400)
#define TIM17_BASEADDR	    (APB2_BASE + 0x4800)
#define TIM20_BASEADDR	    (APB2_BASE + 0x5000)

/*AHB1*/
#define DMA1_BASEADDR	    AHB1_BASE
#define DMA2_BASEADDR	   	(AHB1_BASE + 0x0400)
#define RCC_BASEADDR	   	(AHB1_BASE + 0x1000)
#define Flash_interface_BASEADDR (AHB1_BASE + 0x2000)
#define CRC_BASEADDR	   	(AHB1_BASE + 0x3000)
#define TSC_BASEADDR	   	(AHB1_BASE + 0x4000)

/*AHB2*/
#define GPIOA_BASEADDR		AHB2_BASE
#define GPIOB_BASEADDR		(AHB2_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB2_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB2_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB2_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB2_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB2_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB2_BASE + 0x1C00)

/*AHB3*/
#define ADC1_ADC2_BASEADDR	AHB3_BASE
#define ADC3_ADC4_BASEADDR	(AHB3_BASE + 0x0400)

/*AHB4*/
#define FMC_banks_1_2_BASEADDR	AHB4_BASE
#define FMC_banks_3_4_BASEADDR	0x80000400U
#define FMC_control_BASEADDR	0xA0000400U

/****************************************************************************/

/*structures*/

typedef struct {
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];
	_vo uint32_t BRR;

} GPIO_Type;

typedef struct {
	_vo uint32_t CR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t APB2RSTR;
	_vo uint32_t APB1RSTR;
	_vo uint32_t AHBENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t APB1ENR;
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	_vo uint32_t AHBRSTR;
	_vo uint32_t CFGR2;
	_vo uint32_t CFGR3;

} RCC_RegDef_t;


typedef struct
{

	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;


}SPI_Type;



typedef struct
{

	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t OAR1;
	_vo uint32_t OAR2;
	_vo uint32_t TIMINGR;
	_vo uint32_t TIMEOUTR;
	_vo uint32_t ISR;
	_vo uint32_t ICR;
	_vo uint32_t PECR;
	_vo uint32_t RXDR;
	_vo uint32_t TXDR;


}I2C_Type;

typedef struct
{
	_vo uint32_t IMR1;
	_vo uint32_t EMR1;
	_vo uint32_t RTSR1;
	_vo uint32_t FTSR1;
	_vo uint32_t SWIER1;
	_vo uint32_t PR1;
	_vo uint32_t IMR2;
	_vo uint32_t EMR2;
	_vo uint32_t RTSR2;
	_vo uint32_t FTSR2;
	_vo uint32_t SWIER2;
	_vo uint32_t PR2;

}EXTI_RegDef_t;

typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t CR3;
	_vo uint32_t BRR;
	_vo uint32_t GTPR;
	_vo uint32_t RTOR;
	_vo uint32_t RQR;
	_vo uint32_t ISR;
	_vo uint32_t ICR;
	_vo uint32_t RDR;
	_vo uint32_t TDR;

}USART_Type;

typedef struct
{
	_vo uint32_t CFGR1;
	_vo uint32_t RCR;
	_vo uint32_t EXTICR[4];
	_vo uint32_t CFGR2;
	uint32_t     RESERVED2[14];
	_vo uint32_t CFGR3;
} SYSCFG_RegDef_t;


typedef struct
{
  _vo const  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  _vo uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  _vo uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  _vo uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  _vo uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  _vo uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  _vo uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  _vo uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  _vo uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  _vo uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  _vo uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  _vo uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  _vo uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  _vo uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  _vo const  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  _vo const  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  _vo const  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  _vo const  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  _vo const  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;


typedef struct
{
  _vo uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  _vo uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  _vo uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  _vo uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  _vo uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  _vo uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  _vo  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;
/****************************************************************************/

/*definitions*/

/*GPIO*/
#define GPIOA  ((GPIO_Type*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_Type*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_Type*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_Type*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_Type*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_Type*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_Type*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_Type*)GPIOH_BASEADDR)

/*SPI*/
#define SPI1	((SPI_Type*)SPI1_BASEADDR)
#define SPI2	((SPI_Type*)SPI2_I2S2_BASEADDR)
#define SPI3	((SPI_Type*)SPI3_I2S3_BASEADDR)
#define SPI4	((SPI_Type*)SPI4_BASEADDR)


#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_COMP_OPAMP_BASEADDR)


#define	I2C1	((I2C_Type*)I2C1_BASEADDR)
#define	I2C2	((I2C_Type*)I2C2_BASEADDR)
#define	I2C3	((I2C_Type*)I2C3_BASEADDR)


#define	USART1	((USART_Type*)USART1_BASEADDR)
#define	USART2	((USART_Type*)USART2_BASEADDR)
#define	USART3	((USART_Type*)USART3_BASEADDR)
#define	UART4	((USART_Type*)UART4_BASEADDR)
#define	UART5	((USART_Type*)UART5_BASEADDR)

#define SCB     ((SCB_Type*)SCB_BASE)   /*!< SCB configuration struct */
#define NVIC    ((NVIC_Type*)NVIC_BASE)   /*!< NVIC configuration struct */
/****************************************************************************/

/*Clock enable*/

/*AHBENR*/
#define DMA1_PCLCK_EN()  (RCC->AHBENR |= (1<<0))
#define DMA2_PCLCK_EN()  (RCC->AHBENR |= (1<<1))
#define SRAM_PCLCK_EN()  (RCC->AHBENR |= (1<<2))
#define FLITF_PCLCK_EN() (RCC->AHBENR |= (1<<4))
#define FMC_PCLCK_EN()   (RCC->AHBENR |= (1<<5))
#define CRC_PCLCK_EN()   (RCC->AHBENR |= (1<<6))
#define IOPH_PCLCK_EN()  (RCC->AHBENR |= (1<<16))
#define IOPA_PCLCK_EN()  (RCC->AHBENR |= (1<<17))
#define IOPB_PCLCK_EN()  (RCC->AHBENR |= (1<<18))
#define IOPC_PCLCK_EN()  (RCC->AHBENR |= (1<<19))
#define IOPD_PCLCK_EN()  (RCC->AHBENR |= (1<<20))
#define IOPE_PCLCK_EN()  (RCC->AHBENR |= (1<<21))
#define IOPF_PCLCK_EN()  (RCC->AHBENR |= (1<<22))
#define IOPG_PCLCK_EN()  (RCC->AHBENR |= (1<<23))
#define TSC_PCLCK_EN()   (RCC->AHBENR |= (1<<24))
#define ADC12_PCLCK_EN() (RCC->AHBENR |= (1<<28))
#define ADC34_PCLCK_EN() (RCC->AHBENR |= (1<<29))

/*APB2ENR*/
#define SYSCFG_PCLCK_EN()  (RCC->APB2ENR |= (1<<0))
#define TIM1_PCLCK_EN()    (RCC->APB2ENR |= (1<<11))
#define SPI1_PCLCK_EN()    (RCC->APB2ENR |= (1<<12))
#define TIM8_PCLCK_EN()    (RCC->APB2ENR |= (1<<13))
#define USART1_PCLCK_EN()  (RCC->APB2ENR |= (1<<14))
#define SPI4_PCLCK_EN()    (RCC->APB2ENR |= (1<<15))
#define TIM15_PCLCK_EN()   (RCC->APB2ENR |= (1<<16))
#define TIM16_PCLCK_EN()   (RCC->APB2ENR |= (1<<17))
#define TIM17_PCLCK_EN()   (RCC->APB2ENR |= (1<<18))
#define TIM20_PCLCK_EN()   (RCC->APB2ENR |= (1<<20))

/*APB1ENR*/
#define TIM2_PCLCK_EN()    (RCC->APB1ENR |= (1<<0))
#define TIM3_PCLCK_EN()    (RCC->APB1ENR |= (1<<1))
#define TIM4_PCLCK_EN()    (RCC->APB1ENR |= (1<<2))
#define TIM6_PCLCK_EN()    (RCC->APB1ENR |= (1<<4))
#define TIM7_PCLCK_EN()    (RCC->APB1ENR |= (1<<5))
#define WWDG_PCLCK_EN()    (RCC->APB1ENR |= (1<<11))
#define SPI2_PCLCK_EN()    (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLCK_EN()    (RCC->APB1ENR |= (1<<15))
#define USART2_PCLCK_EN()  (RCC->APB1ENR |= (1<<17))
#define USART3_PCLCK_EN()  (RCC->APB1ENR |= (1<<18))
#define UART4_PCLCK_EN()   (RCC->APB1ENR |= (1<<19))
#define UART5_PCLCK_EN()   (RCC->APB1ENR |= (1<<20))
#define I2C1_PCLCK_EN()    (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLCK_EN()    (RCC->APB1ENR |= (1<<22))
#define USB_PCLCK_EN()     (RCC->APB1ENR |= (1<<23))
#define CAN_PCLCK_EN()     (RCC->APB1ENR |= (1<<25))
#define DAC2_PCLCK_EN()    (RCC->APB1ENR |= (1<<26))
#define PWR_PCLCK_EN()     (RCC->APB1ENR |= (1<<28))
#define DAC1_PCLCK_EN()    (RCC->APB1ENR |= (1<<29))
#define I2C3_PCLCK_EN()    (RCC->APB1ENR |= (1<<30))


/*Clock disable*/
#define DMA1_PCLCK_DIS()  (RCC->AHBENR |= (1<<0))
#define DMA2_PCLCK_DIS()  (RCC->AHBENR |= (1<<1))
#define SRAM_PCLCK_DIS()  (RCC->AHBENR |= (1<<2))
#define FLITF_PCLCK_DIS() (RCC->AHBENR |= (1<<4))
#define FMC_PCLCK_DIS()   (RCC->AHBENR |= (1<<5))
#define CRC_PCLCK_DIS()   (RCC->AHBENR |= (1<<6))
#define IOPH_PCLCK_DIS()  (RCC->AHBENR |= (1<<16))
#define IOPA_PCLCK_DIS()  (RCC->AHBENR |= (1<<17))
#define IOPB_PCLCK_DIS()  (RCC->AHBENR |= (1<<18))
#define IOPC_PCLCK_DIS()  (RCC->AHBENR |= (1<<19))
#define IOPD_PCLCK_DIS()  (RCC->AHBENR |= (1<<20))
#define IOPE_PCLCK_DIS()  (RCC->AHBENR |= (1<<21))
#define IOPF_PCLCK_DIS()  (RCC->AHBENR |= (1<<22))
#define IOPG_PCLCK_DIS()  (RCC->AHBENR |= (1<<23))
#define TSC_PCLCK_DIS()   (RCC->AHBENR |= (1<<24))
#define ADC12_PCLCK_DIS() (RCC->AHBENR |= (1<<28))
#define ADC34_PCLCK_DIS() (RCC->AHBENR |= (1<<29))

/*APB2ENR*/
#define SYSCFG_PCLCK_DIS()  (RCC->APB2ENR |= (1<<0))
#define TIM1_PCLCK_DIS()    (RCC->APB2ENR |= (1<<11))
#define SPI1_PCLCK_DIS()    (RCC->APB2ENR |= (1<<12))
#define TIM8_PCLCK_DIS()    (RCC->APB2ENR |= (1<<13))
#define USART1_PCLCK_DIS()  (RCC->APB2ENR |= (1<<14))
#define SPI4_PCLCK_DIS()    (RCC->APB2ENR |= (1<<15))
#define TIM15_PCLCK_DIS()   (RCC->APB2ENR |= (1<<16))
#define TIM16_PCLCK_DIS()   (RCC->APB2ENR |= (1<<17))
#define TIM17_PCLCK_DIS()   (RCC->APB2ENR |= (1<<18))
#define TIM20_PCLCK_DIS()   (RCC->APB2ENR |= (1<<20))

/*APB1ENR*/
#define TIM2_PCLCK_DIS()    (RCC->APB1ENR |= (1<<0))
#define TIM3_PCLCK_DIS()    (RCC->APB1ENR |= (1<<1))
#define TIM4_PCLCK_DIS()    (RCC->APB1ENR |= (1<<2))
#define TIM6_PCLCK_DIS()    (RCC->APB1ENR |= (1<<4))
#define TIM7_PCLCK_DIS()    (RCC->APB1ENR |= (1<<5))
#define WWDG_PCLCK_DIS()    (RCC->APB1ENR |= (1<<11))
#define SPI2_PCLCK_DIS()    (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLCK_DIS()    (RCC->APB1ENR |= (1<<15))
#define USART2_PCLCK_DIS()  (RCC->APB1ENR |= (1<<17))
#define USART3_PCLCK_DIS()  (RCC->APB1ENR |= (1<<18))
#define UART4_PCLCK_DIS()   (RCC->APB1ENR |= (1<<19))
#define UART5_PCLCK_DIS()   (RCC->APB1ENR |= (1<<20))
#define I2C1_PCLCK_DIS()    (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLCK_DIS()    (RCC->APB1ENR |= (1<<22))
#define USB_PCLCK_DIS()     (RCC->APB1ENR |= (1<<23))
#define CAN_PCLCK_DIS()     (RCC->APB1ENR |= (1<<25))
#define DAC2_PCLCK_DIS()    (RCC->APB1ENR |= (1<<26))
#define PWR_PCLCK_DIS()     (RCC->APB1ENR |= (1<<28))
#define DAC1_PCLCK_DIS()    (RCC->APB1ENR |= (1<<29))
#define I2C3_PCLCK_DIS()    (RCC->APB1ENR |= (1<<30))


/*other*/
/****************************************************************************/
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7:0)

/*gpio reg reset*/
#define GPIOA_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 0)); (RCC->AHBRSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 1)); (RCC->AHBRSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 2)); (RCC->AHBRSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 3)); (RCC->AHBRSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 4)); (RCC->AHBRSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 5)); (RCC->AHBRSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 6)); (RCC->AHBRSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 7)); (RCC->AHBRSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHBRSTR |= (1 << 8)); (RCC->AHBRSTR &= ~(1 << 8)); }while(0)

/*spi reg reset*/
#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->AHBRSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15)); }while(0)

/*i2c reg reset*/
#define I2C1_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 30)); (RCC->AHBRSTR &= ~(1 << 30)); }while(0)


#endif /* INC_STM3F3XX_H_*/
