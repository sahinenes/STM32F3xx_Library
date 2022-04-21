/*
 * stm32f3xx_cortex.h
 *
 *  Created on: Apr 3, 2022
 *      Author: Enes
 */



#ifndef INC_STM32F3XX_CORTEX_H_
#define INC_STM32F3XX_CORTEX_H_

#include "stm32f3xx.h"


/**
 * @brief STM32F303xE devices Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line 19          */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line 20                     */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_TSC_IRQn              = 8,      /*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt         */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 Interrupt                                          */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 Interrupt                                          */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 Interrupt                                          */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 Interrupt                                          */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 Interrupt                                          */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 Interrupt                                          */
  ADC1_2_IRQn                 = 18,     /*!< ADC1 & ADC2 Interrupts                                            */
  USB_HP_CAN_TX_IRQn          = 19,     /*!< USB Device High Priority or CAN TX Interrupts                     */
  USB_LP_CAN_RX0_IRQn         = 20,     /*!< USB Device Low Priority or CAN RX0 Interrupts                     */
  CAN_RX1_IRQn                = 21,     /*!< CAN RX1 Interrupt                                                 */
  CAN_SCE_IRQn                = 22,     /*!< CAN SCE Interrupt                                                 */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)        */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)        */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)   */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)   */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)   */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt                 */
  USBWakeUp_IRQn              = 42,     /*!< USB Wakeup Interrupt                                              */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                             */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt & EXTI Line34 Interrupt (UART4 wakeup)     */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt & EXTI Line35 Interrupt (UART5 wakeup)     */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC underrun error Interrupt             */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  ADC4_IRQn                   = 61,     /*!< ADC4  global Interrupt                                            */
  COMP1_2_3_IRQn              = 64,     /*!< COMP1, COMP2 and COMP3 global Interrupt via EXTI Line21, 22 and 29*/
  COMP4_5_6_IRQn              = 65,     /*!< COMP4, COMP5 and COMP6 global Interrupt via EXTI Line30, 31 and 32*/
  COMP7_IRQn                  = 66,     /*!< COMP7 global Interrupt via EXTI Line33                            */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 Error Interrupt                                              */
  USB_HP_IRQn                 = 74,     /*!< USB High Priority global Interrupt                                */
  USB_LP_IRQn                 = 75,     /*!< USB Low Priority global Interrupt                                 */
  USBWakeUp_RMP_IRQn          = 76,     /*!< USB Wakeup Interrupt remap                                        */
  TIM20_BRK_IRQn              = 77,     /*!< TIM20 Break Interrupt                                              */
  TIM20_UP_IRQn               = 78,     /*!< TIM20 Update Interrupt                                             */
  TIM20_TRG_COM_IRQn          = 79,     /*!< TIM20 Trigger and Commutation Interrupt                            */
  TIM20_CC_IRQn               = 80,     /*!< TIM20 Capture Compare Interrupt                                    */
  FPU_IRQn                    = 81,      /*!< Floating point Interrupt                                          */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
} IRQn_Type;


 /* NVIC Functions*/
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void NVIC_EnableIRQ(IRQn_Type IRQn);
void NVIC_DisableIRQ(IRQn_Type IRQn);
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);

/* NVIC Helper */
void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);
uint32_t __NVIC_GetPriorityGrouping(void);
uint32_t __NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority);
void __NVIC_EnableIRQ(IRQn_Type IRQn);
void __NVIC_ClearPendingIRQ(IRQn_Type IRQn);
void __NVIC_DisableIRQ(IRQn_Type IRQn);
#endif /* INC_STM32F3XX_CORTEX_H_ */
