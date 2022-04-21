/*
 * stm32f3xx_cortex.c
 *
 *  Created on: Apr 3, 2022
 *      Author: Enes
 */

#include "stm32f3xx_cortex.h"


/*********************************************************************
 * @fn      		  - NVIC_SetPriority
 *
 * @brief             -
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	uint32_t
 * @param[in]         - uint32_t
 *
 * @return            - none
 *
 * @Note              - example -> NVIC_SetPriority(EXTI0_IRQn, 0, 0);
 * 						example -> NVIC_SetPriority(USART2_IRQn, 1, 0);
 */
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
	  uint32_t prioritygroup = 0x00U;

	  prioritygroup = __NVIC_GetPriorityGrouping();

	  __NVIC_SetPriority(IRQn, __NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}


/*********************************************************************
 * @fn      		  - NVIC_ClearPendingIRQ
 *
 * @brief             - main function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - example -> NVIC_ClearPendingIRQ(EXTI0_IRQn);
 *
 */
void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  /* Clear pending interrupt */
  __NVIC_ClearPendingIRQ(IRQn);
}



/*********************************************************************
 * @fn      		  - NVIC_EnableIRQ
 *
 * @brief             - main function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - example -> NVIC_EnableIRQ(EXTI0_IRQn);
 *
 */
void NVIC_EnableIRQ(IRQn_Type IRQn)
{
	__NVIC_EnableIRQ(IRQn);
}

/*********************************************************************
 * @fn      		  - NVIC_DisableIRQ
 *
 * @brief             - main function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - example -> NVIC_DisableIRQ(EXTI0_IRQn);
 *
 */
void NVIC_DisableIRQ(IRQn_Type IRQn)
{
	__NVIC_DisableIRQ(IRQn);
}

/*********************************************************************
 * @fn      		  - __NVIC_GetPriorityGrouping
 *
 * @brief             - alt function
 *
 * @param[in]         - none
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - uint32_t
 *
 * @Note              - none
 *
 */
uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & (7UL<<8U)) >> 8U));
}

/*********************************************************************
 * @fn      		  - __NVIC_SetPriority
 *
 * @brief             - alt function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	uint32_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
 if ((int32_t)(IRQn) >= 0)
 {
   NVIC->IP[((uint32_t)IRQn)]= (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
 }
 else
 {
   SCB->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
 }
}


/*********************************************************************
 * @fn      		  - __NVIC_EncodePriorityy
 *
 * @brief             - alt function
 *
 * @param[in]         - uint32_t
 * @param[in]         -	uint32_t
 * @param[in]         - uint32_t
 *
 * @return            - uint32_t
 *
 * @Note              - none
 *
 */
uint32_t __NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}


/*********************************************************************
 * @fn      		  - __NVIC_EnableIRQ
 *
 * @brief             - alt function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}



/*********************************************************************
 * @fn      		  -__NVIC_ClearPendingIRQ
 *
 * @brief             - alt function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

/*********************************************************************
 * @fn      		  - __NVIC_DisableIRQ
 *
 * @brief             - alt function
 *
 * @param[in]         - IRQn_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));

  }
}
