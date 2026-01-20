/*
 *                                     stm32f407_driver.c
 *   GPIO DRIVER FILE
 *  Created on: 05-Jan-2026
 *      Author: debian
 */
#include "stm32f407_driver.h"




/*********************************************** Peripheral Clock Setup ****************************************************
 * @fn           - GPIO_periClockControl
 * @brief        - this function enable or disables peripheral clock for the GPIO
 * @param[in]	 - GPIO_RegDef_t *pGPIOx ,base addr of the gpio peripheral
 * @param[in]	 - EnOrDi , ENABle or disable macros
 * @param[in]	 -
 *
 * @return 		 - NONE
 * @Note    	 - NONE
 *
 */
void GPIO_periClockControl(GPIO_RegDef_t *pGPIOx,uint8_t ENOrDI){

	if(ENOrDI == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC ){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}

	}
	else{
		// disabling the gpio
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC ){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}

	}

}




/***************************************************** initialize and deinitialize the peripheral****************************
 * @fn           - GPIO_Init
 * @brief        - initialize the gpio peripherals , it is a structure basically defined in stm32f407 drvier.h header file
 * @param[in]	 - base addr of the gpio handler.
 * @param[in]	 -
 * @param[in]	 -
 *
 * @return 		- NONE
 * @Note    	- NONE
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	// enable the PERIPHERAL CLOCK FOR USER

	GPIO_periClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// pGPIOHandle is given by the user
	uint32_t temp =0; // temp temporarily holds the correctly shifted bit pattern for ONE pin before it is written into the hardware register.
	// pGPIOHandle give us the base address. to access the gpio

	//1. config the mode of gpio pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){  // if pin Mode is less than the 4 - analog mode then this work
		// non interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bits field
		pGPIOHandle -> pGPIOx -> MODER |= temp ; // setting of the bits fields //storing the data into the actual register
		temp =0;
	}
	else{
		// ****************************   code later for interrupt
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. config the FISR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. config the RTSR -- rising trigger shift reg
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode  == GPIO_MODE_IT_RFT){
			//1. config both FTST AND RTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. config the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4 ;// this give us EXTI register number
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4; // give us the positon of the exti where we have to set the value
		// macro
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2*4));

		// 3. enable the exti interrupt delivery using the IMR (interrupt mask register)
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber; // enabling the IMR COrresponding to IMR number

	}

	//2. config the speed
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bits field
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp ;
	temp =0;

	//3. config the pupd setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bits field
	pGPIOHandle -> pGPIOx -> PUPDR |= temp ;
	temp =0;

	//4. config the optype
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_OPType << (1*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER  &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bits field
	pGPIOHandle -> pGPIOx -> OTYPER |= temp ;
	temp =0;

	//5. config the alternate functionality
	if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		// config alt fn reg
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2)); // 4 bits are used therefore 0xf , clearing the bits field
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

	}


}




/*/*************************************************************************************************************************
 *
 * @fn           - GPIO_DeInit
 * @brief        - we will simply disable the rcc for the gpio pin
 * @param[in]	 - base addr of the gpio
 * @param[in]	 -
 * @param[in]	 -
 *
 * @return 		 - NONE
 * @Note    	 - NONE
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

			if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC ){
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE){
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG){
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH){
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI){
				GPIOI_REG_RESET();
			}

}




/*/********************************************  RD or WR to the peripheral  *************************************************
 * @fn           - GPIO_ReadFromInputPin
 * @brief        - READING FROM THE INPUT PINS OF THE GPIO when any interrupt occur through the peripherals
 * @param[in]	 - base addr of the GPIO from the regdef
 * @param[in]	 - PIN NUMBER
 * @param[in]	 -
 *
 * @return 	 	 -  0 or 1
 * @Note     	 - NONE
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	// name itself suggest we need the baseaddr or pinNumber

	uint8_t value;

	value = (uint8_t)( (pGPIOx->IDR >> PinNumber ) & 0x00000001) ;
	return value;
}





/*/***************************************************************************************************************************
 *
 * @fn           - GPIO_ReadFromInputPort
 * @brief        -  READING FROM THE WHOLE PORT
 * @param[in]	 - gpio base addr
 * @param[in]	 -
 * @param[in]	 -
 *
 * @return 		-  as the port is 16 bit therefore output is also 16 bit long
 * @Note    	- NONE
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR ) ;
	return value;
}




/*/***************************************************************************************************************************
 *
 * @fn           - GPIO_WriteToOutputPin
 * @brief        - writing to the output pin
 * @param[in]	 - base addr of the gpio
 * @param[in]	 - pin number
 * @param[in]	 - value define either set or reset the pin
 *
 * @return 		 -  as we are sending data therefore no need of return type
 * @Note    	 - NONE
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value){
	if(Value == GPIO_PIN_SET){
		// write 1 to output  data reg at bit field the corresponding pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		// write 0
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}








/*/***************************************************************************************************************************
 *
 * @fn           -  GPIO_WriteToOutputPort
 * @brief        - writing to the port either A,B, c or so on.
 * @param[in]	 - base addr of gpio
 * @param[in]	 - ENABLE OR DISABLE VALUE
 * @param[in]	 -
 *
 * @return 		- NONE
 * @Note    	- NONE
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){ // value is 16 bit bcz output is 16 bit long(port)
 // copy the value to the ODR register , bcz you are copying the whole port
	pGPIOx->ODR = Value;

}





/*/***************************************************************************************************************************
 *
 * @fn           - GPIO_ToggleOutputPin
 * @brief        - toggling of the gpio pins  state
 * @param[in]	 -  base addr of gpio
 * @param[in]	 - pin number
 * @param[in]	 -
 *
 * @return 		- NONE
 * @Note    	- NONE
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	// for toggling we need the gpio pin or port number
	pGPIOx->ODR ^= (1<<PinNumber);
}

// ------------------------------------------------------------  interrupt handling  ------------------------------------------

/*/***************************************************************************************************************************
 *
 * @fn           - GPIO_IRQInterruptConfig
 * @brief        - interupt request will be configured her , when to deal  with which interrupt and how much is her priority
 * @param[in]	 - IRQ number will be needed
 * @param[in]	 - enabling or disabling the interupt
 *
 * @return 		-
 * @Note    	-
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDi){
 // we are going to processor side
	// go to cortex M4 user manual( NVIC )
	if( EnOrDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER 0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64){
			// program ISER 1 register
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER 2 register
			*NVIC_ISER3 |= (1<<(IRQNumber%64));
		}
	}
	else{
		// ***************************** DISABLE *******************************

		if(IRQNumber <= 31){
					// program ISER 0 register
			*NVIC_ISER0 &= ~(1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64){
					// program ISER 1 register
			*NVIC_ISER1 &= ~(1<<(IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
					// program ISER 2 register
			*NVIC_ISER3 &= ~(1<<(IRQNumber%64));
		}
	}
}

/*/***************************************************************************************************************************
 *
 * @fn           - GPIO_IRQPriorityCOnfig
 * @brief        - interupt request will be configured her , when to deal  with which interrupt and how much is her priority
 * @param[in]	 - IRQ number will be needed
 * @param[in]    - Priority of the interrupt
 * @param[in]	 - enabling or disabling the interupt
 *
 * @return 		-
 * @Note    	-
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
 uint8_t iprx = IRQNumber /4;
 uint8_t iprx_section = IRQNumber % 4;
 uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
 *(NVIC_PR_BASE_ADDR + (4*iprx) )  |= (IRQPriority << shift_amount);

}




/*/**************************************************************************************************************************
 * @fn           -
 * @brief        -
 * @param[in]	 -
 * @param[in]	 -
 * @param[in]	 -
 *
 * @return 		-
 * @Note    	-
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber){ // require only the pin number to handle the intrerrupt
// clear the exti pr register
	if (EXTI->PR & (1<< PinNumber)){
		// clear
		EXTI->PR |= (1<<PinNumber);
	}
}
