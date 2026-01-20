/*
 * stm32f407_driver.c
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){ // pGPIOHandle is given by the user
	uint32_t temp =0;

	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// non interrupt mode
		//1. config the mode of gpio pin
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits field
		pGPIOHandle -> pGPIOx -> MODER |= temp ; // setting of the bits fields
		temp =0;
		//2. config the speed
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits field
		pGPIOHandle -> pGPIOx -> OSPEEDR |= temp ;
		temp =0;
		//3. config the pupd setting
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits field
		pGPIOHandle -> pGPIOx -> PUPDR |= temp ;
		temp =0;
		//4. config the optype
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_OPType << (1*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER  &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits field
		pGPIOHandle -> pGPIOx -> OTYPER |= temp ;
		temp =0;
	}
	else{
		// code later for interrupt
	}

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
 * @fn           - GPIO_IRQCOnfig
 * @brief        - interupt request will be configured her , when to deal  with which interrupt and how much is her priority
 * @param[in]	 - IRQ number will be needed
 * @param[in]	 - IRQ priority will be set here
 * @param[in]	 - enabling or disabling the interupt
 *
 * @return 		-
 * @Note    	-
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDi){

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
void GPIO_IRQHandling(uint8_t PinNumber); // require only the pin number to handle the intrerrupt


