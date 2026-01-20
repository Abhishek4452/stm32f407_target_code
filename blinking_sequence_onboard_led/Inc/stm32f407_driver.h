/*
 * stm32f407_driver.h
 *  *************************************************    GPIO driver file  *********************************************************************************
 *  Created on: 05-Jan-2026
 *      Author: debian
 */

#ifndef STM32F407_DRIVER_H_
#define STM32F407_DRIVER_H_

#include "stm32f407xx.h"

//***************************************************  API FOR THE USER ****************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//*********************************************************  USER CONFIG STRUCTURE FOR GPIO ******************************************************
typedef struct
{
	uint8_t GPIO_PinNumber;		 // << possible value from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;        // << possible values from @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;		// << possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl; // << possible value from @GPIO_PULL_UP_&_DOWN
	uint8_t GPIO_OPType;		 // << possible value from @GPIO_OP_TYPE
	uint8_t GPIO_PinAltFunMode;  // << possible value from @GPIO_ALT_FUN

}GPIO_Pin_Config_t;

//-----------------------------------------------------------   HANDLE STRUCTURE OF GPIO   --------------------------------------------------------------------------------------------------
typedef struct{
	GPIO_RegDef_t *pGPIOx;  // this hold the base addr of the gpio
	GPIO_Pin_Config_t GPIO_PinConfig;      // this hold the pin config of the gpio

}GPIO_Handle_t;

// ========================================(   GPIO PIN ports POSSIBLE MODES  )=====================================================================
// @GPIO_PIN_NUMBER
#define GPIO_PIN_NO_0     0
#define GPIO_PIN_NO_1     1
#define GPIO_PIN_NO_2     2
#define GPIO_PIN_NO_3     3
#define GPIO_PIN_NO_4     4
#define GPIO_PIN_NO_5     5
#define GPIO_PIN_NO_6     6
#define GPIO_PIN_NO_7     7
#define GPIO_PIN_NO_8     8
#define GPIO_PIN_NO_9     9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11    11
#define GPIO_PIN_NO_12    12
#define GPIO_PIN_NO_13    13
#define GPIO_PIN_NO_14    14
#define GPIO_PIN_NO_15    15

// @GPIO_PIN_MODE
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3

// @GPIO_PIN_IT
#define GPIO_MODE_IT_FT		4 // falling eddge trigger
#define GPIO_MODE_IT_RT		5 // rising edge trigger
#define GPIO_MODE_IT_RFT	6 // rising and falling edge trigger

// @GPIO_OP_TYPE
#define GPIO_OP_TYPE_PP     0// push pull
#define GPIO_OP_TYPE_OD     1 // open drain

//  @GPIO_PIN_SPEED
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// @GPIO_PULL_UP_&_DOWN
#define GPIO_NO_PUPD            0  // in case of open drain use this conenct an external register of 320k ohm and then the ext led will glow.
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

//@GPIO_ALT_FUN  --- not yet done


/*******************************************************************************************************************************************************************
 *                                              API SUPPORTED BY THE GPIO
 * ****************************************************************************************************************************************************************/


/*
 * Peripheral Clock Setup
 */

void GPIO_periClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi);
/* initialize and deinitiaze the peripheral
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);  // handle through this handler
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // we will simply disable the rcc for the gpio pin
/*
 * RD or WR to the peripheral
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); // name itself suggest we need the baseaddr or pinNumber
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);  //as the port is 16 bit therefore output is also 16 bit long
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value); // as we are sending data therefore no need of return type , value define either set or reset the pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value); // value is 16 bit bcz output is 16 bit long(port)
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); // for toggling we need the gpio pin or port number
/*
 * interrupt handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber); // require only the pin number to handle the intrerrupt









#endif /* STM32F407_DRIVER_H_ */
