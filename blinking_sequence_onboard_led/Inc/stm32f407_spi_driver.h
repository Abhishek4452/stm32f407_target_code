/*
 *                                               stm32f407_spi_driver.h
 *
 *  Created on: 13-Jan-2026
 *      Author: debian
 */

#ifndef STM32F407_SPI_DRIVER_H_
#define STM32F407_SPI_DRIVER_H_

#include "stm32f407xx.h"
// this section is done from the rf manual one to one register
typedef struct
{  // this is done for the user
	 uint8_t SPI_DeviceMode;//default slave // MASTER OR SLAVE ,        				    @SPI_DEVICEMODE
	 uint8_t SPI_BusConfig;      			// FULL DUPLEX, half ,simplex   				@SPI_BUSConfig
	 uint8_t SPI_SclkSpeed;      			//												@SPI_SclkSpeed
	 uint8_t SPI_DFF;      					// data frame format							@SPI_DFF
	 uint8_t SPI_CPOL;      				//												@SPI_CPOL
	 uint8_t SPI_CPHA;      				//												@SPI_CPHL
	 uint8_t SPI_SSM;      					// slave management - hardware or software		@SPI_SSM

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx; //hold base addr of the spi
	SPI_Config_t SPIConfig;

}SPI_Handle_t;

/*
 *      @SPI_DEVICEMODE
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0

/*
 *      @SPI_BUSConfig
 *      BIDIMODE and BIDIOE both decide the config
 *      NOW I AM PERFORMING ONLY FULL DUPLEX CONFIG
 */
#define SPI_BUS_CONFIG_FD		      1
#define SPI_BUS_CONFIG_HD   		  2
// no need of it - #define SPI_BUS_CONFIG_SIMPLEX_TXONLY 3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 4
/*
 *     @SPI_SclkSpeed
 *     BR( baud rate ) decide the speed, 3 bits = 8 possible combination
 */
#define SPI_SCLK_SPEED_DIV_BY_2            0
#define SPI_SCLK_SPEED_DIV_BY_4            1
#define SPI_SCLK_SPEED_DIV_BY_8            2
#define SPI_SCLK_SPEED_DIV_BY_16           3
#define SPI_SCLK_SPEED_DIV_BY_32           4
#define SPI_SCLK_SPEED_DIV_BY_64           5
#define SPI_SCLK_SPEED_DIV_BY_128          6
#define SPI_SCLK_SPEED_DIV_BY_256          7

/*
 *     @SPI_DFF
 *     0: 8-bit data frame , 1: 16-bit data frame
 */
#define SPI_DFF_8_BIT          0
#define SPI_DFF_16_BIT		   1
/*
 *      @SPI_CPOL
 */
#define SPI_CPOL_LOW 			0
#define SPI_CPOL_HIGH			1
/*
 *      @SPI_CPHL
 */
#define SPI_CPHL_LOW 			0
#define SPI_CPHL_HIGH			1
/*      @SPI_SSM
 *      0: Software slave management disabled ,( sw )
 *      1: Software slave management enabled
 */
#define SPI_SSM_EN				1
#define SPI_SSM_DI    			0

/*******************************************************************************************************************************************************************
 *                                              API SUPPORTED BY THE SPI
 * ****************************************************************************************************************************************************************/

/*
 *    PERIPHERAL CLK SETUP
 */
void SPI_periClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);




/*
 *  INIT and DE-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/*
 * IRQ configuration and ISR handling
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);
/*
 *  IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * other peripheral control APIs
 */


#endif /* STM32F407_SPI_DRIVER_H_ */
