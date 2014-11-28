 /*********************************************************************************************//*!
 *
 * @file UART.c
 *
 * @author Carlos Neri
 *
 * @date 4/02/2012
 *
 * @brief Brief description of the file
 *************************************************************************************************/
/*************************************************************************************************/
/*                                      Includes Section                                         */
/*************************************************************************************************/
#include <derivative.h>
#include "NVIC.h"
#include "UART.h"
/*************************************************************************************************/
/*                                  Function Prototypes Section                                  */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Defines & Macros Section                                    */
/*************************************************************************************************/
#define UART_BDR_DIVIDER	(16)
#define UART_BAUD_RATE_REGISTERS	(2)
#define UART_BAUD_RATE_LOW_REGISTER_OFFSET	(0)
#define UART_BAUD_RATE_HIGH_REGISTER_OFFSET	(1)
#define UART_REGISTERS	(9)

enum eUART_Registers
{
	UART_BDH = 0,
	UART_BDL,
	UART_C1,
	UART_C2,
	UART_S1,
	UART_S2,
	UART_C3,
	UART_D,
	UART_C4
};
/*************************************************************************************************/
/*                                       Typedef Section                                         */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Global Variables Section                                    */
/*************************************************************************************************/
uint32_t UART_u32DriverStatus = 0;

static uint8_t * UART_gpu8OutputBuffer;
static uint16_t  UART_gu16DataOutCounter = 0;

static volatile uint8_t  UART_gbRxData;

/*************************************************************************************************/
/*                                   Static Variables Section                                    */
/*************************************************************************************************/
static const uint32_t UART_gau32ClockGateMask[MAX_UARTS] =
{
		SIM_SCGC4_UART0_MASK,
		SIM_SCGC4_UART1_MASK,
		SIM_SCGC4_UART2_MASK
};

static volatile uint8_t * const UART_gapu8Registers[MAX_UARTS][UART_REGISTERS] =
{
		{
			&UART0_BDH,
			&UART0_BDL,
			&UART0_C1,
			&UART0_C2,
			&UART0_S1,
			&UART0_S2,
			&UART0_C3,
			&UART0_D,
			&UART0_C4,
		},
		{
			&UART1_BDH,
			&UART1_BDL,
			&UART1_C1,
			&UART1_C2,
			&UART1_S1,
			&UART1_S2,
			&UART1_C3,
			&UART1_D,
			&UART1_C4,
		},
		{
			&UART2_BDH,
			&UART2_BDL,
			&UART2_C1,
			&UART2_C2,
			&UART2_S1,
			&UART2_S2,
			&UART2_C3,
			&UART2_D,
			&UART2_C4,
		}
};
/*************************************************************************************************/
/*                                   Global Constants Section                                    */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                   Static Constants Section                                    */
/*************************************************************************************************/
volatile uint16_t gwFramingErrorCounter = 0;
volatile uint16_t gwOverRunCounter = 0;
volatile uint16_t gwRxDataCounter = 0;
/*************************************************************************************************/
/*                                      Functions Section                                        */
/*************************************************************************************************/
void UART_vfnInit(uint8_t u8UARTToEnable, uint16_t u16BaudRate, uint8_t u8OverSamplingUart0)
{
	
	uint8_t * pu8BaudRateLow;
	uint8_t * pu8BaudRateHigh;
	uint8_t * pbControlRegister;
	if(u8UARTToEnable < MAX_UARTS)
	{
		
		
		SIM_SCGC4 |= UART_gau32ClockGateMask[u8UARTToEnable];
		
		pu8BaudRateLow = (uint8_t*)UART_gapu8Registers[u8UARTToEnable][UART_BDL];
		pu8BaudRateHigh = (uint8_t*)UART_gapu8Registers[u8UARTToEnable][UART_BDH];
		
		*pu8BaudRateLow = (uint8_t)u16BaudRate;
		u16BaudRate >>= 8;
		u16BaudRate &=  UART_BDH_SBR_MASK;
		*pu8BaudRateHigh = u16BaudRate;
		
		
		if(u8UARTToEnable == UART0)
		{			
			NVIC_vfnEnableIRQ(NVIC_UART0);
			 
			SIM_SOPT2 |= SIM_SOPT2_UART0SRC(UART0_CLK_OSC);
			 
			UART0_C4 = UART0_C4_OSR(u8OverSamplingUart0-1); 
			
			pbControlRegister = (uint8_t*)UART_gapu8Registers[u8UARTToEnable][UART_C2];
			*pbControlRegister |= UART_C2_RE_MASK|UART_C2_RIE_MASK;
			
		}
		else if(u8UARTToEnable == UART1)
		{
			NVIC_vfnEnableIRQ(NVIC_UART1);
		}
		else
		{	
			NVIC_vfnEnableIRQ(NVIC_UART2);
		}
		
	}
		
}

void UART_vfnTxBuffer(uint8_t u8UartToUse, const  uint8_t * pu8TxBuffer, uint16_t u16DataToSend)
{
	uint8_t * pu8ControlRegister;
	
	if(!(UART_CHECK_STATUS(UART_TX_PROGRESS)))
	{
		if(u8UartToUse < MAX_UARTS)
		{
			pu8ControlRegister = (uint8_t*)UART_gapu8Registers[u8UartToUse][UART_C2];
			
			UART_gpu8OutputBuffer = (uint8_t *)pu8TxBuffer;
			UART_gu16DataOutCounter = u16DataToSend;
			
			UART_SET_STATUS(UART_TX_PROGRESS);
			
			*pu8ControlRegister |= UART_C2_TE_MASK|UART_C2_TIE_MASK;
		}
	}
}

uint8_t UART_bfnRxBuffer(void)
{
	UART_CLEAR_STATUS(UART_RX_DONE);
	
	return(UART_gbRxData);

}

void UART0_IRQHandler(void)
{
	if(UART0_S1&UART_S1_TDRE_MASK)
	{
		if(UART_CHECK_STATUS(UART_TX_PROGRESS))
		{
			if(UART_gu16DataOutCounter--)
			{
				UART0_D = *UART_gpu8OutputBuffer++;
			}
			else
			{
				UART0_C2 &= ~(UART_C2_TE_MASK|UART_C2_TIE_MASK);
				UART_CLEAR_STATUS(UART_TX_PROGRESS);
			}
		}
	}
	if(UART0_S1&UART_S1_RDRF_MASK)
	{

		UART_gbRxData = UART0_D;
					
		UART_SET_STATUS(UART_RX_DONE);
		
		gwRxDataCounter++;
	}
	if(UART0_S1&UART_S1_OR_MASK)
	{
		gwOverRunCounter++;
		UART0_S1 |= UART_S1_OR_MASK;
	}
	if(UART0_S1&UART0_S1_FE_MASK)
	{
		gwFramingErrorCounter++;
		
		UART0_S1 |= UART0_S1_FE_MASK;
	}
}

void UART1_IRQHandler(void)
{
	if(UART1_S1&UART_S1_TDRE_MASK)
	{
		if(UART_gu16DataOutCounter--)
		{
			UART1_D = *UART_gpu8OutputBuffer++;
		}
		else
		{
			UART1_C2 &= ~UART_C2_TE_MASK;
			UART_CLEAR_STATUS(UART_TX_PROGRESS);
		}
	}
}

void UART2_IRQHandler(void)
{
	if(UART2_S1&UART_S1_TDRE_MASK)
	{
		if(UART_gu16DataOutCounter--)
		{
			UART2_D = *UART_gpu8OutputBuffer++;
		}
		else
		{
			UART2_C2 &= ~UART_C2_TE_MASK;
			UART_CLEAR_STATUS(UART_TX_PROGRESS);
		}
	}

}
