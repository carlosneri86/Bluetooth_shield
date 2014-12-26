
/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <derivative.h>
#include "NVIC.h"
#include "UART.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Amount of registers for configuring the baud rate
#define UART_BAUD_RATE_REGISTERS	(2)
//! Offset for the first baudrate register
#define UART_BAUD_RATE_LOW_REGISTER_OFFSET	(0)
//! Offset for the second baudrate register
#define UART_BAUD_RATE_HIGH_REGISTER_OFFSET	(1)
//! Amount of registers available on each the UART modules
#define UART_REGISTERS	(9)
//! Offset for each UART register
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
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section                 
///////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Array with each UART clock gate masks
static const uint32_t UART_gadwClockGateMask[MAX_UARTS] =
{
		SIM_SCGC4_UART0_MASK,
		SIM_SCGC4_UART1_MASK,
		SIM_SCGC4_UART2_MASK
};
//! Each UART module registers
static volatile uint8_t * const UART_gapbRegisters[MAX_UARTS][UART_REGISTERS] =
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
//! Each UART NVIC source
static const uint32_t UART_gadwNvicSources[MAX_UARTS] =
{
		NVIC_UART0,
		NVIC_UART1,
		NVIC_UART2
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! UART driver status register
uint32_t UART_gdwDriverStatus = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Holds the buffer pointer to Transmit over Uart0 
static volatile uint8_t * UART_gpbOutputBufferUart0;
//! Holds the buffer pointer to Transmit over Uart1
static volatile uint8_t * UART_gpbOutputBufferUart1;
//! Holds the buffer pointer to Transmit over Uart2
static volatile uint8_t * UART_gpbOutputBufferUart2;
//! Holds amount of bytes to be sent over Uart0
static volatile uint16_t  UART_gwDataOutCounterUart0 = 0;
//! Holds amount of bytes to be sent over Uart1
static volatile uint16_t  UART_gwDataOutCounterUart1 = 0;
//! Holds amount of bytes to be sent over Uart2
static volatile uint16_t  UART_gwDataOutCounterUart2 = 0;
//! Holds the data received from Uart0
static volatile uint8_t  UART_gbRxDataUart0;
//! Holds the data received from Uart1
static volatile uint8_t  UART_gbRxDataUart1;
//! Holds the data received from Uart2
static volatile uint8_t  UART_gbRxDataUart2;
//! Counter for framing error. Used only for debug
volatile uint16_t UART_gwFramingErrorCounter = 0;
//! Counter for over run error. Used only for debug
volatile uint16_t UART_gwOverRunCounter = 0;
//! Counter for received data. Used only for debug
volatile uint16_t UART_gwRxDataCounter = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

void UART_vfnInit(uint8_t bUARTToEnable, uint16_t wBaudRate, uint8_t bOverSamplingUart0)
{
	
	uint8_t * pbBaudRateLow;
	uint8_t * pbBaudRateHigh;
	uint8_t * pbControlRegister;
	
	/* Confirm the UART exists */
	if(bUARTToEnable < MAX_UARTS)
	{
		/* Turn on the clock gate for the proper UART */	
		SIM_SCGC4 |= UART_gadwClockGateMask[bUARTToEnable];
		
		/* Take the addresses for the baud rate registers */
		pbBaudRateLow = (uint8_t*)UART_gapbRegisters[bUARTToEnable][UART_BDL];
		pbBaudRateHigh = (uint8_t*)UART_gapbRegisters[bUARTToEnable][UART_BDH];
		
		/* write the register with the received baud rate*/
		*pbBaudRateLow = (uint8_t)wBaudRate;
		wBaudRate >>= 8;
		wBaudRate &=  UART_BDH_SBR_MASK;
		*pbBaudRateHigh = wBaudRate;
		
		/* In case is UART0, configure clock sources and oversampling */
		if(bUARTToEnable == UART0)
		{			
			SIM_SOPT2 |= SIM_SOPT2_UART0SRC(UART0_CLK_OSC);
			 
			UART0_C4 = UART0_C4_OSR(bOverSamplingUart0-1);
		}

		/* Enable the receiver and the interrupt Rx */
		pbControlRegister = (uint8_t*)UART_gapbRegisters[bUARTToEnable][UART_C2];
		*pbControlRegister |= UART_C2_RE_MASK|UART_C2_RIE_MASK;
		/* Enable the interrupt of the proper UART */
		NVIC_vfnEnableIRQ(UART_gadwNvicSources[bUARTToEnable]);
	}		
}

void UART_vfnTxBuffer(uint8_t bUartToUse, const  uint8_t * pbTxBuffer, uint16_t wDataToSend)
{
	uint8_t * pbControlRegister;
	/* Confirm the UART exists */
	if(bUartToUse < MAX_UARTS)
	{
		/* Take the register address*/
		pbControlRegister = (uint8_t*)UART_gapbRegisters[bUartToUse][UART_C2];
		/* Back up the received buffer buffer and amount of data on the proper UART	*/
		/* Clear the TX done flag													*/
		if(bUartToUse == UART0)
		{
		
			UART_gpbOutputBufferUart0 = (uint8_t *)pbTxBuffer;
			UART_gwDataOutCounterUart0 = wDataToSend;
			
			UART_CLEAR_STATUS(UART0_TX_DONE);
		}
		else if(bUartToUse == UART1)
		{
		
			UART_gpbOutputBufferUart1 = (uint8_t *)pbTxBuffer;
			UART_gwDataOutCounterUart1 = wDataToSend;
			
			UART_CLEAR_STATUS(UART1_TX_DONE);
		}
		else if(bUartToUse == UART2)
		{
		
			UART_gpbOutputBufferUart2 = (uint8_t *)pbTxBuffer;
			UART_gwDataOutCounterUart2 = wDataToSend;
			
			UART_CLEAR_STATUS(UART2_TX_DONE);
		}
		
		/* Enable the transmitter and interrupt */
		*pbControlRegister |= UART_C2_TE_MASK|UART_C2_TIE_MASK;
	}
}

uint8_t UART_bfnRxBuffer(uint8_t bUartToRead)
{
	uint8_t bNewReadData;
	
	/* Take the data from the proper UART */
	if(bUartToRead == UART0)
	{
		bNewReadData = UART_gbRxDataUart0;
		
		UART_CLEAR_STATUS(UART0_RX_DONE);
	}
	else if(bUartToRead == UART1)
	{
	
		bNewReadData = UART_gbRxDataUart1;
		
		UART_CLEAR_STATUS(UART1_RX_DONE);
	}
	else if(bUartToRead == UART2)
	{
	
		bNewReadData = UART_gbRxDataUart2;
		
		UART_CLEAR_STATUS(UART2_RX_DONE);
	}
	
	return(bNewReadData);

}

void UART0_IRQHandler(void)
{
	if(UART0_S1&UART_S1_TDRE_MASK)
	{
		/* Process the Tx interrupt. Check if there's still data and send it 	*/
		/* Set Tx done flag otherwise and turn off transmitter					*/
		if(UART_gwDataOutCounterUart0--)
		{
			UART0_D = *UART_gpbOutputBufferUart0++;
		}
		else
		{
			UART0_C2 &= ~(UART_C2_TE_MASK|UART_C2_TIE_MASK);
			
			UART_SET_STATUS(UART0_TX_DONE);
		}
	
	}
	
	if(UART0_S1&UART_S1_RDRF_MASK)
	{
		/* Take data from the UART and set the proper flag */
		UART_gbRxDataUart0 = UART0_D;
					
		UART_SET_STATUS(UART0_RX_DONE);
		
		UART_gwRxDataCounter++;
	}
	if(UART0_S1&UART_S1_OR_MASK)
	{
		UART_gwOverRunCounter++;
		UART0_S1 |= UART_S1_OR_MASK;
	}
	if(UART0_S1&UART0_S1_FE_MASK)
	{
		UART_gwFramingErrorCounter++;
		UART0_S1 |= UART0_S1_FE_MASK;
	}
}

void UART1_IRQHandler(void)
{
	if(UART1_S1&UART_S1_TDRE_MASK)
	{
		/* Process the Tx interrupt. Check if there's still data and send it 	*/
		/* Set Tx done flag otherwise and turn off transmitter					*/
		if(UART_gwDataOutCounterUart1--)
		{
			UART1_D = *UART_gpbOutputBufferUart1++;
		}
		else
		{
			UART1_C2 &= ~(UART_C2_TE_MASK|UART_C2_TIE_MASK);
			
			UART_SET_STATUS(UART1_TX_DONE);
		}
	}
	if(UART1_S1&UART_S1_RDRF_MASK)
	{
		/* Take data from the UART and set the proper flag */
		UART_gbRxDataUart1 = UART1_D;
					
		UART_SET_STATUS(UART1_RX_DONE);
		
		UART_gwRxDataCounter++;
	}
	if(UART1_S1&UART_S1_OR_MASK)
	{
		UART_gwOverRunCounter++;
		UART1_S1 |= UART_S1_OR_MASK;
	}
	if(UART1_S1&UART_S1_FE_MASK)
	{
		UART_gwFramingErrorCounter++;
		UART1_S1 |= UART_S1_FE_MASK;
	}
}

void UART2_IRQHandler(void)
{
	if(UART2_S1&UART_S1_TDRE_MASK)
	{
		/* Process the Tx interrupt. Check if there's still data and send it 	*/
		/* Set Tx done flag otherwise and turn off transmitter					*/
		if(UART_gwDataOutCounterUart2--)
		{
			UART2_D = *UART_gpbOutputBufferUart2++;
		}
		else
		{
			UART2_C2 &= ~(UART_C2_TE_MASK|UART_C2_TIE_MASK);
			
			UART_SET_STATUS(UART2_TX_DONE);
		}
	}
	if(UART2_S1&UART_S1_RDRF_MASK)
	{
		/* Take data from the UART and set the proper flag */
		UART_gbRxDataUart2 = UART2_D;
					
		UART_SET_STATUS(UART2_RX_DONE);
		
		UART_gwRxDataCounter++;
	}
	if(UART2_S1&UART_S1_OR_MASK)
	{
		UART_gwOverRunCounter++;
		UART2_S1 |= UART_S1_OR_MASK;
	}
	if(UART2_S1&UART_S1_FE_MASK)
	{
		UART_gwFramingErrorCounter++;
		UART2_S1 |= UART_S1_FE_MASK;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
