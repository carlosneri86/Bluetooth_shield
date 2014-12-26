/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
#ifndef UART_H_
#define UART_H_


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                                         
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! UART driver status flags
enum eUARTStatus
{
	UART0_TX_DONE = 0,
	UART1_TX_DONE,
	UART2_TX_DONE,
	UART0_RX_DONE,
	UART1_RX_DONE,
	UART2_RX_DONE,
};
//! UARTs available on the SoC
enum eUARTS
{
	UART0 = 0,
	UART1,
	UART2,
	MAX_UARTS
};
//! Clock sources for UART0. Refer to the KL25 refernce manual for further information
enum eUART0_CLK
{
	UART0_CLK_FLL_PLL = 1,
	UART0_CLK_OSC,
	UART0_CLK_IRC
};
//! Macro used to poll the status register
#define UART_CHECK_STATUS(X)	(UART_gdwDriverStatus&(1<<X))
//! Macro used to set specific status
#define UART_SET_STATUS(X)		(UART_gdwDriverStatus |= (1<<X))
//! Macro used to clear specific status
#define UART_CLEAR_STATUS(X)	(UART_gdwDriverStatus &=~ (1<<X))
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Typedef Section                                          
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function-like Macros Section                                   
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Constants Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Variables Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Driver status register
extern uint32_t UART_gdwDriverStatus;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section                                    
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus
/*!
 *	@brief	Initializes the UART module specified, including baud rate.
 *	
 *	@param	bUARTToEnable		[in]	The Uart module to enable. Based on eUARTS enum
 *	
 *	@param	wBaudRate			[in]	Value to be configured on the Baud Rate register. Must be \
 *										pre-calculated, based on the KL25 reference manual
 *										
 *	@param	bOverSamplingUart0	[in]	In case the UART0 is used, an oversampling value is	\
 *										required.
 * 	@return	void							
 *	
*/
void UART_vfnInit(uint8_t bUARTToEnable, uint16_t wBaudRate, uint8_t bOverSamplingUart0);
/*!
 *	@brief	Transmits the specified data over the selected UART
 *	
 *	@param	bUartToUse			[in]	The Uart module to used for Tx
 *	
 *	@param	pbTxBuffer			[in]	Pointer to the buffer to be sent 
 *										
 *	@param	wDataToSend			[in]	Amount of bytes to be sent
 *	
 * 	@return	void							
 *	
*/
void UART_vfnTxBuffer(uint8_t bUartToUse, const uint8_t * pbTxBuffer, uint16_t wDataToSend);
/*!
 *	@brief	Returns the received byte from the selected UART. 
 *	
 *	@param	bUartToUse			[in]	The Uart module to used for Rx
 *		
 * 	@return	uint8_t	
 * 	@retval	The character received on the specified UART							
 *	
 *	@note Call this function just after UARTx_RX_DONE is set
*/
uint8_t UART_bfnRxBuffer(uint8_t bUartToRead);
#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* UART_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
