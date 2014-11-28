 /*************************************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2010 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 **************************************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************//*!
 *
 * @file UART.h
 *
 * @author B22385
 *
 * @date Dec 21, 2012
 *
 * @brief 
 *************************************************************************************************/


#ifndef UART_H_
#define UART_H_

/*************************************************************************************************/
/*                                      Includes Section                                         */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Defines & Macros Section                                     */
/*************************************************************************************************/
enum
{
	UART_INTERRUPT_ENABLE = 0,
	UART_DMA_ENABLE,
	UART_PARITY_ENABLE,
	UART_PARITY_ODD
};

enum eUARTStatus
{
	UART_TX_PROGRESS = 0,
	UART_RX_PROGRESS,
	UART_TX_DONE,
	UART_RX_DONE
};
enum eUARTS
{
	UART0 = 0,
	UART1,
	UART2,
	MAX_UARTS
};

enum eUART0_CLK
{
	UART0_CLK_FLL_PLL = 1,
	UART0_CLK_OSC,
	UART0_CLK_IRC
};
#define UART_UART0_SRC_CLK			()

#define UART_INTERRUPT_ENABLE_MASK	(1<<UART_INTERRUPT_ENABLE)
#define UART_DMA_ENABLE_MASK		(1<<UART_DMA_ENABLE)
#define UART_PARITY_ENABLE_MASK		(1<<UART_PARITY_ENABLE)
#define	UART_PARITY_ODD_MASK		(1<<UART_PARITY_ODD)

#define UART_CHECK_STATUS(X)	(UART_u32DriverStatus&(1<<X))
#define UART_SET_STATUS(X)		(UART_u32DriverStatus |= (1<<X))
#define UART_CLEAR_STATUS(X)	(UART_u32DriverStatus &=~ (1<<X))
/*************************************************************************************************/
/*                                      Typedef Section                                          */
/*************************************************************************************************/


/*************************************************************************************************/
/*                                Function-like Macros Section                                   */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Extern Constants Section                                     */
/*************************************************************************************************/

/*************************************************************************************************/
/*                                  Extern Variables Section                                     */
/*************************************************************************************************/
extern uint32_t UART_u32DriverStatus;
/*************************************************************************************************/
/*                                Function Prototypes Section                                    */
/*************************************************************************************************/
void UART_vfnInit(uint8_t u8UARTToEnable, uint16_t u16BaudRate, uint8_t u8OverSamplingUart0);
void UART_vfnTxBuffer(uint8_t u8UartToUse, const uint8_t * pu8TxBuffer, uint16_t u16DataToSend);
uint8_t UART_bfnRxBuffer(void);
/*************************************************************************************************/

#endif /* UART_H_ */
