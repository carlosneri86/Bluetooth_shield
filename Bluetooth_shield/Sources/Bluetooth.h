/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                                         
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Sets STWMOD to 1 or 0. Refer to the Seeed BT module init
#define BLUETOOTH_MODE 					(0)
//! Baudrate for communication. Must be left as 38400. Currently not supporting baudrate changing
#define BLUETOOTH_BAUDRATE				(38400)
//! Sets STOAUT to 1 or 0. Refer to the Seed BT module init
#define BLUETOOTH_PERMIT_PAIRED_CONNECT	(1)
//! Sets STAUT to 1 or 0. Refer to the Seed BT module init
#define BLUETOOTH_AUTO_CONNECT			(0)
//! Enables (1) or Disables (0) being inquired
#define BLUETOOTH_INQUIRE				(1)
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Typedef Section                                          
///////////////////////////////////////////////////////////////////////////////////////////////////
//! BT layer status. These shall be polled by the upper layers
typedef enum
{
	BLUETOOTH_BUSY = 0,
	BLUETOOTH_CONNECTED,
	BLUETOOTH_DATA_READY
}eBluetoothStatus;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function-like Macros Section                                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Macro to poll specific status
#define BLUETOOTH_CHECK_STATUS(x)	(Bluetooth_gbStatus & 1<<x)
//! Macro to set specific status
#define BLUETOOTH_SET_STATUS(x)	(Bluetooth_gbStatus |= (1<<x))
//! Macro to clear specific status
#define BLUETOOTH_CLEAR_STATUS(x)	(Bluetooth_gbStatus &= ~(1<<x))
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Constants Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Variables Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Status register
extern volatile uint8_t Bluetooth_gbStatus;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section                                    
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus
/*!
 *	@brief	Initializes phyisical layer for communication with BT module and start sending commands
 * 	
 * 	@return	void		
 * 	
 * 	@note Current initialization is as slaver						
 *	
*/
void Bluetooth_vfnInitModule(void);
/*!
 *	@brief	Moves the state machine and polls receiving data
 * 	
 * 	@return	void	
 * 	
 * 	@note Should be called periodically						
 *	
*/
void Bluetooth_vfnStateMachine(void);
/*!
 *	@brief	Retreives received data on a byte basis
 * 	
 * 	@return	uint8_t
 * 	@retval	Character received by the BT module							
 *	
*/
uint8_t Bluetooth_bfnReadData (void);
/*!
 *	@brief	Sends data thru BT
 *	
 *	@param	bpDataToSend	[in]	pointer to the data to be sent
 *	
 *	@param	wAmountOfData	[in]	Amount of bytes to be sent
 * 	
 * 	@return	void							
 *	
*/
void Bluetooth_vfnWriteData(const uint8_t * bpDataToSend, uint16_t wAmountOfData);

#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* BLUETOOTH_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
