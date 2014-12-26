/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include "Bluetooth.h"
#include "UART.h"
#include "statemachine.h"
#include "SW_Timer.h"
#include "MiscFunctions.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! UART module to be used
#define BLUETOOTH_UART				(UART0)
//! Baudrate divider (calculated by hand)
#define BLUETOOTH_UART_BAUDRATE		(0x08)
//! On KL25, the UART0 requires an oversampling setting
#define BLUETOOTH_UART_BAUDRATE_OSR	(26U)
//! Amount of commands on the initialization array
#define BLUETOOTH_MAX_INIT_COMMANDS		(5)
//! RX buffer size
#define BLUETOOTH_MAX_RX_CHARACTERS	(255)
//! Timer period, to 5 seconds. This is used for init delays
#define BLUETOOTH_TIMER		(5000/SWTIMER_BASE_TIME)
//! Offset on the INIT array for the inquiry command
#define BLUETOOTH_INQUIRY_COMMAND_OFFSET	(4)
//! First byte Start of Frame of the AT command
#define BLUETOOTH_COMMAND_SOF_0			('\r')
//! Second byte Start of Frame of the AT command
#define BLUETOOTH_COMMAND_SOF_1			('\n')
//! First byte End of Frame of the AT command
#define BLUETOOTH_COMMAND_EOF_0				('\r')
//! Second byte End of Frame of the AT command
#define BLUETOOTH_COMMAND_EOF_1				('\n')
//! AT Command for a device connected
#define BLUETOOTH_DEVICE_CONNECTED			('4')
//! AT Command for a device disconnected
#define BLUETOOTH_DEVICE_DISCONNECTED			('1')
//! Offset on the RX buffer for the module status
#define BLUETOOTH_BTSTATUS_PARAMETER_OFFSET		(9U)
//! Asks for the specified flag
#define BLUETOOTH_CHECK_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus & 1<<x)
//! Sets the specified flag
#define BLUETOOTH_SET_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus |= (1<<x))
//! Clears the specified flag
#define BLUETOOTH_CLEAR_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus &= ~(1<<x))
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////
//! States offsets for the state machine
typedef enum
{
	BLUETOOTH_IDLE_STATE = 0,
	BLUETOOTH_WAIT_UART_STATE,
	BLUETOOTH_INIT_STATE,
	BLUETOOTH_WAIT_TIMER_STATE,
	BLUETOOTH_EXECUTE_COMMAND_STATE,
	BLUETOOTH_INQUIRY_STATE,
	BLUETOOTH_MAX_STATE
}eBluetoothStates;
//! Driver internal status values
typedef enum
{
	BLUETOOTH_TIMEOUT = 0,
	BLUETOOTH_COMMAND,
	BLUETOOTH_INIT_DONE,
	BLUETOOTH_SOF_0,
	BLUETOOTH_EOF_0
}eBluetoothInternalStatus;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section                 
///////////////////////////////////////////////////////////////////////////////////////////////////

static void Bluetooth_vfnIdleState (void);

static void Bluetooth_vfnWaitUartState (void);

static void Bluetooth_vfnInitState (void);

static void Bluetooth_vfnWaitTimerState (void);

static void Bluetooth_vfnExecuteCommandState (void);

static void Bluetooth_vfnInquiryState (void);

static void Bluetooth_vfnProcessRxData(void);

void Bluetooth_vfnSWTimerCallback(void);

static void ( * const Bluetooth_vfpaStateMachine[BLUETOOTH_MAX_STATE])(void) =
{
		Bluetooth_vfnIdleState,
		Bluetooth_vfnWaitUartState,
		Bluetooth_vfnInitState,
		Bluetooth_vfnWaitTimerState,
		Bluetooth_vfnExecuteCommandState,
		Bluetooth_vfnInquiryState
};
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////

#if BLUETOOTH_MODE == 0
static const uint8_t Bluetooth_gbaSetMode[] =
{
		"\n\r+STWMOD=0\r\n"
};
#elif  BLUETOOTH_MODE == 1
static const uint8_t Bluetooth_gbaSetMode[] =
{
		"\n\r+STWMOD=1\r\n"
};
#endif

#if	BLUETOOTH_BAUDRATE == 9600
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=9600\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 19200
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=19200\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 38400
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=38400\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 57600
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=57600\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 115200
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=115200\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 230400
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=230400\r\n"
};
#elif	BLUETOOTH_BAUDRATE == 460800
static const uint8_t Bluetooth_gbaSetBaudRate[] =
{
		"\r\n+STBD=460800\r\n"
};
#endif

#if BLUETOOTH_PERMIT_PAIRED_CONNECT == 1
static const uint8_t Bluetooth_gbaPermitPairedConnect[] =
{
		"\r\n+STOAUT=1\r\n"
};

#elif BLUETOOTH_PERMIT_PAIRED_CONNECT == 0
static const uint8_t Bluetooth_gbaPermitPairedConnect[] =
{
		"\r\n+STOAUT=0\r\n"
};
#endif

#if BLUETOOTH_AUTO_CONNECT == 1
static const uint8_t Bluetooth_gbaAutoConnect[] =
{
		"\r\n+STAUTO=1\r\n"
};

#elif BLUETOOTH_AUTO_CONNECT == 0
static const uint8_t Bluetooth_gbaAutoConnect[] =
{
		"\r\n+STAUTO=0\r\n"
};
#endif

#if BLUETOOTH_INQUIRE == 1
static const uint8_t Bluetooth_gbaInquire[] =
{
		"\r\n+INQ=1\r\n"
};

#elif BLUETOOTH_INQUIRE == 0
static const uint8_t Bluetooth_gbaInquire[] =
{
		"\r\n+INQ=0\r\n"
};
#endif


static const uint8_t Bluetooth_gbaModuleName[] =
{
		"\r\n+STNA=KL25_BT\r\n"	
};

static const uint8_t Bluetooth_gbaState[] =
{
		"+BTSTATE"
};

static const uint8_t * Bluetooth_gbpaInitCommands[BLUETOOTH_MAX_INIT_COMMANDS]=
{
	&Bluetooth_gbaSetMode[0],
	&Bluetooth_gbaModuleName[0],
	&Bluetooth_gbaPermitPairedConnect[0],
	&Bluetooth_gbaAutoConnect[0],
	&Bluetooth_gbaInquire[0],
				
};

static const uint8_t Bluetooth_gbpaInitCommandsSize[BLUETOOTH_MAX_INIT_COMMANDS]=
{
	sizeof(Bluetooth_gbaSetMode) - 1U,
	sizeof(Bluetooth_gbaModuleName) - 1U,
	sizeof(Bluetooth_gbaPermitPairedConnect) - 1U,
	sizeof(Bluetooth_gbaAutoConnect) - 1U,
	sizeof(Bluetooth_gbaInquire) - 1U,
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Holds the driver status
volatile uint8_t Bluetooth_gbStatus = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Holds the state machine states
static state_machine_t Bluetooth_tStateMachine;
//! Buffer used to received data from the BT module
static uint8_t Bluetooth_gbaCommandRxBuffer[BLUETOOTH_MAX_RX_CHARACTERS];
//! SW timer id
static uint8_t Bluetooth_gbSWTimer = 0;
//! Holds status used on the state machine
static volatile uint8_t Bluetooth_gInternalbStatus = 0;
//! Received data from UART
static uint8_t Bluetooth_NewRxData = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

void Bluetooth_vfnInitModule(void)
{
	/* Initialize the UART, request a SW timer and move the state machine */
	UART_vfnInit(BLUETOOTH_UART, BLUETOOTH_UART_BAUDRATE, BLUETOOTH_UART_BAUDRATE_OSR);
	
	Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_INIT_STATE;
	
	Bluetooth_gbSWTimer =  SWTimer_vfnAllocateChannel(BLUETOOTH_TIMER, Bluetooth_vfnSWTimerCallback);
	
}

uint8_t Bluetooth_bfnReadData (void)
{
	/* Clear the RX ready flag and return the new data */
	BLUETOOTH_CLEAR_STATUS(BLUETOOTH_DATA_READY);
	
	return(Bluetooth_NewRxData);
}

void Bluetooth_vfnWriteData(const uint8_t * bpDataToSend, uint16_t wAmountOfData)
{
	UART_vfnTxBuffer(BLUETOOTH_UART,bpDataToSend,wAmountOfData);
}

void Bluetooth_vfnStateMachine(void)
{
	
	/* start receiving data just after initialization */
	if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_INIT_DONE))
	{
		if(UART_CHECK_STATUS(UART0_RX_DONE))
		{			
			Bluetooth_vfnProcessRxData();
		}
	}
	
	Bluetooth_vfpaStateMachine[Bluetooth_tStateMachine.bCurrentState]();
}

static void Bluetooth_vfnIdleState(void)
{
	
}

static void Bluetooth_vfnWaitUartState(void)
{
	/* Do nothing until all data is sent */
	if(UART_CHECK_STATUS(UART0_TX_DONE))
	{
		Bluetooth_tStateMachine.bCurrentState = Bluetooth_tStateMachine.bNextState;
		
		UART_CLEAR_STATUS(UART0_TX_DONE);
	}
}

static void Bluetooth_vfnInitState(void)
{
	static uint8_t bInitOffset = 0;
	
	/* Send all the AT commands for the initialization */
	if(BLUETOOTH_MAX_INIT_COMMANDS > bInitOffset)
	{
		UART_vfnTxBuffer(BLUETOOTH_UART,Bluetooth_gbpaInitCommands[bInitOffset],Bluetooth_gbpaInitCommandsSize[bInitOffset]);
		
		bInitOffset++;
		
		if(bInitOffset != BLUETOOTH_INQUIRY_COMMAND_OFFSET)
		{
			Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_WAIT_UART_STATE;
			
			Bluetooth_tStateMachine.bNextState = BLUETOOTH_INIT_STATE;
		}
		else
		{
			/* The inquire command must be sent after a delay (according to sample code) */
			
			SWTimer_vfnEnableTimer(Bluetooth_gbSWTimer);
			
			Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_WAIT_TIMER_STATE;
				
			Bluetooth_tStateMachine.bNextState = BLUETOOTH_INIT_STATE;
		}
	}
	else
	{
		/* Once the initialization is done, return to IDLE */
		Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
		
		BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_INIT_DONE);
		
		bInitOffset = 0;
	}
	
}

static void Bluetooth_vfnWaitTimerState (void)
{
	/* Wait until the SW timer period is exhausted */
	if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_TIMEOUT))
	{
		Bluetooth_tStateMachine.bCurrentState = Bluetooth_tStateMachine.bNextState;
		
		BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_TIMEOUT);
	}
}

static void Bluetooth_vfnExecuteCommandState (void)
{
	uint8_t bStatus;
	/* Compare the command received with BTSTATUS (the only command supported for now */
	bStatus = bfnStringCompare(&Bluetooth_gbaState[0], Bluetooth_gbaCommandRxBuffer ,sizeof(Bluetooth_gbaState) - 1U);
	
	/* If it was a BTSTATUS, now check if it was a connect or disconnect event */
	if(bStatus == STRING_OK)
	{
		/* In case the status resulted in a connected device, set proper flags and go back to idle*/
		if(BLUETOOTH_DEVICE_CONNECTED == Bluetooth_gbaCommandRxBuffer[BLUETOOTH_BTSTATUS_PARAMETER_OFFSET])
		{
			BLUETOOTH_SET_STATUS(BLUETOOTH_CONNECTED);
			
			BLUETOOTH_CLEAR_STATUS(BLUETOOTH_DATA_READY);
			
			Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
		}
		else
		{
			/* if it was a device disconnection, then clear the proper flags, wait some time and send the inquiry command*/
			if(BLUETOOTH_DEVICE_DISCONNECTED == Bluetooth_gbaCommandRxBuffer[BLUETOOTH_BTSTATUS_PARAMETER_OFFSET])
			{
				BLUETOOTH_CLEAR_STATUS(BLUETOOTH_CONNECTED);
				
				Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_WAIT_TIMER_STATE;
				
				Bluetooth_tStateMachine.bNextState = BLUETOOTH_INQUIRY_STATE;
				
				SWTimer_vfnEnableTimer(Bluetooth_gbSWTimer);
			}
			else
			{
				Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
			}
		}
	}
	else
	{
		Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
	}
}

static void Bluetooth_vfnInquiryState (void)
{
	/* Send the inquiry command, wait for UART to finish tx and go back to Idle */
	UART_vfnTxBuffer(BLUETOOTH_UART,&Bluetooth_gbaInquire[0],sizeof(Bluetooth_gbaInquire) - 1U);
	
	Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_WAIT_UART_STATE;
				
	Bluetooth_tStateMachine.bNextState = BLUETOOTH_IDLE_STATE;
}

void Bluetooth_vfnSWTimerCallback(void)
{
	BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_TIMEOUT);
	
	SWTimer_vfnDisableTimer(Bluetooth_gbSWTimer);
}

static void Bluetooth_vfnProcessRxData(void)
{
	static uint16_t wRxBufferOffset = 0;
	
	uint8_t bRxData;
	
	bRxData = UART_bfnRxBuffer(BLUETOOTH_UART);
	/* Parse the received data */
	/* first identify if it is a command by looking for the SOF */
	/* the AT commands sends \r\n COMMAND \r\n					*/
	/* This function looks for \r\n to confirm it is a command 	*/
	/* if not, then is plain data, and reports it accordingly	*/
	/* if is a command, receive it and execute it				*/
	if(!BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_COMMAND))
	{
		if(bRxData == BLUETOOTH_COMMAND_SOF_0)
		{
			BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_SOF_0);	
		}
		else
		{
			if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_SOF_0))
			{
				if(bRxData == BLUETOOTH_COMMAND_SOF_1)
				{
					BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_COMMAND);	
				}
				else
				{
					BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_SOF_0);	
				}
			}
			else
			{

				BLUETOOTH_SET_STATUS(BLUETOOTH_DATA_READY);
				
				Bluetooth_NewRxData = bRxData;

			}
		}
	}
	else
	{
		if(bRxData == BLUETOOTH_COMMAND_EOF_0)
		{
			BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_EOF_0);	
		}
		else
		{
			if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_EOF_0))
			{
				if(bRxData == BLUETOOTH_COMMAND_EOF_1)
				{
					BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_COMMAND);
					BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_SOF_0);
					BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_EOF_0);
					Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_EXECUTE_COMMAND_STATE;
					wRxBufferOffset = 0;
				}
				else
				{
					BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_SOF_0);	
				}
			}
			else
			{
				Bluetooth_gbaCommandRxBuffer[wRxBufferOffset++] = bRxData;
				
				if(wRxBufferOffset> BLUETOOTH_MAX_RX_CHARACTERS)
				{
					wRxBufferOffset = 0;
				}
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
