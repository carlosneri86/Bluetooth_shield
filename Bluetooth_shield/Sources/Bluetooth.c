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

#define BLUETOOTH_UART				(UART0)

#define BLUETOOTH_UART_BAUDRATE		(0x08)

#define BLUETOOTH_UART_BAUDRATE_OSR	(26U)

#define BLUETOOTH_MAX_INIT_COMMANDS		(5)

#define BLUETOOTH_MAX_RX_CHARACTERS	(255)

#define BLUETOOTH_TIMER		(5000/SWTIMER_BASE_TIME)

#define BLUETOOTH_INQUIRY_COMMAND_OFFSET	(4)

#define BLUETOOTH_COMMAND_SOF_0			('\r')

#define BLUETOOTH_COMMAND_SOF_1			('\n')

#define BLUETOOTH_COMMAND_EOF_0				('\r')

#define BLUETOOTH_COMMAND_EOF_1				('\n')

#define BLUETOOTH_DEVICE_CONNECTED			('4')

#define BLUETOOTH_DEVICE_DISCONNECTED			('1')

#define BLUETOOTH_BTSTATUS_PARAMETER_OFFSET		(9U)

#define BLUETOOTH_CHECK_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus & 1<<x)

#define BLUETOOTH_SET_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus |= (1<<x))

#define BLUETOOTH_CLEAR_INTERNAL_STATUS(x)	(Bluetooth_gInternalbStatus &= ~(1<<x))
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////

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

volatile uint8_t Bluetooth_gbStatus = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////

static state_machine_t Bluetooth_tStateMachine;

static uint8_t Bluetooth_gbaRxBuffer[BLUETOOTH_MAX_RX_CHARACTERS];

static uint8_t Bluetooth_gbaCommandRxBuffer[BLUETOOTH_MAX_RX_CHARACTERS];

static uint8_t Bluetooth_gbSWTimer = 0;

static volatile uint8_t Bluetooth_gInternalbStatus = 0;

static uint8_t Bluetooth_NewRxData = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

void Bluetooth_vfnInitModule(void)
{
	UART_vfnInit(BLUETOOTH_UART, BLUETOOTH_UART_BAUDRATE, BLUETOOTH_UART_BAUDRATE_OSR);
	
	Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_INIT_STATE;
	
	Bluetooth_gbSWTimer =  SWTimer_vfnAllocateChannel(BLUETOOTH_TIMER, Bluetooth_vfnSWTimerCallback);
	
}

uint8_t Bluetooth_bfnReadData (void)
{
	BLUETOOTH_CLEAR_STATUS(BLUETOOTH_DATA_READY);
	
	return(Bluetooth_NewRxData);
}

void Bluetooth_vfnWriteData(const uint8_t * bpDataToSend, uint16_t wAmountOfData)
{
	UART_vfnTxBuffer(BLUETOOTH_UART,bpDataToSend,wAmountOfData);
}

void Bluetooth_vfnStateMachine(void)
{
	
	/*  rx data to be parsed after init */
	if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_INIT_DONE))
	{
		if(UART_CHECK_STATUS(UART_RX_DONE))
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
	if(!UART_CHECK_STATUS(UART_TX_PROGRESS))
	{
		Bluetooth_tStateMachine.bCurrentState = Bluetooth_tStateMachine.bNextState;
	}
}

static void Bluetooth_vfnInitState(void)
{
	static uint8_t bInitOffset = 0;
	
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
		Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
		
		BLUETOOTH_SET_INTERNAL_STATUS(BLUETOOTH_INIT_DONE);
		
		bInitOffset = 0;
	}
	
}

static void Bluetooth_vfnWaitTimerState (void)
{
	if(BLUETOOTH_CHECK_INTERNAL_STATUS(BLUETOOTH_TIMEOUT))
	{
		Bluetooth_tStateMachine.bCurrentState = Bluetooth_tStateMachine.bNextState;
		
		BLUETOOTH_CLEAR_INTERNAL_STATUS(BLUETOOTH_TIMEOUT);
	}
}

static void Bluetooth_vfnExecuteCommandState (void)
{
	uint8_t bStatus;
	
	bStatus = bfnStringCompare(&Bluetooth_gbaState[0], Bluetooth_gbaCommandRxBuffer ,sizeof(Bluetooth_gbaState) - 1U);
	
	if(bStatus == STRING_OK)
	{
		if(BLUETOOTH_DEVICE_CONNECTED == Bluetooth_gbaCommandRxBuffer[BLUETOOTH_BTSTATUS_PARAMETER_OFFSET])
		{
			BLUETOOTH_SET_STATUS(BLUETOOTH_CONNECTED);
			
			BLUETOOTH_CLEAR_STATUS(BLUETOOTH_DATA_READY);
			
			Bluetooth_tStateMachine.bCurrentState = BLUETOOTH_IDLE_STATE;
		}
		else
		{
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
	
	bRxData = UART_bfnRxBuffer();
				
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
