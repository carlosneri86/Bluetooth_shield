/*
 * main implementation: use this 'C' sample to create your own application
 *
 */





#include "derivative.h" /* include peripheral declarations */
#include "BoardConfig.h"
#include "Heartbeat.h"
#include "SW_Timer.h"
#include "UART.h"
#include "Bluetooth.h"


#define MAX_BT	(200)

uint8_t gbaTestString1[] =
{
	"Test bt"	
		
};

uint8_t gbaRxBuffer[MAX_BT];

uint8_t gbRxOffset = 0;

uint8_t gbRollOverCounter = 0;

int main(void)
{
	BoardConfig_vfnInit();
	SWTimer_vfnInit();
	Heartbeat_vfnInit();
	Bluetooth_vfnInitModule();
	

	
	for(;;) 
	{	   
		SWTimer_vfnServiceTimers();
		Bluetooth_vfnStateMachine();
		
		if(BLUETOOTH_CHECK_STATUS(BLUETOOTH_CONNECTED))
		{
			if(BLUETOOTH_CHECK_STATUS(BLUETOOTH_DATA_READY))
			{
				gbaRxBuffer[gbRxOffset] = Bluetooth_bfnReadData();
				
				Bluetooth_vfnWriteData(&gbaRxBuffer[gbRxOffset], 1U);
				
				gbRxOffset++;
				
				if(gbRxOffset > MAX_BT)
				{
					gbRxOffset = 0;
					gbRollOverCounter++;
				}
			}
		}
	}
	
	return 0;
}
