/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
#ifndef LPTIMER_H_
#define LPTIMER_H_


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                                         
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "BoardConfig.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Typedef Section                                          
///////////////////////////////////////////////////////////////////////////////////////////////////
//! HW Prescaler values
typedef enum
{
	LPTIMER_PRESCALER_BY_2 = 0,
	LPTIMER_PRESCALER_BY_4,
	LPTIMER_PRESCALER_BY_8,
	LPTIMER_PRESCALER_BY_16,
	LPTIMER_PRESCALER_BY_32,
	LPTIMER_PRESCALER_BY_64,
	LPTIMER_PRESCALER_BY_128,
	LPTIMER_PRESCALER_BY_256,
	LPTIMER_PRESCALER_BY_512,
	LPTIMER_PRESCALER_BY_1024,
	LPTIMER_PRESCALER_BY_2048,
	LPTIMER_PRESCALER_BY_4096,
	LPTIMER_PRESCALER_BY_8192,
	LPTIMER_PRESCALER_BY_16384,
	LPTIMER_PRESCALER_BY_32768,
	LPTIMER_PRESCALER_BY_65536
}eLPTimerPresacaler;
//! HW clock selections
typedef enum
{
	LPTIMER_PRESCALER_CLOCK_0 = 0,
	LPTIMER_PRESCALER_CLOCK_1,
	LPTIMER_PRESCALER_CLOCK_2,
	LPTIMER_PRESCALER_CLOCK_3
}eLPTimerClockSelect;
//! Event flag
typedef enum
{
	LPTIMER_COUNTER_MATCH_STATUS = 0
}eLPTimerStatus;
//! Event flag mask
typedef enum
{
	LPTIMER_COUNTER_MATCH_MASK_STATUS = (1 << LPTIMER_COUNTER_MATCH_STATUS)
}eLPTimerStatusMasks;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function-like Macros Section                                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Set asks for the timer event
#define LPTIMER_CHECK_STATUS(status)	(LPTimer_gbStatus & status)
//! Clear the timer event
#define LPTIMER_CLEAR_STATUS(status)	(LPTimer_gbStatus &= ~status)
//! Set the timer event
#define LPTIMER_SET_STATUS(status)	(LPTimer_gbStatus |= status)
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Constants Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Variables Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Timer events
extern volatile uint8_t LPTimer_gbStatus;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section                                    
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus
/*!
 *	@brief	Initialize the HW LPTimer
 * 
 *	@param	bPrescaler		[in]	Prescaler configuration, dependent of the user calculations. \
 *									From eLPTimerPresacaler enum
 *	
 *	@param  bClockSource	[in]	Clock source to be used. Refer to the HW manual for further \
 *									details. From eLPTimerClockSelect enum
 *	
 *	@param	dwCompareValue	[in]	Timer modulo. Calculated by the user. Directly dependent of \
 *									the clock source and prescaler
 * 	
 * 	@return	void							
 *	
*/
void LPTimer_Init(uint8_t bPrescaler,uint8_t bClockSource, uint32_t dwCompareValue);
/*!
 *	@brief	Starts the LPTimer
 * 
 * 	@return	void							
 *	
*/
void LPTimer_EnableTimer (void);

#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* LPTIMER_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
