/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
#ifndef SW_TIMER_H_
#define SW_TIMER_H_


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                                         
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Error flag for SW timer
#define SWTIMER_ERROR	(1)
//! Time based in milliseconds. Used to calculate on precompile each SW timer period
#define SWTIMER_BASE_TIME	(1)
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

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section                                    
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus
/*!
 *	@brief	Initializes the SW timer servie
 * 
 *	@param	dwIRQtoEnable		[in]	Interrupt source to enable
 * 	
 * 	@return	void								
 *	
*/
void SWTimer_vfnInit(void);
/*!
 *	@brief	Allocates and configures a SW Timer channel
 * 
 *	@param	dwCounter			[in]	Amount of milliseconds for the timer
 *	
 *	@param	vfnpTimerCallback	[in]	Callback to be executed when the timer reaches zero
 * 	
 * 	@return	uint8_t			Possible error after allocating the channel
 * 	@retval	SWTIMER_ERROR	There are no channels available
 * 	@retval	Any value between 0 and SWTIMER_MAX_TIMERS is a valid channel							
 *	
*/
uint8_t SWTimer_vfnAllocateChannel(uint32_t dwCounter, void (* vfnpTimerCallback)(void));
/*!
 *	@brief	The selected timer is started
 * 
 *	@param	bTimerToEnable			[in]	Timer to start
 *	
 * 	@return	void
 *	
*/
void SWTimer_vfnEnableTimer(uint8_t bTimerToEnable);
/*!
 *	@brief	Stops the selected timer
 * 
 *	@param	bTimerToEnable			[in]	Timer to stop
 *	
 * 	@return	void
 *	
*/
void SWTimer_vfnDisableTimer(uint8_t bTimerToDisable);
/*!
 *	@brief	Changes the selected timer period
 * 
 *	@param	bTimerToEnable			[in]	Timer to update
 *	
 * 	@return	void
 *	
*/
void SWTimer_vfnUpdateCounter(uint8_t bTimerToUpdate, uint32_t dwNewCounter);
/*!
 *	@brief	Main task for the SW timers
 * 
 *	@param	void
 *	
 * 	@return	void
 * 	
 * 	@note This function must be called periodically
 *	
*/
void SWTimer_vfnServiceTimers(void);

#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* SW_TIMER_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
