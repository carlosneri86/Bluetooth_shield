/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                                         
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section                                     
///////////////////////////////////////////////////////////////////////////////////////////////////

#define HEARTBEAT_PIN			(19)

#define HEARTBEAT_MASK			(1<<HEARTBEAT_PIN)

#define HEARTBEAT_TOGGLE_PORT	(GPIOB_PTOR)

#define HEARTBEAT_TIMEOUT		(5000)
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

void Heartbeat_vfnInit(void);

#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* HEARTBEAT_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////