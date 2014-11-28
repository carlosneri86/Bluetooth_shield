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
#include "MiscFunctions.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


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


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t bfnStringCompare(const uint8_t * bpStringBase, const uint8_t * bpStringToCompare, uint16_t wAmountOfCharacters)
{
	uint8_t bStatus = STRING_OK;
	uint8_t bStringOffset = 0;
	
	while(wAmountOfCharacters--)
	{
		if(bpStringBase[bStringOffset] != bpStringToCompare[bStringOffset])
		{
			bStatus = STRING_ERROR;
			wAmountOfCharacters = 0;
		}
		bStringOffset++;
	}
	
	return (bStatus);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
