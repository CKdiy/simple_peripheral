/**
  @headerfile: ibeaconcfg.h
  $Date: 2019-03-21 $
  $Revision:    $
*/

#ifndef IBEACONCFG_H
#define IBEACONCFG_H

/*********************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

/*********************************************************************
 * CONSTANTS
 */
#define MANUFACTURE_DATE_LEN 11
#define DEFAULT_UUID_LEN     16
/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */
typedef struct 
{  
    uint8_t txPower;                          
    uint8_t txInterval;                         
    uint8_t majorValue[2];
    uint8_t minorValue[2];
    uint8_t atFlag;  
	uint8_t Rxp;
    uint8_t uuidValue[16];
    uint8_t hwvr[4];
    uint8_t mDate[11];       
    uint8_t initFlag;          
}ibeaconinf_config_t;

typedef struct 
{
    uint32_t   crc32;
	ibeaconinf_config_t ibeaconinf_config;
}snvinf_t;

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * Observer Profile Public APIs
 */


#endif /* IBEACONCFG_H */
