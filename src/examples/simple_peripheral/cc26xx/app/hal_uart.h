/**************************************************************************************************
  Filename:       hal_uart.h
  Revised:        $Date: 2019-03-13  $
  Revision:       $Revision:   $

**************************************************************************************************/

#ifndef HAL_UART_H
#define HAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/*********************************************************************
 * DEFINE
 */

#define UART0_RECEICE_BUFF_SIZE 1<<6
#define RX_BUFF_SIZE 256
  
/*********************************************************************
 * MACROS
 */
  

/*********************************************************************
 * FUNCTIONS
 */
void Close_uart0(void);
extern bool Open_uart0(UART_Callback appuartCB);
bool Uart0_Write(const uint8_t *wbuf, size_t wlen);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
