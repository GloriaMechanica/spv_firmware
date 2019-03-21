

/** @file debug_uart.c
 *  @brief Provides functions to easily access debug uart
 *
 *  For debugging purposes, uart3 is used. It can be directly
 *  accessed through HAL, but here some functions are provided
 *  to more easily do what one wants to do with a debug uart.
 *
 *  @author Josef Heel
	@date March 19th, 2019
 */

#include "main.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>
#include "device_handles.h"
#include "debug_tools.h"

// PRIVATE DEFINES
#define 	DEBUG_UART_HANDLE			&huart3		// Handle of the uart to be used as debug uart
#define 	DEBUG_UART_TX_BUFFER_SIZE	256  		// string of dbgprintf must not be longer than that
#define		DEBUG_UART_TX_TIMEOUT		1000 		// [ms]. Pretty useless, but driver needs that



/** @brief prints out a standard printf-type format char over debug uart
 * 			and adds a \n, because you always foget that...
 * 			However, be careful, it only takes up to DEBUG_UART_TX_BUFFER_SIZE
 * 			characters, otherwise it returns and does nothing!
 *
 *  @param fmt - format string of printf-type
 *  @return (none)
 */
void dbgprintf(const char *fmt, ...)
{
	char buf[DEBUG_UART_TX_BUFFER_SIZE];

	// Get additional arguments and pass over to sprintf
	va_list arg_ptr;
	va_start(arg_ptr, fmt);
	vsprintf(buf, fmt, arg_ptr);
	va_end(arg_ptr);
	strcat(buf, "\n");

	HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t*) buf, strlen(buf), DEBUG_UART_TX_TIMEOUT);
}

/** @brief Prints out some bytes of a buffer directly on
 * 			the debug uart, inside a little frame
 *
 *  @param buf - buffer of data bytes
 *  @param len - number of bytes to be read out of buffer
 *  @return (none)
 */
void dbgprintbuf(uint8_t *buf, uint32_t len)
{
	dbgprintf("--- BEGIN DATA ---");
	HAL_UART_Transmit(DEBUG_UART_HANDLE, buf, len, DEBUG_UART_TX_TIMEOUT);
	dbgprintf("\n--- END DATA ---");
}

/** @brief Prints character ch at the current location
 *         of the cursor.
 *
 *  The first function to be implemented (therefore of historic value!)
 *  Just prints out Hello World on the debug uart.
 *
 *  @param (none)
 *  @return (none)
 */
void print_hello_world (void)
{
	  char buf[30];
	  sprintf(buf,"Hello World says SPV!\n");
	  HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t*)buf, strlen(buf), DEBUG_UART_TX_TIMEOUT);
}

/** @brief toggles blue led on NUCLEO board
 *
 *  @param (none)
 *  @return (none)
 */
void toggle_debug_led (void)
{
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}







