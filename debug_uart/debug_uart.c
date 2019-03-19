

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
#include "debug_uart.h"
#include "string.h"
#include <stdio.h>
#include "device_handles.h"




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
	  HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf),1000);
}
