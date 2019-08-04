

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
#include "usb_cdc_comm.h"

// PRIVATE DEFINES
#define 	DEBUG_UART_HANDLE			&huart3		// Handle of the uart to be used as debug uart
#define 	DEBUG_UART_TX_BUFFER_SIZE	256  		// string of dbgprintf must not be longer than that
#define		DEBUG_UART_TX_TIMEOUT		5000 		// [ms]. Pretty useless, but driver needs that

// Used to save and export the real motor movements on the PC
#define 	DEBUG_MOTOR_TRACKING_BUFFER_SIZE		4096		// Size of big timer preload value array
#define 	DEBUG_MOTOR_TRACKING_MAX_BLOCK_SIZE		256		// How many bytes to transmit with one hit
uint16_t 	debug_preload_buffer[DEBUG_MOTOR_TRACKING_BUFFER_SIZE];
int32_t 	debug_motor_tracking_input_pointer; 				// pointing to the next free buffer word
int32_t 	debug_motor_tracking_output_pointer; 				// pointing to the next word to read out
uint32_t 	debug_motor_tracking_running; 						// Flag whether timer preload values should be output via USB
uint32_t 	debug_motor_tracking_drop_counter; 					// Counts how many timer values had to be dropped because they could not be emptied fast enough



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

void cpu_load_pin_on (void)
{
    CPU_LOAD_GPIO_Port->BSRR = CPU_LOAD_Pin;
}

void cpu_load_pin_off (void)
{
    CPU_LOAD_GPIO_Port->BSRR = (uint32_t)CPU_LOAD_Pin << 16;
}


/** @brief  Initializes the timer preload debug output functions and starts it.
 *  		This function is used to directly print each timer preload over the
 *  		USB-CDC connection. With the "real" hardware timer preloads, it is
 *  		easier to find out if something in the stepper engine is going wrong.
 *
 *  @param  (none)
 *  @return (none)
 */
void debug_start_motor_tracking (void)
{
	debug_motor_tracking_input_pointer = 0;  	// start and stop are the same-> buffer is empty
	debug_motor_tracking_output_pointer = 0;
	debug_motor_tracking_running = 1; 			// start outputting values
}

/** @brief  Stops the motor speed tracking function
 *
 *  @param  (none)
 *  @return (none)
 */
void debug_stop_motor_tracking (void)
{
	debug_motor_tracking_running = 0;
}

/** @brief 	Saves the current timer preload value (c_hw, to be precise) on a buffer
 * 			From this buffer, it will be spit out over the USB CDC UART from the main loop
 * 			housekeeper.
 *
 * 			If the buffer should overflow, it drops the values and writes an error
 * 			over the debug uart
 *
 *  @param c_hw - 16 bit timer preload value (number of ticks to the next step.
 *  @return (none)
 */
void debug_push_preload(uint16_t preload)
{
	int32_t words_in_buffer = debug_motor_tracking_input_pointer - debug_motor_tracking_output_pointer;
	if (words_in_buffer < 0)
		words_in_buffer += DEBUG_MOTOR_TRACKING_BUFFER_SIZE; // If input overflowed, but output did not so far

	if (debug_motor_tracking_running == 1)
	{
		if (words_in_buffer < DEBUG_MOTOR_TRACKING_BUFFER_SIZE - 1) // One byte less, because input==output means empty
		{
			// There is still space in the buffer
			debug_preload_buffer[debug_motor_tracking_input_pointer] = preload;
			debug_motor_tracking_input_pointer = (debug_motor_tracking_input_pointer + 1) % DEBUG_MOTOR_TRACKING_BUFFER_SIZE; // Increment and
		}
		else
		{
			// There was no space in the buffer.
			debug_motor_tracking_drop_counter++;
		}
	}
}

/** @brief 	Function to be periodically called by the main loop for clearing out the
 * 			data accumulated in the preload buffer.
 *
 *  @param c_hw - 16 bit timer preload value (number of ticks to the next step.
 *  @return (none)
 */
void debug_transmit_motor_tracking_data (void)
{
	if (debug_motor_tracking_running == 1)
	{
		// Only do something if this feature is activated

		// Calculate how many words are in the buffer
		int32_t words_in_buffer = debug_motor_tracking_input_pointer - debug_motor_tracking_output_pointer;
		int32_t words_to_transmit;

		if (words_in_buffer < 0)
			words_in_buffer += DEBUG_MOTOR_TRACKING_BUFFER_SIZE; // If input overflowed, but output did not so far

		// There is something in the buffer to print
		if (words_in_buffer > 0)
		{
			// Transmit what's there, but a maximum of the DEBUG_MOTOR_TRACKING_MAX_BLOCK_SIZE
			words_to_transmit = words_in_buffer;
			if (words_to_transmit > DEBUG_MOTOR_TRACKING_MAX_BLOCK_SIZE)
				words_to_transmit = DEBUG_MOTOR_TRACKING_MAX_BLOCK_SIZE;

			// And actually transmit via USB CDC UART
			// Two cases because data region is split when input overflowed but output did not yet
			if (debug_motor_tracking_input_pointer > debug_motor_tracking_output_pointer)
			{
				// data region is one block
				uint16_t *start_pointer = &(debug_preload_buffer[debug_motor_tracking_output_pointer]);
				uint32_t data_length = words_to_transmit * sizeof(uint16_t);

				USB_CDC_TransmitBuffer((uint8_t*)start_pointer, data_length);
			}
			else
			{
				// region is split. one goes from output_pointer to top and one from 0 to input-pointer -1
				// first chunk from output_pointer to top
				uint16_t *start_pointer = &(debug_preload_buffer[debug_motor_tracking_output_pointer]);
				uint32_t words_in_top_part = DEBUG_MOTOR_TRACKING_BUFFER_SIZE - debug_motor_tracking_output_pointer;
				if (words_in_top_part > words_to_transmit)
					words_in_top_part = words_to_transmit;
				uint32_t data_length =  words_in_top_part * sizeof(uint16_t);

				USB_CDC_TransmitBuffer((uint8_t*)start_pointer, data_length);

				// second chunk from 0 upwards containing the remaining values
				start_pointer = &(debug_preload_buffer[0]);
				data_length = (words_to_transmit - words_in_top_part) * sizeof(uint16_t);

				if (data_length > 0)
					USB_CDC_TransmitBuffer((uint8_t*)start_pointer, data_length);
			}

			// And increment output pointer accordingly
			debug_motor_tracking_output_pointer = (debug_motor_tracking_output_pointer + words_to_transmit) % DEBUG_MOTOR_TRACKING_BUFFER_SIZE;
		}
	}

	// The ISR could not write one or more values, so the data is not continuous
	if (debug_motor_tracking_drop_counter > 0)
	{
		dbgprintf("##### MOTOR TRACKING ERRROR BUFFER OVERFLOW (%d words) !!! #####", debug_motor_tracking_drop_counter);
		debug_motor_tracking_drop_counter = 0;
	}

}






