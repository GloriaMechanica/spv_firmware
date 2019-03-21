/** @file debug_uart.h
 *  @brief Provides functions to easily access debug uart
 *
 *  @author Josef Heel
	@date March 19th, 2019
 */

// PROTOTYPES
void print_hello_world (void);
void dbgprintbuf(uint8_t *buf, uint32_t len);
void dbgprintf(const char *fmt, ...);

