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
void toggle_debug_led (void);

void cpu_load_pin_on (void);
void cpu_load_pin_off (void);
void isr_load_pin_on (void);
void isr_load_pin_off (void);

void debug_start_motor_tracking (void);
void debug_stop_motor_tracking (void);
void debug_indicate_cycle_start(uint16_t delta_s, uint16_t delta_t);
void debug_push_preload(uint16_t preload);
void debug_transmit_motor_tracking_data (void);
