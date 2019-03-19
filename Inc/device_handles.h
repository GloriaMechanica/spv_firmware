/** @file debug_uart.h
 *  @brief Can be included in other files to access the hardware
 *  device handles that CubeMX generates in the main.
 *
 *  If some new device is added via CubeMX, it has to be added here as well
 *  with extern!
 *
 *  This solution is kinda dirty, but what to do when CubeMX generates
 *  everything in the main...
 *
 *  @author Josef Heel
	@date March 19th, 2019
 */

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart3;
