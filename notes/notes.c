/** @file notes.c
 *  @brief interfaces to note lever control board via SPI bus
 *
 *  @author Josef Heel
	@date October 17th, 2020
 */

#include "main.h"
#include "debug_tools.h"
#include "device_handles.h"
#include "notes_mapping.h"

#define NUMBER_DRIVER_CARDS 1 // Number of magnet driver cards with 8 bit each

uint8_t notes_state[NUMBER_DRIVER_CARDS]; // global variable for storing magnet data
uint8_t notes_return[NUMBER_DRIVER_CARDS]; // used for the returning SPI data. Not used for anything at the moment

// PROTOTYPES
static void notes_update(void);

/** @brief 	Resets all note levers on all four strings to
 * 			the off (non-touching) position
 *
 *  @param 	(none)
 *  @return (none)
 */
void notes_init(void)
{
	uint8_t i;
	for(i=0; i<NUMBER_DRIVER_CARDS; i++)
	{
		notes_state[i] = 0x00;
	}
	notes_update();
}
/** @brief 	Sets the value of one specific note lever (up->0, down->1)
 * 			on the e-string.
 *
 *  @param note - note number starting with 0 at last
 *  @return (none)
 */
void notes_e_set(uint8_t note)
{
	dbgprintf("E string note set to %d", note);

	uint8_t note_offset = 0xFF;
	if (note >= E_STRING_EMPTY_MIDI_NOTE && note <= E_STRING_MAX_NOTE)
	{
		note_offset = note - E_STRING_EMPTY_MIDI_NOTE;
	}
	else
		return;

	uint8_t board = e_boards[note_offset];
	uint8_t pin = e_pins[note_offset];

	// ONLY FOR PROTOTYPE
	notes_state[board] = 0x00;


	// note has no corresponding magnet. Could be because not ready or because its an empty string
	if (board == NOTE_UNPOPULATED || pin == NOTE_UNPOPULATED || (board + 1 > NUMBER_DRIVER_CARDS || pin > 7))
	{
		// TODO: error handling
	}
	else
	{
		notes_state[board] |= (1<<pin);
	}

	notes_update();
}

/** @brief 	Updates all the magnet drivers by writing notes_state
 * 			over the SPI bus
 *
 *  @param (none)
 *  @return (none)
 */
static void notes_update(void)
{
	HAL_SPI_TransmitReceive(&hspi1, notes_state, notes_return, NUMBER_DRIVER_CARDS, 10);
	HAL_GPIO_WritePin(NOTE_LATCH_GPIO_Port, NOTE_LATCH_Pin, 1);
	HAL_GPIO_WritePin(NOTE_LATCH_GPIO_Port, NOTE_LATCH_Pin, 0);
}

