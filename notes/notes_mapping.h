/** @file notes_mapping.h
 *  @brief 	Defines which notes are on which pins of the magnet
 *  		drivers.
 *
 *  A brief explaination:
 *  Notes are numbered on each string, starting with the lowest note 0,
 *  which is the empty string without any lever.
 *  e.g. 0 on the e-string means an the note E.
 *
 *  Then, every half tone gets an increasing number:
 *  1 = F
 *  2 = F#
 *  3 = G
 *  4 = G#
 *  5 = ...
 *  @author Josef Heel
	@date October 17th, 2020
 */

#ifndef NOTES_MAPPING_H_
#define NOTES_MAPPING_H_

#define E_STRING_EMPTY_MIDI_NOTE 76
#define E_STRING_MAX_NOTE 93


#define NOTES_ON_G (10+1) // Means there are 10 levers on the G-string, but the empty string is also one note
#define NOTES_ON_D (11+1)
#define NOTES_ON_A (11+1)
#define NOTES_ON_E (17+1)

#define NOTE_UNPOPULATED 0xFF
static const int e_pins[NOTES_ON_E] = {0xFF, 0xFF, 0, 0xFF, 1, 2, 0xFF, 3, 0xFF, 4, 0xFF, 5, 6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int e_boards[NOTES_ON_E] = {0, 0xFF, 0, 0xFF, 0, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


#endif /* NOTES_MAPPING_H_ */
