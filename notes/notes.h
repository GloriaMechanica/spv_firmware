/** @file notes.h
 *  @brief interfaces to note lever control board via SPI bus
 *
 *  @author Josef Heel
	@date October 17th, 2020
 */
#ifndef NOTES_H_
#define NOTES_H_

void notes_init(void);
void notes_e_set(uint8_t note);


#endif /* NOTES_H_ */
