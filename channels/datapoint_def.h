/*
 * datapoint_def.h
 *
 *  Created on: Nov 21, 2020
 *      Author: josef
 */

#ifndef DATAPOINT_DEF_H_
#define DATAPOINT_DEF_H_


/*
 * Datapoint in motor axis (X, Y and Z for DAE and GDA)
 */
typedef struct __attribute__((__packed__))
{
	uint32_t timediff;		// relative time difference in ms to last datapoint (or to start timepoint)
	int32_t steps;			// number of steps from calibrated zero-point
}T_DTP_MOTOR;

/*
 * Datapoint for note fingers
 *
 * The note byte denotes the midi note. The ranges for the strings are:
 * G: 55 (empty string) - 65 (highest finger)
 * D: 62 - 73
 * A: 69 - 80
 * E: 76 - 93
 * 255 means no note is played.
 */
typedef struct __attribute__((__packed__))
{
	uint32_t timediff;    	// relative time difference in ms to last datapoint (or to start timepoint)
	uint8_t  note;			// number of note to be pressed on that string, in midi notataion (e.g. 76 is the empty E-string)
}T_DTP_NOTE;


#endif /* DATAPOINT_DEF_H_ */
