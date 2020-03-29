/*
 * events.h
 *
 *  Created on: 27 Mar 2020
 *      Author: Sterna
 */

#ifndef STM32UTILS_INCLUDE_EVENTS_H_
#define STM32UTILS_INCLUDE_EVENTS_H_


#include <stdbool.h>
#include <stdint.h>
#include <string.h>	//For memset
#include "time.h"

/*
 * How to use:
 * Create your variable with the eventState_t type
 * Run init and set a threshold for the event. This threshold is the debouncing threshold and is the speed of the filter
 * Whenever the event is evaluated to true or false, pass the event-struct and the result to the eventStateUpdate-function.
 * Suggestion is to do this periodically.
 * To read an event state, use any of the provided functions. You CAN read the event-active state directly from the struct,
 * 	but no other information is recommended to read directly.
 */

/*
 * Definition for a general event debouncing
 */
typedef struct
{
	uint8_t counter;				//Counter to debounce the sample
	uint8_t threshold;				//The threshold when the event is considered triggered
	bool active;					//Indicates if the event is currently active
	bool fallingEdgeActive;			//Indicates if a falling edge has happened. Will be reset once read.
	bool risingEdgeActive;			//Indicates if a rising edge has happened. Will be reset once read.
	uint32_t activationStartTime;	//Records the systemTime at which the switch had a rising edge
}eventState_t;


void eventInit(eventState_t* event);
void eventStateUpdate(eventState_t* event,bool sourceState);
bool eventGetSate(eventState_t* event);
bool eventGetRisingEdge(eventState_t* event);
bool eventGetFallingEdge(eventState_t* event);
bool eventGetActiveForMoreThan(eventState_t* event, uint32_t ms);

#endif /* STM32UTILS_INCLUDE_EVENTS_H_ */
