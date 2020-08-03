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
 *
 * Note: The reset-timer feature should only be used if you're sure you're the subscriber with the longest time frame
 */

#define EVENT_TIMED_MAX_EVENTS	10

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

/*
 * Definition for an event timing list.
 */
typedef struct
{
	bool recordActive;				//Indicates if a recording is currently active
	bool stopRecordAutomatically;	//Will stop recording when the number of events has filled up
	bool hasLooped;					//Indicates if the number of recorded events has passed the set number of events at least once
	uint8_t nofEvents;				//The number of events to record
	uint32_t lastEventTime;			//The last time an event was recorded
	uint8_t currentEventNum;		//The counter for the current event
	uint32_t eventTimes[EVENT_TIMED_MAX_EVENTS];	//List of event times
	uint32_t eventTimesTotal;		//The total time for all events
	uint32_t avgTime;				//The average time for events
}eventTimeList;


void eventInit(eventState_t* event);
void eventStateUpdate(eventState_t* event,bool sourceState);
bool eventGetSate(eventState_t* event);
bool eventGetRisingEdge(eventState_t* event);
bool eventGetFallingEdge(eventState_t* event);
bool eventGetActiveForMoreThan(eventState_t* event, uint32_t ms);
void eventResetEvent(eventState_t* event);
void eventSetTimerToNow(eventState_t* event);

void eventTimedInit(eventTimeList* event, bool stopRecordAuto, uint8_t nofEvents, bool start);
bool eventTimedSendTrig(eventTimeList* event,bool start);
void eventTimedStartRecording(eventTimeList* event);
void eventTimedStopRecording(eventTimeList* event);
bool eventTimedIsRecording(eventTimeList* event);
uint32_t eventTimedRecalcAndGetAvg(eventTimeList* event);
uint8_t eventTimeGetNofEventsRecorded(eventTimeList* event);

#endif /* STM32UTILS_INCLUDE_EVENTS_H_ */
