/*
 * events.c
 *
 *  Created on: 27 Mar 2020
 *      Author: Sterna
 *
 *  Event lists - Record timings between events
 *  - Register event timing list
 *  - Start/Stop Recording
 *  - Set number of recordings
 *  - Get full list or avg time
 *  -
 */


#include "events.h"

/*
 * Inits an event (clears all data)
 */
void eventInit(eventState_t* event)
{
	memset(event,0,sizeof(eventState_t));
}

/*
 * Handler for a general event debouncing.
 * Event is a pointer to an event state struct (provided and stored by the caller)
 * sourceState is the state of the event to be debounced
 *
 * Todo: Consider if making the events use values instead, to allow for a more rich experience
 */
void eventStateUpdate(eventState_t* event, bool sourceState)
{
	if(sourceState)
	{
		if(event->counter < event->threshold)
		{
			event->counter++;
		}
	}
	else
	{
		if(event->counter>0)
		{
			event->counter--;
		}
	}
	if(event->counter >= event->threshold)
	{
		if(event->active==false)
		{
			event->risingEdgeActive = true;
			event->fallingEdgeActive = false;
			event->activationStartTime=systemTime;
		}
		event->active=true;
	}
	else if(event->counter==0)
	{
		if(event->active==true)
		{
			event->risingEdgeActive = false;
			event->fallingEdgeActive = true;
			event->activationStartTime=UINT32_MAX;
		}
		event->active=false;
	}
}

/*
 * Returns the state of a switch (true if active, false if not)
 * Will return false if the switch does not exist
 */
bool eventGetSate(eventState_t* event)
{
	return event->active;
}

/*
 * Returns true if a rising has occurred
 * Will reset the rising edge state
 */
bool eventGetRisingEdge(eventState_t* event)
{
	bool tmp=event->risingEdgeActive;
	event->risingEdgeActive=false;
	return tmp;
}

/*
 * Returns true if a rising has occurred
 * Will reset the rising edge state
 */
bool eventGetFallingEdge(eventState_t* event)
{
	bool tmp=event->fallingEdgeActive;
	event->fallingEdgeActive=false;
	return tmp;
}

/*
 * Returns true if a switch has been active for more than ms milliseconds
 */
bool eventGetActiveForMoreThan(eventState_t* event, uint32_t ms)
{
	if(event->active==false || systemTime<ms)
	{
		return false;
	}
	else if((systemTime - ms) > event->activationStartTime)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 *	Resets the status of the event
 */
void eventResetEvent(eventState_t* event)
{
	event->active=false;
	event->counter=0;
	event->fallingEdgeActive=false;
	event->risingEdgeActive=false;
	event->activationStartTime=UINT32_MAX;
}

/*
 * Sets the activation timer to now (only if event is actually active)
 * Note that the timer will be reset for all that listens to the event
 */
void eventSetTimerToNow(eventState_t* event)
{
	if(event->active)
	{
		event->activationStartTime=systemTime;
	}
}


//------------------ Timed events ---------------//

/*
 * Inits a timed event struct
 * Will clamp nofEvents to EVENT_TIMED_MAX_EVENTS, if larger
 */
void eventTimedInit(eventTimeList* event, bool stopRecordAuto, uint8_t nofEvents, bool start)
{
	memset(event,0,sizeof(eventTimeList));
	event->stopRecordAutomatically=stopRecordAuto;
	if(nofEvents>EVENT_TIMED_MAX_EVENTS)
	{
		nofEvents=EVENT_TIMED_MAX_EVENTS;
	}
	event->nofEvents=nofEvents;
	if(start)
	{
		eventTimedStartRecording(event);
	}
}

/*
 * Returns true if an event has finished recording (or not started)
 */
bool eventTimedIsRecording(eventTimeList* event)
{
	return event->recordActive;
}

/*
 * Inputs a trig into an event, to record a single event
 * Returns true if recording is finished or if recording recording is not active
 */
bool eventTimedSendTrig(eventTimeList* event,bool start)
{
	if(!eventTimedIsRecording(event))
	{
		if(start)
		{
			eventTimedStartRecording(event);
			return false;
		}
		else
		{
			return true;
		}
	}

	if(event->hasLooped)
	{
		//If an event has looped, we need to remove the oldest sample from max
		event->eventTimesTotal-=event->eventTimes[event->currentEventNum];
	}
	const uint32_t sysTimeTmp=systemTime;	//To avoid having a mismatch if systemTime tick happens
	//Record time
	event->eventTimes[event->currentEventNum]=sysTimeTmp - event->lastEventTime;
	event->lastEventTime=sysTimeTmp;
	event->eventTimesTotal+=event->eventTimes[event->currentEventNum];
	event->currentEventNum++;
	//Calculate current average
	if(event->hasLooped)
	{
		event->avgTime=event->eventTimesTotal/event->nofEvents;
	}
	else
	{
		event->avgTime=event->eventTimesTotal/event->currentEventNum;
	}
	//Check if we're at the end of the buffer
	if(event->currentEventNum>=event->nofEvents)
	{
		//Recording done?
		if(event->stopRecordAutomatically)
		{
			event->recordActive=false;
			return true;
		}
		else
		{
			event->currentEventNum=0;
			event->hasLooped=true;
		}
	}
	return false;
}

/*
 * Trigger a timed recording to start
 */
void eventTimedStartRecording(eventTimeList* event)
{
	event->avgTime=0;
	event->currentEventNum=0;
	event->eventTimesTotal=0;
	event->hasLooped=false;
	event->recordActive=true;
	event->lastEventTime=systemTime;
	//Note: No need to clear the event time list. The old times are just ignored :(
}

/*
 * Stops a recording.
 * Stopping a recording will not record a last point
 */
void eventTimedStopRecording(eventTimeList* event)
{
	event->recordActive=false;
}

/*
 * Recalculates the average time and returns it
 */
uint32_t eventTimedRecalcAndGetAvg(eventTimeList* event)
{
	uint8_t tmpNofEvents=eventTimeGetNofEventsRecorded(event);
	if(tmpNofEvents==0)
	{
		return 0;
	}
	event->eventTimesTotal=0;
	for(uint8_t i=0;i<tmpNofEvents;i++)
	{
		event->eventTimesTotal+=event->eventTimes[i];
	}
	event->avgTime=event->eventTimesTotal/tmpNofEvents;
	return event->avgTime;
}

/*
 * Returns the number of events recorded during the latest recording session
 */
uint8_t eventTimeGetNofEventsRecorded(eventTimeList* event)
{
	uint8_t tmpNofEvents=event->nofEvents;
	if(!event->hasLooped)
	{
		tmpNofEvents=event->currentEventNum;
	}
	return tmpNofEvents;
}
