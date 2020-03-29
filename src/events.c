/*
 * events.c
 *
 *  Created on: 27 Mar 2020
 *      Author: Sterna
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
 * If true is returned, the state will be restarted treating it as if the switch has been released for a very short period of time
 * This means that this function will not return true again until ms milliseconds has passed again
 */
bool eventGetActiveForMoreThan(eventState_t* event, uint32_t ms)
{
	if(event->active==false || systemTime<ms)
	{
		return false;
	}
	else if((systemTime - ms) > event->activationStartTime)
	{
		event->activationStartTime=systemTime;
		return true;
	}
	else
	{
		return false;
	}
}

