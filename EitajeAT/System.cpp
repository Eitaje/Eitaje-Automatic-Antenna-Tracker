#include "System.h"


/**
Updates the smart timer struct
*/

void System::updateSmartTimer(System::SmartTimer * timer)
{
	timer->now = micros();

	// Set integration time by time elapsed since last filter update
	timer->deltat = ((timer->now - timer->lastUpdate) / 1000.0f);
}

