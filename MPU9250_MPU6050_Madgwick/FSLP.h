#ifndef FSLP_h
#define FSLP_h

#include "Arduino.h"

class FSLP
{
	public:
	int fslpGetPosition(uint8_t fslpSenseLine, uint8_t fslpDriveLine1, uint8_t fslpDriveLine2, uint8_t fslpBotR0);
	float fslpGetPressure(uint8_t fslpSenseLine, uint8_t fslpDriveLine1, uint8_t fslpDriveLine2, uint8_t fslpBotR0);
};

#endif
