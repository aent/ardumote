#ifndef PPMGENERATOR_H_
#define PPMGENERATOR_H_

/************************************************************************
 * INCLUDES                                                            
 ************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/************************************************************************
 * PUBLIC CONSTANTS
 ************************************************************************/

static uint8_t const MAX_NUM_PPM_CHANNELS = 8;

/************************************************************************
 * PROTOTYPES                                                            
 ************************************************************************/

class PPMGenerator
{
public:

	/** 
	 * \brief initialize the PPM generator
	 */
	static bool begin(uint8_t const number_of_channels);
	
	/** 
	 * \brief set the pulse duration in microseconds
	 */
	static bool setPulseWidthUs(uint8_t const channel, uint16_t const pulse_duration_us);	
	
private:

	/**
	 * \brief no public constructing
	 */
	PPMGenerator() { }	
};

#endif /* PPMGENERATOR_H_ */
