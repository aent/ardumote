/************************************************************************
 * INCLUDES                                                            
 ************************************************************************/

#include "PPMGenerator.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/************************************************************************
 * DEFINES
 ************************************************************************/

/* PPM = D7 = PD7 */
#define PPM_DDR			(DDRD)
#define PPM_PORT		(PORTD)
#define PPM_bm			(1<<7)

/* PPM = D6 = PD6 */
/* This output is used to provide a trigger signal for an oscilloscope */
#define TRIGGER_DDR		(DDRD)
#define TRIGGER_PORT	(PORTD)
#define TRIGGER_bm		(1<<6)

/************************************************************************
 * INTERNAL GLOBAL CONSTANTS
 ************************************************************************/

/* fClk = 16 MHz
 * fTimer = 2 MHz -> tTimer = 0.5 us -> Prescaler = 8
 * TTimer = 20 ms -> Steps per TTimer = 20 ms / 0.5 us = 40000
 * TimerReloadValue = 0xFFFF (65535) - 40000 = 0x63BF (25535)
 */
static uint16_t const TIMER_RELOAD_VALUE = 0x63BF;

/* A single PPM pulse has an duration of 500 us */
static uint16_t const PPM_PULSE_DURATION_US = 500;

/************************************************************************
 * INTERNAL GLOBAL VARIABLES
 ************************************************************************/

static uint16_t _channel_pulse_duration_us[MAX_NUM_PPM_CHANNELS] = { 1500 };
static uint8_t _num_ppm_channels = 0;
static uint8_t _current_ppm_channel = 0;

/************************************************************************
 * PRIVATE FUNCTIONS
 ************************************************************************/

void initPPM() { PPM_DDR |= PPM_bm; }
void setPPM() { PPM_PORT |= PPM_bm; }
void clearPPM() { PPM_PORT &= ~PPM_bm; }
	
void initTRIGGER() { TRIGGER_DDR |= TRIGGER_bm; }
void setTRIGGER() { TRIGGER_PORT |= TRIGGER_bm; }
void clearTRIGGER() { TRIGGER_PORT &= ~TRIGGER_bm; }

/************************************************************************
 * PUBLIC FUNCTIONS
 ************************************************************************/

/** 
 * \brief initialize the PPM generator
 */
bool PPMGenerator::begin(uint8_t const number_of_channels)
{
	if(number_of_channels > MAX_NUM_PPM_CHANNELS)
	{
		/* Return false if there are more channels requested than its possible
		 * to squeeze into a single PPM frame
		 */
		return false;
	}
	else
	{
		/* Keep the number of desired ppm channels for later reuse */
		_num_ppm_channels = number_of_channels;
		
		/* Setup the digital outputs ports */
		
		initPPM();
		setPPM();
		
		initTRIGGER();
		clearTRIGGER();
		
		/* Setup the timer */
		
		/* Operate in normal timer mode, Top = 0xFFFF */
	
		TCCR1A = 0;
	
		/* Ensure that TCCR1C is set to reset value (Arduino IDE might
		 * corrupt this registers at a future point in time because
		 * it needs the registers for its own purposes
		 */
	
		TCCR1C = 0;
	
		/* Set the Timer/Counter register of Timer 1 to the precalculated
		 * reload value.
		 */
	
		TCNT1 = TIMER_RELOAD_VALUE;
	
		/* Enable both output compare interrupts as well as the timer
		 * overflow interrupt 
		 */
	
		TIMSK1 = (1<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1);
	
		/* Set prescaler to 8. 
		 * fClk = 16 MHz
		 * fTimer = fClk / Prescaler = 16 MHz / 8 = 2 MHz
		 * 1 Timerstep = 1 / fTimer = 1 / 2 MHz = 0.5 us
		 * Timer is active now.
		 */
		
		TCCR1B = (1<<CS11);
		
		return true;
	}
}
	
/** 
 * \brief set the pulse duration in microseconds
 */
bool PPMGenerator::setPulseWidthUs(uint8_t const channel, uint16_t const pulse_duration_us)
{
	if(channel < _num_ppm_channels)
	{
		_channel_pulse_duration_us[channel] = pulse_duration_us;
		
		return true;
	}
	else
	{
		return false;
	}
}

/************************************************************************/
/* INTERRUPT SERVICE HANDLERS                                           */
/************************************************************************/

/** 
 * \brief Interrupt Service Routine for Timer 1 - this function is 
 * executed every 20 ms
 */
ISR(TIMER1_OVF_vect)
{
	/* Reload the Timer/Counter register with the correct reload value */
	
	TCNT1 = TIMER_RELOAD_VALUE;
	
	/* Reset the variable _current_ppm_channel. This variable is used
	 * within the compare match A interrupts to determine which pause
	 * time between two consecutive impulses need to be kept
	 */
	
	_current_ppm_channel = 0;
	
	/* Set Output */
	
	clearPPM();
	
	/* Set Trigger */
	
	setTRIGGER();
	
	/* Load OCR1A and OCR1AB with their next values */
	
	OCR1A = TCNT1 + (_channel_pulse_duration_us[_current_ppm_channel] * 2);
	OCR1B = TCNT1 + (PPM_PULSE_DURATION_US * 2);
}

/** 
 * \brief Interrupt service routine for compare match a interrupt
 */
ISR(TIMER1_COMPA_vect)
{
	/* Clear Output */
	
	clearPPM();
	
	/* Increase counter */
	
	_current_ppm_channel++;
	
	/* Clear trigger signal if this is the first pulse */
	
	bool const is_first_pulse = _current_ppm_channel == 1;
	
	if(is_first_pulse)
	{
		clearTRIGGER();	
	}
	
	/* Load OCR1A and OCR1AB with their next values */
	
	bool const is_last_pulse = _current_ppm_channel == _num_ppm_channels;
	
	if(!is_last_pulse)
	{
		OCR1A = TCNT1 + (_channel_pulse_duration_us[_current_ppm_channel] * 2);
	}

	OCR1B = TCNT1 + (PPM_PULSE_DURATION_US * 2);
}

/** 
 * \brief Interrupt service routine for compare match b interrupt
 */
ISR(TIMER1_COMPB_vect)
{
	/* Set output */
	
	setPPM();
}
