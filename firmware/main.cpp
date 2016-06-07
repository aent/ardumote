/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Arduino.h>

#include <ros.h>

#include <ardumote/PPM.h>

#include "PPMGenerator.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint16_t const INITIAL_CHANNEL_VALUE_US = 1500;
static uint8_t  const DEFAULT_NUM_OF_CHANNELS  = 5;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void PPMCallbackFunction(const ardumote::PPM &ppm_msg);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ros::NodeHandle node_handle;

ros::Subscriber<ardumote::PPM> ppm_subscriber("/ppm", &PPMCallbackFunction);

/**************************************************************************************
 * ARDUINO FRAMEWORK FUNCTIONS
 **************************************************************************************/

void setup() 
{
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();

  while(!node_handle.connected())
  {
    node_handle.spinOnce();
  }

  int num_channels = 0;
  bool const success = node_handle.getParam("num_channels", &num_channels);
  if(!success)
  {
    num_channels = DEFAULT_NUM_OF_CHANNELS;
  }

  PPMGenerator::begin((uint8_t)(num_channels));
  for(uint8_t c = 0; c < num_channels; c++)
  {
    PPMGenerator::setPulseWidthUs(c, INITIAL_CHANNEL_VALUE_US);
  }

  node_handle.subscribe(ppm_subscriber);
}

void loop() 
{
  node_handle.spinOnce();
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void PPMCallbackFunction(const ardumote::PPM &ppm_msg)
{
  PPMGenerator::setPulseWidthUs(ppm_msg.channel, ppm_msg.pulse_duration_us);
}
