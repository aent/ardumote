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

void retrieveParameters(uint8_t *num_channels);
void initPPMGenerator(uint8_t const num_channels);
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
  /* Setup the connection to the ROS */

  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();

  while(!node_handle.connected())
  {
    node_handle.spinOnce();
  }

  /* Initialize the PPM generator library */

  uint8_t num_channels = 0;
  retrieveParameters(&num_channels);

  initPPMGenerator(num_channels);

  /* Subscribe to the topic */

  node_handle.subscribe(ppm_subscriber);
}

void loop() 
{
  node_handle.spinOnce();
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void retrieveParameters(uint8_t *num_channels)
{
  int int_num_channels = 0;
  bool const success = node_handle.getParam("ardumote/num_channels", &int_num_channels);
  if(success)
  {
    *num_channels = int_num_channels;
  }
  else
  {
    *num_channels = DEFAULT_NUM_OF_CHANNELS;
  }
}

void initPPMGenerator(uint8_t const num_channels)
{
  PPMGenerator::begin(num_channels);
  for(uint8_t c = 0; c < num_channels; c++)
  {
    PPMGenerator::setPulseWidthUs(c, INITIAL_CHANNEL_VALUE_US);
  }
}

void PPMCallbackFunction(const ardumote::PPM &ppm_msg)
{
  bool const success = PPMGenerator::setPulseWidthUs(ppm_msg.channel, ppm_msg.pulse_duration_us);
  if(!success)
  {
    node_handle.logerror("Could not set desired pulse width");
  }
}
