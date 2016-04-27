/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Arduino.h>

#include <ros.h>

#include <ardumote/initPPM.h>
#include <ardumote/setPPM.h>

#include "PPMGenerator.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint16_t const INITIAL_CHANNEL_VALUE_US = 1500;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void initPPMCallbackFunction(ardumote::initPPM::Request const &req, ardumote::initPPM::Response &res);
void setPPMCallbackFunction(ardumote::setPPM::Request const &req, ardumote::setPPM::Response &res);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ros::NodeHandle node_handle;

ros::ServiceServer<ardumote::initPPM::Request, ardumote::initPPM::Response> service_server_init_ppm("initPPM", &initPPMCallbackFunction);
ros::ServiceServer<ardumote::setPPM::Request, ardumote::setPPM::Response> service_server_set_ppm("setPPM", &setPPMCallbackFunction);

/**************************************************************************************
 * ARDUINO FRAMEWORK FUNCTIONS
 **************************************************************************************/

void setup() 
{
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();

  node_handle.advertiseService(service_server_init_ppm);
  node_handle.advertiseService(service_server_set_ppm);
}

void loop() 
{
  node_handle.spinOnce();
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void initPPMCallbackFunction(ardumote::initPPM::Request const &req, ardumote::initPPM::Response &res)
{
  bool success = PPMGenerator::begin(req.num_channels);

  for(uint8_t i = 0; i < req.num_channels; i++)
  {
    if(!PPMGenerator::setPulseWidthUs(i, INITIAL_CHANNEL_VALUE_US))
    {
      success = false;
    }
  }

  res.success = success;
}

void setPPMCallbackFunction(ardumote::setPPM::Request const &req, ardumote::setPPM::Response &res)
{
  res.success = PPMGenerator::setPulseWidthUs(req.channel, req.pulse_duration_us);
}
