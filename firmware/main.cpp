#include <ros.h>

#include <Arduino.h>

#include "PPMGenerator.h"

static uint8_t const NUM_PPM_CHANNELS = 5;
static uint16_t const INITIAL_CHANNEL_VALUE_US = 1500;

ros::NodeHandle node_handle;

void setup() 
{
  PPMGenerator::begin(NUM_PPM_CHANNELS);

  PPMGenerator::setPulseWidthUs(0, INITIAL_CHANNEL_VALUE_US);
  PPMGenerator::setPulseWidthUs(1, INITIAL_CHANNEL_VALUE_US);
  PPMGenerator::setPulseWidthUs(2, INITIAL_CHANNEL_VALUE_US);
  PPMGenerator::setPulseWidthUs(3, INITIAL_CHANNEL_VALUE_US);
  PPMGenerator::setPulseWidthUs(4, INITIAL_CHANNEL_VALUE_US);

  node_handle.initNode();
}

void loop() 
{

}
