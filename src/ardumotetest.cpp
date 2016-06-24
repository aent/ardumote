/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <ros/ros.h>

#include <ardumote/PPM.h>

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void handleSetPPM      (ros::Publisher &ppm_publisher);
void handleExit        ();
void handleInvalidValue();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardumotetest");
  
  ros::NodeHandle node_handle;

  /* Create the publisher for sending ppm messages to the arduino */
  
  ros::Publisher ppm_publisher = node_handle.advertise<ardumote::PPM>("/ppm", 10);

  /* Provide a crude menu for selecting which service one wants to invoke */

  char cmd = 0;

  do
  {
    std::cout << std::endl;
    std::cout << "set the [p]pm value of a channel" << std::endl;
    std::cout << "[q]uit" << std::endl;

    std::cout << ">>"; std::cin >> cmd;

    switch(cmd)
    {
    case 'p': handleSetPPM (ppm_publisher);  break;
    case 'q': handleExit                 ();                       break;
    default:  handleInvalidValue         ();                       break;
    }
  } while(cmd != 'q');

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void handleSetPPM(ros::Publisher &ppm_publisher)
{
  std::cout << "Enter the desired channel number for which you want to set the pulse duration: ";
  size_t ppm_channel = 0;
  std::cin >> ppm_channel;
  std::cout << "Enter the desired pulse duration in us: ";
  size_t pulse_duration_us = 0;
  std::cin >> pulse_duration_us;

  ardumote::PPM msg;
  msg.channel           = static_cast<uint8_t>(ppm_channel);
  msg.pulse_duration_us = static_cast<uint16_t>(pulse_duration_us);
 
  ppm_publisher.publish(msg);
}

void handleExit()
{
  std::cout << "Exiting function ardumotetest" << std::endl;
}

void handleInvalidValue()
{
  std::cout << "Invalid input value" << std::endl;
}

