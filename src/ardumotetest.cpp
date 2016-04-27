/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <ros/ros.h>

#include <ardumote/initPPM.h>
#include <ardumote/setPPM.h>

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void handleInitPPMServiceRequest(ros::ServiceClient &initPPM_service_client);
void handlesetPPMServiceRequest (ros::ServiceClient &setPPM_service_client);
void handleExit                 ();
void handleInvalidValue         ();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardumotetest");
  
  ros::NodeHandle node_handle;

  /* Create the two service clients for the two services */
  
  ros::ServiceClient initPPM_service_client = node_handle.serviceClient<ardumote::initPPM>("initPPM");
  ros::ServiceClient setPPM_service_client  = node_handle.serviceClient<ardumote::setPPM> ("setPPM");

  /* Provide a crude menu for selecting which service one wants to invoke */

  char cmd = 0;

  do
  {
    std::cout << std::endl;
    std::cout << "[i]nitPPM service request" << std::endl;
    std::cout << "[s]etPPM service request" << std::endl;
    std::cout << "[q]uit" << std::endl;

    std::cout << ">>"; std::cin >> cmd;

    switch(cmd)
    {
    case 'i': handleInitPPMServiceRequest(initPPM_service_client); break;
    case 's': handlesetPPMServiceRequest (setPPM_service_client);  break;
    case 'q': handleExit                 ();                       break;
    default:  handleInvalidValue         ();                       break;
    }
  } while(cmd != 'q');

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void handleInitPPMServiceRequest(ros::ServiceClient &initPPM_service_client)
{
  std::cout << "Enter the desired number of PPM channels: ";
  size_t num_ppm_channels = 0;
  std::cin >> num_ppm_channels;

  ardumote::initPPM srv;
  srv.request.num_channels = static_cast<uint8_t>(num_ppm_channels);
  
  bool const success = initPPM_service_client.call(srv);

  if(success)
  {
    if(srv.response.success) std::cout << "initPPM SUCCESS" << std::endl;
    else                     std::cout << "initPPM ERROR"   << std::endl;
  }
  else
  {
    std::cout << "initPPM service call failed" << std::endl;
  }
}

void handlesetPPMServiceRequest(ros::ServiceClient &setPPM_service_client)
{
  std::cout << "Enter the desired channel number for which you want to set the pulse duration: ";
  size_t ppm_channel = 0;
  std::cin >> ppm_channel;
  std::cout << "Enter the desired pulse duration in us: ";
  size_t pulse_duration_us = 0;
  std::cin >> pulse_duration_us;

  ardumote::setPPM srv;
  srv.request.channel           = static_cast<uint8_t>(ppm_channel);
  srv.request.pulse_duration_us = static_cast<uint8_t>(pulse_duration_us);
  
  bool const success = setPPM_service_client.call(srv);

  if(success)
  {
    if(srv.response.success) std::cout << "setPPM SUCCESS" << std::endl;
    else                     std::cout << "setPPM ERROR"   << std::endl;
  }
  else
    std::cout << "setPPM service call failed" << std::endl;
  {
  }
}

void handleExit()
{
  std::cout << "Exiting function ardumotetest" << std::endl;
}

void handleInvalidValue()
{
  std::cout << "Invalid input value" << std::endl;
}

