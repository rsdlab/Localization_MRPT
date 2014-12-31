// -*- C++ -*-
/*!
 * @file  Localization_MRPT.cpp
 * @brief Localization MRPT Component
 * @date $Date$
 *
 * $Id$
 */

#include "Localization_MRPT.h"
#include "MonteCarloLocalization.h"

// Module specification
// <rtc-template block="module_spec">
static const char* localization_mrpt_spec[] =
  {
    "implementation_id", "Localization_MRPT",
    "type_name",         "Localization_MRPT",
    "description",       "Localization MRPT Component",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigatio",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    // Widget
    "conf.__widget__.debug", "text",
    // Constraints
    ""
  };
// </rtc-template>
/*!
 * @brief constructor
 * @param manager Maneger Object
 */

ssr::Pose2D OldPose;
ssr::MCLocalization_MRPT mcl;

Localization_MRPT::Localization_MRPT(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_odometryIn("odometry", m_odometry),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_mapServerPort("mapServer")
{
}
    // </rtc-template>

/*!
 * @brief destructor
 */
Localization_MRPT::~Localization_MRPT()
{
}

RTC::ReturnCode_t Localization_MRPT::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);
  addInPort("odometry", m_odometryIn);
  
  // Set OutPort buffer
  addOutPort("estimatedPose", m_estimatedPoseOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_mapServerPort.registerConsumer("mapServer", "RTC::OGMapServer", m_mapServer);
  
  // Set CORBA Service Ports
  addPort(m_mapServerPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Localization_MRPT::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
      
RTC::ReturnCode_t Localization_MRPT::onActivated(RTC::UniqueId ec_id)
{
  //Load OGMap
  OGMap* ogmap = new OGMap();
  while (m_mapServerPort.get_connector_profiles()->length() == 0) {
	  coil::sleep(1);
	  std::cout << "[RTC::Localization_MRPT] Waiting for Map Server Connection" << std::endl;
  }
  m_mapServer->requestCurrentBuiltMap(ogmap); 

  //initilize PF
  mcl.setMap(*ogmap);
  mcl.initialize();

  OldPose.x = 0;
  OldPose.y = 0;
  OldPose.th = 0;

  m_odomUpdated = m_rangeUpdated = false;
  return RTC::RTC_OK;

}

RTC::ReturnCode_t Localization_MRPT::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Localization_MRPT::onExecute(RTC::UniqueId ec_id)
{
  if(m_odometryIn.isNew()){
	  m_odometryIn.read();
	  ssr::Pose2D CurrentPose(m_odometry.data.position.x, m_odometry.data.position.y, m_odometry.data.heading);
      ssr::Pose2D deltaPose = CurrentPose - OldPose;	  
	  OldPose = CurrentPose;
	  mcl.addPose(deltaPose);
	  m_odomUpdated = true;
  }
  if(m_rangeIn.isNew()){
	  m_rangeIn.read();
	  ssr::Range range(&(m_range.ranges[0]), m_range.ranges.length(), m_range.config.maxAngle - m_range.config.minAngle);
	  mcl.addRange(range);
	  m_rangeUpdated = true;
  }

  if(m_rangeUpdated && m_odomUpdated) {
    CPose2D estPose;
    estPose = mcl.getEstimatedPose();
   
	m_estimatedPose.data.position.x = estPose.x();
	m_estimatedPose.data.position.y = estPose.y();
	m_estimatedPose.data.heading = estPose.phi();
	
	m_estimatedPoseOut.write();
  }

  return RTC::RTC_OK;
}




/*
RTC::ReturnCode_t Localization_MRPT::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Localization_MRPT::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Localization_MRPTInit(RTC::Manager* manager)
  {
    coil::Properties profile(localization_mrpt_spec);
    manager->registerFactory(profile,
                             RTC::Create<Localization_MRPT>,
                             RTC::Delete<Localization_MRPT>);
  }
  
};


