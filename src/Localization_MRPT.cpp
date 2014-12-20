// -*- C++ -*-
/*!
 * @file  Localization_MRPT.cpp
 * @brief Localization MRPT Component
 * @date $Date$
 *
 * $Id$
 */

#include "Localization_MRPT.h"

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
Localization_MRPT::Localization_MRPT(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_odometryIn("odometry", m_odometry),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_mapServerPort("mapServer")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Localization_MRPT::~Localization_MRPT()
{
}

static OGMap ogmap;

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
  m_mapServerPort.registerConsumer("OGMapServer", "RTC::OGMapServer", m_mapServer);
  
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
{  //Load OGMap
  OGMap_out ogmap = new OGMap();
  if(m_mapServer->requestCurrentBuiltMap(ogmap) == RETVAL_OK){
    return RTC::RTC_OK;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Localization_MRPT::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Localization_MRPT::onExecute(RTC::UniqueId ec_id)
{
  TimedPose2D currentPose;
  RangeData range;

  if(m_odometryIn.isNew()){
	  m_odometryIn.read();
	  currentPose = m_odometry;
  }
  if(m_rangeIn.isNew()){
	  m_rangeIn.read();
	  range = m_range;
  }
  OGMap ogmap;
  //get OGMAP

  COccupancyGridMap2D theMap;
  OGMapToCOccupancyGridMap(ogmap, &theMap);

  double xmin = theMap.getXMin();
  double xmax = theMap.getXMax();
  double ymin = theMap.getYMin();
  double ymax = theMap.getYMax();

  CMonteCarloLocalization2D pdf;
  pdf.resetUniformFreeSpace(&theMap, 0.7, -1, xmin,xmax,ymin,ymax, 0, 2*M_PI);


  pdf.computeResampling(
	  CParticleFilter::TParticleResamplingAlgorithm.prMultinomial,
	  //pdf.m_particles.getW(particles.size()),
	  ,
	  particles.size());
	  
  

  m_estimatedPose = computeEstimatedPose(&ogmap, currentPose, range, &pdf);

  m_estimatedPoseOut.write();

  return RTC::RTC_OK;
}

TimedPose2D computeEstimatedPose(OGMap* map, TimedPose2D currentPose, RangeData range, CMonteCarloLocalization2D* pdf){
	TimedPose2D estimatedPose;
	return estimatedPose;
}

void OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap) {
	gridmap->setSize(0, ogmap.map.width, 0, ogmap.map.height, 1, 0.5f);
	int height = gridmap->getSizeY();
	int width =  gridmap->getSizeX();

	for(int i=0; i <height ; i++){
		for(int j=0; j <width ; j++){
			int cell = ogmap.map.cells[i * width + j];
	
			if(cell < 100){
				gridmap->setCell(j, i, 0.0);
			}
			else if(cell > 200){
				gridmap->setCell(j, i, 1.0);
			}
			else{
				gridmap->setCell(i, j, 0.5);
			}
		}
	}
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


