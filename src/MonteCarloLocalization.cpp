#include "MonteCarloLocalization.h"

using namespace ssr;


MCLocalization_MRPT::MCLocalization_MRPT(){
	//constractor 
}

MCLocalization_MRPT::~MCLocalization_MRPT(){
	//destractor 
}

void  MCLocalization_MRPT::setMap(const OGMap& map){
	OGMap2COccupancyGridMap(map, &m_ogmap);
}
void  MCLocalization_MRPT::initialize(){
    pdf_.options.metricMap = &m_ogmap;
	m_ActionCollection.clear();
	m_SensoryFrame.clear();

	//////////////////////////////
	//have to create MRPT config file
	//////////////////////////////

	//pfOptions.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm.pfStandardProposal;
	//pfOptions.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm.prMultinomial;
	pfOptions.adaptiveSampleSize = 1;
	pfOptions.pfAuxFilterOptimal_MaximumSearchSamples = 10;
	pfOptions.BETA = 0.;
	pfOptions.sampleSize = 1;

	pdf_.options.KLD_params.KLD_binSize_PHI=10;
	pdf_.options.KLD_params.KLD_binSize_XY=0.10;
	pdf_.options.KLD_params.KLD_delta = 0.01;
	pdf_.options.KLD_params.KLD_epsilon =0.01;
	pdf_.options.KLD_params.KLD_maxSampleSize = 40000;
	pdf_.options.KLD_params.KLD_minSampleSize = 150;
	pdf_.options.KLD_params.KLD_minSamplesPerBin = 0;
		
	motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
	motion_model_options_.gausianModel.minStdXY  = 0.10;
	motion_model_options_.gausianModel.minStdPHI = 2.0;

	/////////
	/////////

	pf_.m_options = pfOptions;

	//initialize PDF
	//
    //double xmin = -100;
    //double xmax =  100;
    //double ymin = -100;
    //double ymax =  100;
	pdf_.resetUniformFreeSpace(&m_ogmap, 0.7f, -1);

}

bool MCLocalization_MRPT::addPose(const ssr::Pose2D& deltaPose)
{
	CActionRobotMovement2D action;
	CActionRobotMovement2D::TMotionModelOptions options;
	action.computeFromOdometry(CPose2D(deltaPose.x, deltaPose.y, deltaPose.th), options);
	action.timestamp = mrpt::system::getCurrentTime();
	static TTimeStamp oldTimestamp;
	if(action.timestamp == oldTimestamp) {
		action.timestamp = oldTimestamp +1;
	}
	oldTimestamp = action.timestamp;
	m_ActionCollection.insert(action);
	return true;
}

bool MCLocalization_MRPT::addRange(const ssr::Range& range)
{
	CObservation2DRangeScanPtr observation = CObservation2DRangeScan::Create();
	observation->rightToLeft = true;
	observation->validRange.resize(range.size);
	observation->scan.resize(range.size);
	observation->aperture = range.aperture;
	observation->timestamp = mrpt::system::getCurrentTime();
	for(int i = 0;i < range.size; i++) {
		observation->scan[i] = range.range[i];
		if(observation->scan[i] > m_range_min && observation->scan[i] < m_range_max) {
			observation->validRange[i] = 1;
		} else {
			observation->validRange[i] = 0;
		}
	}
	observation->setSensorPose(m_RangeSensorPose);
	m_SensoryFrame.insert(observation);
	return true;
}

CPose2D MCLocalization_MRPT::getEstimatedPose(){
  CPose2D estimatedPose;

  pf_.executeOn(pdf_,& m_ActionCollection,& m_SensoryFrame, NULL); //&pf_stats_);
  
  pdf_.getMean(estimatedPose);
	
  return estimatedPose;
}

void MCLocalization_MRPT::OGMap2COccupancyGridMap(OGMap ogmap, COccupancyGridMap2D *gridmap) {
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

void MCLocalization_MRPT::TimedPose2D2CPose2D(const TimedPose2D & tp, CPose2D & cp, const RTC::OGMap & map){
		cp.x(map.map.column + tp.data.position.x / map.config.yScale);
		cp.y(map.map.row + tp.data.position.y / map.config.yScale);
		cp.phi(tp.data.heading);
	}