#include "MonteCarloLocalization.h"

using namespace ssr;


MCLocalization_MRPT::MCLocalization_MRPT(){
	//constractor 
}

MCLocalization_MRPT::~MCLocalization_MRPT(){
	//destractor 
}

void  MCLocalization_MRPT::setMap(const OGMap& map){
	OGMap2COccupancyGridMap(map, &m_map);
}

void  MCLocalization_MRPT::initialize(){
	///
	//pfOptions_.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm.pfStandardProposal;
	//pfOptions_.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm.prMultinomial;
	pfOptions_.adaptiveSampleSize = 1;
	pfOptions_.pfAuxFilterOptimal_MaximumSearchSamples = 10;
	pfOptions_.BETA = 0.5;
	pfOptions_.sampleSize = 1;
    pfOptions_.dumpToConsole();
 
    pf_.m_options = pfOptions_;
	
	///
	pdf_.clear();
	pdf_.options.KLD_params.KLD_binSize_PHI=20;
	pdf_.options.KLD_params.KLD_binSize_XY=0.20;
	pdf_.options.KLD_params.KLD_delta = 0.02;
	pdf_.options.KLD_params.KLD_epsilon =0.02;
	pdf_.options.KLD_params.KLD_maxSampleSize =1000;
	pdf_.options.KLD_params.KLD_minSampleSize = 150;
	pdf_.options.KLD_params.KLD_minSamplesPerBin = 0;

    pdf_.options.metricMap = &m_map;
	
	///
	motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
	motion_model_options_.gausianModel.minStdXY  = 0.10;
	motion_model_options_.gausianModel.minStdPHI = 2.0;
	/*
	motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
    motion_model_default_options_.gausianModel.minStdXY  = 0.10;
    motion_model_default_options_.gausianModel.minStdPHI = 2.0;
	*/
	
	float min_x = -0.01;
    float max_x =  0.01;
    float min_y = -0.01;
    float max_y =  0.01;
    float min_phi = 0.05;
    float max_phi = -0.05;
	this->m_range_max = 10.0;
	this->m_range_min = 0.3;
	
    pdf_.resetUniformFreeSpace(&m_map, 0.7f, 1000, min_x, max_x, min_y, max_y);
	
}

bool MCLocalization_MRPT::addPose(const ssr::Pose2D& deltaPose)
{
	CActionRobotMovement2D action;
	CActionRobotMovement2D::TMotionModelOptions options;
	action.computeFromOdometry(CPose2D(deltaPose.x, deltaPose.y, deltaPose.th), motion_model_options_);
	action.timestamp = mrpt::system::getCurrentTime();
	static TTimeStamp oldTimestamp;
	if(action.timestamp == oldTimestamp) {
		action.timestamp = oldTimestamp +1;
	}
	oldTimestamp = action.timestamp;
	m_ActionCollection.clear();
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
	m_SensoryFrame.clear();
	m_SensoryFrame.insert(observation);
	return true;
}

CPose2D MCLocalization_MRPT::getEstimatedPose(){
    pf_.executeOn(pdf_,& m_ActionCollection,& m_SensoryFrame, &pf_stats_);
  
	return pdf_.getMeanVal();
}

void MCLocalization_MRPT::OGMap2COccupancyGridMap(OGMap ogmap, COccupancyGridMap2D *gridmap) {
	gridmap->setSize(0-ogmap.map.width*ogmap.config.xScale/2, ogmap.map.width*ogmap.config.xScale/2, 0-ogmap.map.width*ogmap.config.yScale/2, ogmap.map.height*ogmap.config.yScale/2, ogmap.config.xScale);
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