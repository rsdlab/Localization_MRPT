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
	pfOptions_.adaptiveSampleSize = adaptiveSampleSize;
	pfOptions_.pfAuxFilterOptimal_MaximumSearchSamples = pfAuxFilterOptimal_MaximumSearchSamples;
	pfOptions_.BETA = BETA;
	pfOptions_.sampleSize = sampleSize;
    pfOptions_.dumpToConsole();
 
    pf_.m_options = pfOptions_;
	
	///
	pdf_.clear();
	pdf_.options.KLD_params.KLD_binSize_PHI=KLD_binSize_PHI;
	pdf_.options.KLD_params.KLD_binSize_XY=KLD_binSize_XY;
	pdf_.options.KLD_params.KLD_delta = KLD_delta;
	pdf_.options.KLD_params.KLD_epsilon =KLD_epsilon;
	pdf_.options.KLD_params.KLD_maxSampleSize =KLD_maxSampleSize;
	pdf_.options.KLD_params.KLD_minSampleSize = KLD_minSampleSize;
	pdf_.options.KLD_params.KLD_minSamplesPerBin = KLD_minSamplesPerBin;

    pdf_.options.metricMap = &m_map;
	
	///
	motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
	motion_model_options_.gausianModel.minStdXY  = minStdXY;
	motion_model_options_.gausianModel.minStdPHI = minStdPHI;
	/*
	motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
    motion_model_default_options_.gausianModel.minStdXY  = 0.10;
    motion_model_default_options_.gausianModel.minStdPHI = 2.0;
	*/
	pdf_.resetUniformFreeSpace(&m_map, 0.7f, 1000, min_x, max_x, min_y, max_y,min_phi, max_phi);
	
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
		if(observation->scan[i] > range_min && observation->scan[i] < range_max) {
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
			}else if(cell > 200){
				gridmap->setCell(j, i, 1.0);
			}else{
				gridmap->setCell(j, i, 0.5);
			}
		}
	}
	/*
	CImage		img;
	gridmap->getAsImage(img,false, true);  
	mrpt::gui::CDisplayWindow	win("Computed path");
	win.showImage(img.scaleDouble().scaleDouble());
	win.waitForKey();
	//*/
}

void MCLocalization_MRPT::TimedPose2D2CPose2D(const TimedPose2D & tp, CPose2D & cp, const RTC::OGMap & map){
		cp.x(map.map.column + tp.data.position.x / map.config.yScale);
		cp.y(map.map.row + tp.data.position.y / map.config.yScale);
		cp.phi(tp.data.heading);
	}