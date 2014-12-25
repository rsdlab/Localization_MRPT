#include "MonteCarloLocalization.h"

		
using namespace ssr;

MCLocalization_MRPT::MCLocalization_MRPT(){
	//constractor 
}

MCLocalization_MRPT::~MCLocalization_MRPT(){
	//destractor 
}

void  MCLocalization_MRPT::setMap(const OGMap& map){
	OGMap2COccupancyGridMap(map, &theMap);
}
void  MCLocalization_MRPT::initialize(){

    pdf.options.metricMap = &theMap;

	//////////////////////////////
	//have to create MRPT config file
	//////////////////////////////

	//CParticleFilter::TParticleFilterOptions		pfOptions;
	//pfOptions.loadFromConfigFile( iniFile, "PF_options" );
	//pfOptions.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm.pfStandardProposal;
	//pfOptions.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm.prMultinomial;
	pfOptions.adaptiveSampleSize = 1;
	pfOptions.pfAuxFilterOptimal_MaximumSearchSamples = 10;
	pfOptions.BETA = 0.;
	pfOptions.sampleSize = 1;


	//TMonteCarloLocalizationParams	pdfPredictionOptions;
	//pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile, "KLD_options");
	pdf.options.KLD_params.KLD_binSize_PHI=10;
	pdf.options.KLD_params.KLD_binSize_XY=0.10;
	pdf.options.KLD_params.KLD_delta = 0.01;
	pdf.options.KLD_params.KLD_epsilon =0.01;
	pdf.options.KLD_params.KLD_maxSampleSize = 40000;
	pdf.options.KLD_params.KLD_minSampleSize = 150;
	pdf.options.KLD_params.KLD_minSamplesPerBin = 0;
		
	dummy_odom_params.modelSelection = CActionRobotMovement2D::mmGaussian;
	dummy_odom_params.gausianModel.minStdXY  = 0.10;
	dummy_odom_params.gausianModel.minStdPHI = 2.0;

	/////////
	/////////

	PF.m_options = pfOptions;

	//initialize PDF
	//
    double xmin = theMap.getXMin();
    double xmax = theMap.getXMax();
    double ymin = theMap.getYMin();
    double ymax = theMap.getYMax();
	pdf.resetUniformFreeSpace(&theMap,40000, -1, xmin,xmax,ymin,ymax, 0, 2*M_PI);

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
  //resampling
  pdf.performResampling(pfOptions,0);

  //prediction
  pdf.prediction_and_update(&m_ActionCollection, &m_SensoryFrame, pfOptions);
  
  //weihting
  /*
  CPose3D CurrentPose3D(CurrentPose.x,CurrentPose.y,CurrentPose.th);
  pdf.PF_SLAM_computeObservationLikelihoodForParticle(pfOptions, pdf.size(),  m_SensoryFrame, CurrentPose3D);
  */
  for(int i=0; i<pdf.size(); i++){
	  pdf.setW(i, theMap.computeObservationsLikelihood(m_SensoryFrame, pdf.getParticlePose(i)));
  }

  //measure
  pdf.getMean(estimatedPose);
	
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