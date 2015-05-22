#include "MobileRobotStub.h"
//#include "mrpt_localization_core.h"

#include <mrpt/base.h>
#include <mrpt/gui.h>

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/poses/CPose2D.h>

using namespace RTC;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;
	
namespace ssr{

	/**
	*
	*/
	class Position3D {
	public:
		Position3D(double X, double Y, double Z, double Roll, double Pitch, double Yaw) {
			x = X; y = Y; z = Z;
			roll = Roll; pitch = Pitch; yaw = Yaw;
		}

	public:
		double x;
		double y;
		double z;

		double roll;
		double pitch;
		double yaw;
	};


	/**
	*
	*/
	class Pose2D {

	public:
		double x;
		double y;
		double th;

	public:

		Pose2D() {
			x = 0; y = 0; th = 0;
		}

		Pose2D(const double X, const double Y, const double Th) {
			this->x = X; this->y = Y; this->th = Th;
		}

		Pose2D(const Pose2D& pose) {
			this->x = pose.x; this->y = pose.y; this->th = pose.th;
		}

	};

	inline static ssr::Pose2D operator-(const ssr::Pose2D& pose1, const ssr::Pose2D& pose2) {
		double dx = pose1.x - pose2.x;
		double dy = pose1.y - pose2.y;
		double dth = pose1.th - pose2.th;
		if(dth > M_PI) {
			dth -= 2*M_PI;
		} else if(dth < -M_PI) {
			dth += 2*M_PI;
		}
		return ssr::Pose2D( dx*cos(pose2.th) + dy*sin(pose2.th), 
			-dx*sin(pose2.th) + dy*cos(pose2.th),
			dth);
	}

	/**
	*
	*/
	class Range {
	public:
		double* range;
		int size;
		float aperture;
	public:
		Range(const double *Range, int Size, const float Aperture) {
			this->aperture = Aperture;
			this->size = Size;
			this->range = new double[size];
			memcpy(this->range, Range, Size*sizeof(double));
		}
	};

	class IndexOutOfRangeException : public ::std::exception {
	public:
	};

	/**
	*
	*/
	class Map {
	private:
		double m_Resolution;
		int32_t m_XMax, m_XMin;
		int32_t m_YMax, m_YMin;

		uint8_t *m_pGrid;

	public:
		Map() : m_Resolution(0), m_XMax(0), m_XMin(0), m_YMax(0), m_YMin(0), m_pGrid(NULL) {

		}

		/*
		Map(double resolution, uint32_t width, uint32_t height) : m_Resolution(resolution), m_Width(width), m_Height(height) {
		if(width == 0 || height == 0) {
		m_pGrid = NULL;
		}
		m_pGrid = new uint8_t[width*height];
		}*/

		~Map() {delete m_pGrid;}

	public:
		double getResolution() const { return m_Resolution; }
		uint32_t getWidth() const { return m_XMax-m_XMin; }
		uint32_t getHeight() const { return m_YMax - m_YMin; }
		int32_t getOriginX() const { return getWidth() - m_XMax; }
		int32_t getOriginY() const { return getHeight() - m_YMax; }

		uint8_t getCell(const uint32_t x, const uint32_t y) {
			if (x >= getWidth() || y >= getHeight()) {
				throw IndexOutOfRangeException();
			}
			return m_pGrid[y*getWidth()+x];
		}

		void setResolution(const double resolution) {m_Resolution = resolution;}

		void setSize(const uint32_t w, const uint32_t h, const uint32_t origin_x, const uint32_t origin_y) { 
			if (getWidth() != w || getHeight() != h) {
				delete m_pGrid;
				m_pGrid = new uint8_t[w*h];
				m_XMax = w - origin_x;
				m_XMin = m_XMax - w;
				m_YMax = h - origin_y;
				m_YMin = m_YMax - h;
			}
		}

		void setCell(const uint32_t x, const uint32_t y, const uint8_t value) {
			if (x >= getWidth() || y >= getHeight()) {
				throw IndexOutOfRangeException();
			}
			m_pGrid[y*getWidth()+x] = value;
		}



		//virtual bool load(const std::string& inputFileName) = 0;

		//virtual bool save(const std::string& outputFileName) = 0;
	};

	/**
	*
	*/
	class NamedString : public std::map<std::string, std::string>{
	public:
		NamedString() {}
		~NamedString() {}

		/*
		NamedString(const NamedString& ns) {
			
		}*/

	public:
		void setFloat(const char* key, const float value) {
			std::ostringstream oss;
			oss << value;
			this->operator[](key) = oss.str(); 
		}

	public:
		std::string getString(const char* key, const char* defaultVal) {
			if (find(key) == this->end()) { return defaultVal; }
			return this->operator[](key);
		}

		int getInt(const char* key, const int defaultVal)  {
			if (this->find(key) == this->end()) { return defaultVal; }
			return atoi(this->operator[](key).c_str());
		}

		bool getBool(const char* key, const bool defaultVal) {
			if (this->find(key) == this->end()) { return defaultVal; }

			std::string val = this->operator[](key);
			std::transform(val.begin(), val.end(), val.begin(), ::tolower);
			if (val == "true") return true;
			else return false;
		}

		float getFloat(const char* key, const float defaultVal) {
			if (this->find(key) == this->end()) { return defaultVal; }

			std::string val = this->operator[](key);
			return (float)atof(val.c_str());
		}
	};

	/**
	*
	*/
	class MCLocalization_MRPT{
	public:
		COccupancyGridMap2D m_map;
		//CMultiMetricMap m_metricmap;
		CMonteCarloLocalization2D pdf_;
		CParticleFilter::TParticleFilterOptions pfOptions_;		
		CParticleFilter::TParticleFilterStats pf_stats_;
		CParticleFilter pf_;
		//PFStates state_;

		CActionRobotMovement2D::TMotionModelOptions motion_model_options_;
		CActionRobotMovement2D::TMotionModelOptions motion_model_default_options_;

 		mrpt::slam::CActionCollection m_ActionCollection;
		mrpt::slam::CSensoryFrame m_SensoryFrame;
		mrpt::poses::CPose3D m_RangeSensorPose;
		mrpt::poses::CPosePDFGaussian initialPose_;

		//CPose2D estimatedPose;
		float m_range_min;
		float m_range_max;
		float m_min_x;
		float m_max_x;
		float m_min_y;
		float m_max_y;
		float m_min_phi;
		float m_max_phi;
				
		float m_minStdXY;
		float m_minStdPHI;
		double m_KLD_binSize_PHI;
		double m_KLD_binSize_XY;
		double m_KLD_delta;
		double m_KLD_epsilon;
		int   m_KLD_maxSampleSize;
		int   m_KLD_minSampleSize;
		double m_KLD_minSamplesPerBin;
		bool m_adaptiveSampleSize;
		int m_pfAuxFilterOptimal_MaximumSearchSamples;
 		double m_BETA;
		int m_sampleSize;

		  /*!
		  // The Particle Filter algorithm:
		  //	0: pfStandardProposal	  ***
		  //	1: pfAuxiliaryPFStandard
		  //	2: pfOptimalProposal    
		  //	3: pfAuxiliaryPFOptimal	  ***
		  */
		  mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm m_PF_algorithm;
		  /*!
		  // The Particle Filter Resampling method:
		  //	0: prMultinomial
		  //	1: prResidual
		  //	2: prStratified
		  //	3: prSystematic
		  */
		  mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm m_resamplingMethod;
		  //std::string m_map_file;	
		  //std::string m_rawlog_file;
		  //std::string m_logOutput_dir;
		  //int m_3DSceneFrequency;
		  int m_particles_count;
		  /*
		  int m_init_PDF_mode;
		  int m_init_PDF_min_x;
		  int m_init_PDF_max_x;
		  int m_init_PDF_min_y;
		  int m_init_PDF_max_y;
		  int m_init_PDF_min_phi_deg;
		  int m_init_PDF_max_phi_deg;
		  bool m_SHOW_PROGRESS_3D_REAL_TIME;

		  int m_occupancyGrid_count;
		  int m_gasGrid_count;
		  int m_landmarksMap_count;
		  int m_pointsMap_count;
		  int m_beaconMap_count;
		  int m_likelihoodMapSelection;
		  int m_enableInsertion_pointsMap;
		  int m_enableInsertion_landmarksMap;
		  int m_enableInsertion_gridMaps;
		  int m_enableInsertion_gasGridMaps;
		  int m_enableInsertion_beaconMap;
		  double m_resolution;
		  int m_mapAltitude;
		  int m_useMapAltitude;
		  int m_maxDistanceInsertion;
		  double m_maxOccupancyUpdateCertainty;
		  int m_considerInvalidRangesAsFreeSpace;
		  double m_minLaserScanNoiseStd;
		  int m_likelihoodMethod;
		  int m_LF_decimation;
		  double m_LF_stdHit;
		  double m_LF_maxCorrsDistance;
		  double m_LF_zHit;
		  double m_LF_zRandom;
		  double m_LF_maxRange;
		  int m_LF_alternateAverageMethod;
		  int m_MI_exponent;
		  int m_MI_skip_rays;
		  int m_MI_ratio_max_distance;				
		  int m_rayTracing_useDistanceFilter;
		  int m_rayTracing_decimation;
		  double m_rayTracing_stdHit;
		  int m_consensus_takeEachRange;
		  int m_consensus_pow;
	*/

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		MCLocalization_MRPT();
		~MCLocalization_MRPT();

		void setMap(const OGMap& map);
		void initialize();
		bool setRangeSensorPosition(const ssr::Position3D& position) {
		m_RangeSensorPose = mrpt::poses::CPose3D(position.x, position.y, position.z, position.roll, position.pitch, position.yaw);
		return true;
		}
		bool addPose(const ssr::Pose2D& pose);
		bool addRange(const ssr::Range& range);
		
		void setRangeSensorRange(float min, float max) {
			m_range_min = min;
			m_range_max = max;
		}
		CPose2D getEstimatedPose();

	protected:
		void configureFilter(const mrpt::utils::CConfigFile &_configFile);
		void OGMap2COccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap);
		void TimedPose2D2CPose2D(const TimedPose2D & tp, CPose2D & cp, const RTC::OGMap & map);
	};

};


 

