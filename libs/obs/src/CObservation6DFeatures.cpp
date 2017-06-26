/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservation6DFeatures.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation6DFeatures, CObservation,mrpt::obs)

/** Default constructor */
CObservation6DFeatures::CObservation6DFeatures( ) :
	minSensorDistance ( 0 ),
	maxSensorDistance ( 1e6f )
{
}

CObservation6DFeatures::TMeasurement::TMeasurement() :
	id(INVALID_LANDMARK_ID)
{
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void  CSerializer<CObservation6DFeatures>::writeToStream(const CObservation6DFeatures &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 0;
	else
	{
		out << o.minSensorDistance << o.maxSensorDistance << o.sensorPose;

		const uint32_t n = o.sensedFeatures.size();
		out << n;
		for (uint32_t i=0;i<n;i++) 
		{
			const CObservation6DFeatures::TMeasurement & m = o.sensedFeatures[i];
			out << m.pose << m.id << m.inf_matrix;
		}

		out << o.sensorLabel
			<< o.timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservation6DFeatures>::readFromStream(CObservation6DFeatures& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> o.minSensorDistance >> o.maxSensorDistance >> o.sensorPose;

			uint32_t n;
			in >> n;
			o.sensedFeatures.clear();
			o.sensedFeatures.resize(n);
			for (uint32_t i=0;i<n;i++) 
			{
				CObservation6DFeatures::TMeasurement & m = o.sensedFeatures[i];
				in >> m.pose >> m.id >> m.inf_matrix;
			}

			in >> o.sensorLabel
			   >> o.timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}
}
}

void CObservation6DFeatures::getSensorPose( CPose3D &out_sensorPose ) const 
{ 
	out_sensorPose = sensorPose; 
}

void CObservation6DFeatures::setSensorPose( const CPose3D &newSensorPose ) 
{ 
	sensorPose = newSensorPose; 
}

void CObservation6DFeatures::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose: " << sensorPose << endl;
	o << "Min range  : " << minSensorDistance << endl;
	o << "Max range  : " << maxSensorDistance << endl << endl;

	o << "Observation count : " << sensedFeatures.size() << endl << endl;

	for (size_t k=0;k<sensedFeatures.size();k++)
	{
		const CObservation6DFeatures::TMeasurement & m = sensedFeatures[k];
		o << "#" << k << ": ID=" << m.id << "; value=" << m.pose << "; inf=" <<m.inf_matrix.inMatlabFormat() << endl;
	}
}
