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
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h> // for << ops
#include <mrpt/math/wrap2pi.h>
#include <set>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CObservationBearingRange>::getClassName() { return "CObservationBearingRange";}

/*---------------------------------------------------------------
 Default constructor.
 ---------------------------------------------------------------*/
CObservationBearingRange::CObservationBearingRange( ) :
	minSensorDistance(0),
	maxSensorDistance(0),
	fieldOfView_yaw(DEG2RAD(180)),
	fieldOfView_pitch(DEG2RAD(90)),
	sensorLocationOnRobot(),
	sensedData(),
	validCovariances(false),
	sensor_std_range(0),
	sensor_std_yaw(0),
	sensor_std_pitch(0)
{
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationBearingRange>::writeToStream(const CObservationBearingRange &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	i,n;

		// The data
		out << o.minSensorDistance
		    << o.maxSensorDistance
		    << o.fieldOfView_yaw
			<< o.fieldOfView_pitch
		    << o.sensorLocationOnRobot
		    << o.timestamp;

		out << o.validCovariances;
		if (!o.validCovariances)
			out << o.sensor_std_range << o.sensor_std_yaw << o.sensor_std_pitch;

		// Detect duplicate landmarks ID, which is an error!
		std::set<int32_t>  lstIDs;

		n = o.sensedData.size();
		out << n;
		for (i=0;i<n;i++)
		{
			int32_t  id = o.sensedData[i].landmarkID;
			if (id!=INVALID_LANDMARK_ID)
			{
				if (0!=lstIDs.count(id))
					THROW_EXCEPTION_FMT("Duplicate landmark ID=%i found.",(int)id);
				lstIDs.insert(id);
			}

			out << o.sensedData[i].range
			    << o.sensedData[i].yaw
			    << o.sensedData[i].pitch
			    << id;

			if (o.validCovariances)
				out << o.sensedData[i].covariance;
		}

		out << o.sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationBearingRange>::readFromStream(CObservationBearingRange& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t		i,n;

			// The data
			in >> o.minSensorDistance
			   >> o.maxSensorDistance;

			if (version>=3)
			{
				in >> o.fieldOfView_yaw
				   >> o.fieldOfView_pitch;
			}
			else
			{
				float fieldOfView;
				in >> fieldOfView;

				o.fieldOfView_yaw =
				o.fieldOfView_pitch = fieldOfView;
			}

			in >> o.sensorLocationOnRobot;

			if (version>=2)
					in >> o.timestamp;
			else 	o.timestamp = INVALID_TIMESTAMP;

			if (version>=3)
			{
				in >> o.validCovariances;
				if (!o.validCovariances)
					in >> o.sensor_std_range >> o.sensor_std_yaw >> o.sensor_std_pitch;
			} else
				o.validCovariances = false;

			in >> n;
			o.sensedData.resize(n);

			// Detect duplicate landmarks ID, what is an error!
			std::set<int32_t>  lstIDs;

			for (i=0;i<n;i++)
			{
				in >> o.sensedData[i].range
				   >> o.sensedData[i].yaw
				   >> o.sensedData[i].pitch
				   >> o.sensedData[i].landmarkID;

				if (version>=3 && o.validCovariances)
					in >> o.sensedData[i].covariance;

				int32_t  id = o.sensedData[i].landmarkID;
				if (id!=INVALID_LANDMARK_ID)
				{
					if (0!=lstIDs.count(id))
						THROW_EXCEPTION_FMT("Duplicate landmark ID=%i found.",(int)id);
					lstIDs.insert(id);
				}
			}

			if (version>=1)
					in >> o.sensorLabel;
			else 	o.sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
}
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBearingRange::debugPrintOut()
{
	printf("[CObservationBearingRange::debugPrintOut] Dumping:\n");
	printf("[CObservationBearingRange::debugPrintOut] minSensorDistance:\t%f\n",minSensorDistance);
	printf("[CObservationBearingRange::debugPrintOut] maxSensorDistance:\t%f:\n",maxSensorDistance);
	printf("[CObservationBearingRange::debugPrintOut] %u landmarks:\n",static_cast<unsigned>(sensedData.size()) );

	size_t		i, n = sensedData.size();
	for (i=0;i<n;i++)
		printf("[CObservationBearingRange::debugPrintOut] \tID[%i]: y:%fdeg p:%fdeg range: %f\n",
		sensedData[i].landmarkID,
		RAD2DEG( sensedData[i].yaw ),
		RAD2DEG( sensedData[i].pitch ),
		sensedData[i].range );
}

void CObservationBearingRange::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << sensorLocationOnRobot.getHomogeneousMatrixVal()
	<< sensorLocationOnRobot << endl << endl;

	o << "Do observations have individual covariance matrices? " << (validCovariances ? "YES":"NO") << endl << endl;

	o << "Default noise sigmas:" << endl;
	o << "sensor_std_range (m)   : " << sensor_std_range << endl;
	o << "sensor_std_yaw   (deg) : " << RAD2DEG(sensor_std_yaw) << endl;
	o << "sensor_std_pitch (deg) : " << RAD2DEG(sensor_std_pitch) << endl;

	o << endl;

	// For each entry in this sequence:
	o << "  LANDMARK_ID    RANGE (m)    YAW (deg)    PITCH (deg)   COV. MATRIX (optional)" << endl;
	o << "--------------------------------------------------------------------------------------" << endl;
	for (size_t q=0;q<sensedData.size();q++)
	{

		o << "      ";
		if (sensedData[q].landmarkID==INVALID_LANDMARK_ID)
			o << "(NO ID)";
		else o << format("%7u",sensedData[q].landmarkID);

		o << format("   %10.03f  %10.03f %10.03f        ",
			sensedData[q].range,
			RAD2DEG( mrpt::math::wrapToPi( sensedData[q].yaw)),
			RAD2DEG( mrpt::math::wrapToPi(sensedData[q].pitch)) );

		if (validCovariances)
			o << sensedData[q].covariance.inMatlabFormat() << endl;
		else
			o << "  (N/A)\n";
	}

}
