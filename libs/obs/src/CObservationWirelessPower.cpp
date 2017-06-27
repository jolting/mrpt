/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CObservationWirelessPower>::getClassName() { return "CObservationWirelessPower";}

/** Constructor
 */
CObservationWirelessPower::CObservationWirelessPower( ) :
	power(0)
{
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationWirelessPower>::writeToStream(const CObservationWirelessPower &o, mrpt::utils::CStream &out, int *version)
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 3;
	else
	{
		// The data
		out << o.power
			<< o.sensorLabel
			<< o.timestamp
			<< o.sensorPoseOnRobot; // Added in v3
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationWirelessPower>::readFromStream(CObservationWirelessPower& o, mrpt::utils::CStream &in, int version)
{
	//MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			in	>> o.power;
			if (version>=1)
				in >> o.sensorLabel;
			else o.sensorLabel="";

			if (version>=2)
					in >> o.timestamp;
			else 	o.timestamp = INVALID_TIMESTAMP;

			if (version>=3)
					in >> o.sensorPoseOnRobot;
			else 	o.sensorPoseOnRobot = CPose3D();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
}
}


void CObservationWirelessPower::getSensorPose( CPose3D &out_sensorPose ) const
{
	out_sensorPose=sensorPoseOnRobot;
}

void CObservationWirelessPower::setSensorPose( const CPose3D &newSensorPose )
{
	sensorPoseOnRobot = newSensorPose;
}

void CObservationWirelessPower::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	std::cout << format("Measured Power: %.02f/100\n", power);
}


