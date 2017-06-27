/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationOdometry, CObservation, mrpt::obs)

/** Constructor
 */
	CObservationOdometry::CObservationOdometry() :
	odometry(),
	hasEncodersInfo(false),
	encoderLeftTicks(0), encoderRightTicks(0),
	hasVelocities(false),
	velocityLocal(.0, .0, .0)
{
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationOdometry>::writeToStream(const CObservationOdometry &o, mrpt::utils::CStream &out, int *version)
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 2;
	else
	{
		// The data
		out << o.odometry
			<< o.sensorLabel
			<< o.timestamp
			// Added in V1:
			<< o.hasEncodersInfo;
		if (o.hasEncodersInfo)
			out << o.encoderLeftTicks << o.encoderRightTicks;

		out << o.hasVelocities; 
		if (o.hasVelocities)
			out << o.velocityLocal;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationOdometry>::readFromStream(CObservationOdometry& o, mrpt::utils::CStream &in, int version)
{
	MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			in	>> o.odometry
			    >> o.sensorLabel
			    >> o.timestamp;

			if (version>=1)
			{
				in >> o.hasEncodersInfo;
				if (o.hasEncodersInfo || version < 2)
					in >> o.encoderLeftTicks >> o.encoderRightTicks;
				
				in >> o.hasVelocities;
				if (version < 2) {
					float vx, w;
					in >> vx >> w;
					o.velocityLocal.vx = vx;
					o.velocityLocal.vy = .0;
					o.velocityLocal.omega = w;
				}
				else
				{// v2
					if (o.hasVelocities)
						in >> o.velocityLocal;
				}
			}
			else
			{  
				o.hasEncodersInfo = false;
				o.encoderLeftTicks = o.encoderRightTicks = 0;
				o.hasVelocities = false;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
}
}

void CObservationOdometry::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << std::endl << "Odometry reading: " << odometry << std::endl;

	// Additional data:
	if (hasEncodersInfo)
	{
		o << format(" Encoder info: deltaL=%i deltaR=%i\n", encoderLeftTicks, encoderRightTicks );
	}
	else    o << "Encoder info: Not available!\n";

	if (hasVelocities)
	{
		o << format("Velocity info: %s\n", velocityLocal.asString().c_str());
	}
	else   o << "Velocity info: Not available!\n";

}

