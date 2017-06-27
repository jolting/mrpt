/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CObservationReflectivity>::getClassName() { return "CObservationReflectivity";}


/** Default constructor.
 */
CObservationReflectivity::CObservationReflectivity( ) :
	reflectivityLevel ( 0.5f ),
	sensorPose(),
	sensorStdNoise( 0.2f )
{
}

CObservationReflectivity::~CObservationReflectivity()
{
}

namespace mrpt {
namespace utils {
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationReflectivity>::writeToStream(const CObservationReflectivity & o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 0;
	else
	{
		out << o.reflectivityLevel << o.sensorPose;
		out << o.sensorLabel
			<< o.timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationReflectivity>::readFromStream(CObservationReflectivity& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> o.reflectivityLevel >> o.sensorPose;
			in >> o.sensorLabel
			   >> o.timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
}
}

void CObservationReflectivity::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

}

