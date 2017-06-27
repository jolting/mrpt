/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationSkeleton.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationSkeleton, CObservation, mrpt::obs)

// Helpful macros for reading and writing joints to a stream
#define WRITE_JOINT(_J) out << _J.x << _J.y << _J.z << _J.conf;
#define READ_JOINT(_J) in >> _J.x >> _J.y >> _J.z >> _J.conf;

namespace mrpt{
namespace utils {
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationSkeleton>::writeToStream(const CObservationSkeleton &o, CStream &out, int *version)
{
	if (version)
		*version = 2;
	else
	{
		WRITE_JOINT(o.head)
		WRITE_JOINT(o.neck)
		WRITE_JOINT(o.torso)
		
		WRITE_JOINT(o.left_shoulder)
		WRITE_JOINT(o.left_elbow)
		WRITE_JOINT(o.left_hand)
		WRITE_JOINT(o.left_hip)
		WRITE_JOINT(o.left_knee)
		WRITE_JOINT(o.left_foot)

		WRITE_JOINT(o.right_shoulder)
		WRITE_JOINT(o.right_elbow)
		WRITE_JOINT(o.right_hand)
		WRITE_JOINT(o.right_hip)
		WRITE_JOINT(o.right_knee)
		WRITE_JOINT(o.right_foot)

		out << o.sensorLabel
		    << o.timestamp
		    << o.sensorPose;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationSkeleton>::readFromStream(CObservationSkeleton &o, CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			READ_JOINT(o.head)
			READ_JOINT(o.neck)
			READ_JOINT(o.torso)
		
			READ_JOINT(o.left_shoulder)
			READ_JOINT(o.left_elbow)
			READ_JOINT(o.left_hand)
			READ_JOINT(o.left_hip)
			READ_JOINT(o.left_knee)
			READ_JOINT(o.left_foot)

			READ_JOINT(o.right_shoulder)
			READ_JOINT(o.right_elbow)
			READ_JOINT(o.right_hand)
			READ_JOINT(o.right_hip)
			READ_JOINT(o.right_knee)
			READ_JOINT(o.right_foot)

			in >> o.sensorLabel;
			in >> o.timestamp;
			if (version >= 2){
				in >> o.sensorPose;
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
}
}

void CObservationSkeleton::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose on the robot: " << sensorPose << endl;

    // ----------------------------------------------------------------------
    //              CObservationSkeleton
    // ----------------------------------------------------------------------
    o << endl << "Joint Positions (x, y, z) [mm] -- confidence" << endl;

#define PRINT_JOINT(_J) cout << "\t" << #_J << ":\t(" << this->_J.x << ", " << this->_J.y << ", " << this->_J.z << ") -- " << this->_J.conf << endl;
														
		PRINT_JOINT(head)
		PRINT_JOINT(neck)
		PRINT_JOINT(torso)

		PRINT_JOINT(left_shoulder)
		PRINT_JOINT(left_elbow)
		PRINT_JOINT(left_hand)
		PRINT_JOINT(left_hip)
		PRINT_JOINT(left_knee)
		PRINT_JOINT(left_foot)

		PRINT_JOINT(right_shoulder)
		PRINT_JOINT(right_elbow)
		PRINT_JOINT(right_hand)
		PRINT_JOINT(right_hip)
		PRINT_JOINT(right_knee)
		PRINT_JOINT(right_foot)

}
