/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers
#include <mrpt/obs/CObservationStereoImagesFeatures.h>

#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationStereoImagesFeatures, CObservation,mrpt::obs)

 CObservationStereoImagesFeatures::CObservationStereoImagesFeatures( ) :
	cameraLeft(),
	cameraRight(),
	rightCameraPose(),
	cameraPoseOnRobot()
	{}

CObservationStereoImagesFeatures::CObservationStereoImagesFeatures(
	const CMatrixDouble33 &iPLeft, const CMatrixDouble33 &iPRight,
	const CArrayDouble<5> &dPLeft, const CArrayDouble<5> &dPRight,
	const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot )
{
	cameraLeft.intrinsicParams	= iPLeft;
	cameraLeft.dist				= dPLeft;

	cameraRight.intrinsicParams	= iPRight;
	cameraRight.dist			= dPRight;

	rightCameraPose				= rCPose;
	cameraPoseOnRobot			= cPORobot;
}

CObservationStereoImagesFeatures::CObservationStereoImagesFeatures(
	const TCamera &cLeft, const TCamera &cRight,
	const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot )
{
	cameraLeft	= cLeft;
	cameraRight = cRight;

	rightCameraPose			= rCPose;
	cameraPoseOnRobot		= cPORobot;
}


CObservationStereoImagesFeatures::~CObservationStereoImagesFeatures( )
{}

void  CObservationStereoImagesFeatures::saveFeaturesToTextFile( const std::string &filename )
{
	CFileOutputStream	file( filename );

	vector<TStereoImageFeatures>::iterator it;
	for( it = theFeatures.begin(); it != theFeatures.end(); ++it )
		file << format("%u %.2f %.2f %.2f %.2f\n", it->ID, it->pixels.first.x, it->pixels.first.y, it->pixels.second.x, it->pixels.second.y );

	file.close();
}

namespace mrpt {
namespace utils {
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void  CSerializer<CObservationStereoImagesFeatures>::writeToStream(const CObservationStereoImagesFeatures &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 0 ;
	else
	{
		// The data
		out << o.cameraLeft;
		out << o.cameraRight;
		out << o.rightCameraPose << o.cameraPoseOnRobot;
		out << (uint32_t)o.theFeatures.size();	// Write the number of items within the feature list
		for( unsigned int i = 0; i < o.theFeatures.size(); ++i )
		{
			out << o.theFeatures[i].pixels.first.x << o.theFeatures[i].pixels.first.y;
			out << o.theFeatures[i].pixels.second.x << o.theFeatures[i].pixels.second.y;
			out << (uint32_t)o.theFeatures[i].ID;
		}
		out << o.sensorLabel << o.timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationStereoImagesFeatures>::readFromStream(CObservationStereoImagesFeatures& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t nL, nR;
			in >> o.cameraLeft;
			in >> o.cameraRight;
			in >> o.rightCameraPose >> o.cameraPoseOnRobot;
			in >> nL;
			o.theFeatures.resize( nL );
			for( unsigned int i = 0; i < o.theFeatures.size(); ++i )
			{
				in >> o.theFeatures[i].pixels.first.x >> o.theFeatures[i].pixels.first.y;
				in >> o.theFeatures[i].pixels.second.x >> o.theFeatures[i].pixels.second.y;
				in >> nR;
				o.theFeatures[i].ID = (unsigned int)nR;
			}
			in >> o.sensorLabel >> o.timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
}
}

void CObservationStereoImagesFeatures::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << cameraPoseOnRobot.getHomogeneousMatrixVal()
	<< cameraPoseOnRobot << endl;

	o << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:\n";
	o << rightCameraPose.getHomogeneousMatrixVal()
	<< rightCameraPose << endl;

	o << "Intrinsic parameters matrix for the LEFT camera:"<< endl;
	CMatrixDouble33 aux = cameraLeft.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the LEFT camera:"<< endl << "[ ";
	for( unsigned int i = 0; i < 5; ++i )
		o << cameraLeft.dist[i] << " ";
	o << "]" << endl;

	o << "Intrinsic parameters matrix for the RIGHT camera:"<< endl;
	aux = cameraRight.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the RIGHT camera:"<< endl << "[ ";
	for( unsigned int i = 0; i < 5; ++i )
		o << cameraRight.dist[i] << " ";
	o << "]"<< endl;

	o << endl << format(" Image size: %ux%u pixels\n", (unsigned int)cameraLeft.ncols, (unsigned int)cameraLeft.nrows );
	o << endl << format(" Number of features in images: %u\n", (unsigned int)theFeatures.size() );


}


