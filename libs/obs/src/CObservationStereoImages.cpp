/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CStream.h>
#if MRPT_HAS_MATLAB
#	include <mexplus/mxarray.h>
#endif

using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CObservationStereoImages>::getClassName() { return "CObservationStereoImages";}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CObservationStereoImages::CObservationStereoImages( void *iplImageLeft,void *iplImageRight, void *iplImageDisparity,bool ownMemory ) :
	imageLeft( UNINITIALIZED_IMAGE ),
	imageRight( UNINITIALIZED_IMAGE ),
	imageDisparity( UNINITIALIZED_IMAGE ),
	hasImageDisparity( iplImageDisparity!=nullptr ),
	hasImageRight( iplImageRight!=nullptr )
{
	if (iplImageLeft)
		ownMemory ? imageLeft.setFromIplImage(iplImageLeft) : imageLeft.loadFromIplImage(iplImageLeft);
	if (iplImageRight)
		ownMemory ? imageRight.setFromIplImage(iplImageRight) : imageRight.loadFromIplImage(iplImageRight);
	if (iplImageDisparity)
		ownMemory ? imageDisparity.setFromIplImage(iplImageDisparity) : imageDisparity.loadFromIplImage(iplImageDisparity);
}

/*---------------------------------------------------------------
					Default Constructor
 ---------------------------------------------------------------*/
CObservationStereoImages::CObservationStereoImages( ) :
	hasImageDisparity(false),
	hasImageRight(true)
{
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CObservationStereoImages::~CObservationStereoImages(  )
{
}

namespace mrpt{
namespace utils {
/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationStereoImages>::writeToStream(const CObservationStereoImages &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 6 ;
	else
	{
		// The data
		out << o.cameraPose << o.leftCamera << o.rightCamera
			<< o.imageLeft;

		out << o.hasImageDisparity << o.hasImageRight;

		if (o.hasImageRight)
			out << o.imageRight;

		if (o.hasImageDisparity)
			out << o.imageDisparity;

		out << o.timestamp;
		out << o.rightCameraPose;
		out << o.sensorLabel;

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationStereoImages>::readFromStream(CObservationStereoImages& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 6:
		{
			in >> o.cameraPose >> o.leftCamera >> o.rightCamera
				>> o.imageLeft;

			in >> o.hasImageDisparity >> o.hasImageRight;

			if (o.hasImageRight)
				in >> o.imageRight;

			if (o.hasImageDisparity)
				in >> o.imageDisparity;

			in >> o.timestamp;
			in >> o.rightCameraPose;
			in >> o.sensorLabel;
		}
		break;

	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		{
			// This, for backwards compatibility before version 6:
			o.hasImageRight = true;
			o.hasImageDisparity = false;


			if( version < 5 )
			{
				CPose3D aux;
				in >> aux;
				o.cameraPose = CPose3DQuat( aux );
			}

			if( version >= 5 )
			{
				in >> o.cameraPose >> o.leftCamera >> o.rightCamera;
			}
			else
			{
				CMatrix intParams;
				in >> intParams;																// Get the intrinsic params
				o.leftCamera.intrinsicParams = CMatrixDouble33(intParams);				// Set them to both cameras
				o.rightCamera.intrinsicParams = CMatrixDouble33(intParams);				// ... distortion parameters are set to zero
			}

			in >> o.imageLeft >> o.imageRight;														// For all the versions

			if(version >= 1) in >> o.timestamp; else o.timestamp = INVALID_TIMESTAMP;				// For version 1 to 5
			if(version >= 2)
			{
				if(version < 5)
				{
					CPose3D aux;
					in >> aux;
					o.rightCameraPose = CPose3DQuat( aux );
			}
			else
					in >> o.rightCameraPose;
			}
			else
				o.rightCameraPose = CPose3DQuat( 0.10f, 0, 0, mrpt::math::CQuaternionDouble(1,0,0,0)  );	// For version 1 to 5

			if(version >= 3 && version < 5)														// For versions 3 & 4
			{
				double foc;
				in >> foc;																		// Get the focal length in meters
				o.leftCamera.focalLengthMeters = o.rightCamera.focalLengthMeters = foc;				// ... and set it to both cameras
			}
			else
				if( version < 3 )
					o.leftCamera.focalLengthMeters = o.rightCamera.focalLengthMeters = 0.002;		// For version 0, 1 & 2 (from version 5, this parameter is included in the TCamera objects)

			if(version >= 4) in >> o.sensorLabel; else o.sensorLabel = "";							// For version 1 to 5

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
}
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::obs::CObservationStereoImages )

mxArray* CObservationStereoImages::writeToMatlab() const
{
	const char* fields[] = {"class",
							"ts","sensorLabel",
							"imageL","imageR",
							"poseL","poseLR","poseR",
							"paramsL","paramsR"};
	mexplus::MxArray obs_struct( mexplus::MxArray::Struct(sizeof(fields)/sizeof(fields[0]),fields) );

	obs_struct.set("class", this->GetRuntimeClass()->className);
	obs_struct.set("ts", this->timestamp);
	obs_struct.set("sensorLabel", this->sensorLabel);
	obs_struct.set("imageL", this->imageLeft);
	obs_struct.set("imageR", this->imageRight);
	obs_struct.set("poseL", this->cameraPose);
	obs_struct.set("poseR", this->cameraPose + this->rightCameraPose);
	obs_struct.set("poseLR", this->rightCameraPose);
	obs_struct.set("paramsL", this->leftCamera);
	obs_struct.set("paramsR", this->rightCamera);
	return obs_struct.release();
}
#endif

/** Populates a TStereoCamera structure with the parameters in \a leftCamera, \a rightCamera and \a rightCameraPose */
void CObservationStereoImages::getStereoCameraParams(mrpt::utils::TStereoCamera &out_params) const
{
	out_params.leftCamera  = this->leftCamera;
	out_params.rightCamera = this->rightCamera;
	out_params.rightCameraPose = this->rightCameraPose;
}

/** Sets \a leftCamera, \a rightCamera and \a rightCameraPose from a TStereoCamera structure */
void CObservationStereoImages::setStereoCameraParams(const mrpt::utils::TStereoCamera &in_params)
{
	this->leftCamera      = in_params.leftCamera;
	this->rightCamera     = in_params.rightCamera;
	this->rightCameraPose = in_params.rightCameraPose;
}

/** This method only checks whether ALL the distortion parameters in \a leftCamera are set to zero, which is
  * the convention in MRPT to denote that this pair of stereo images has been rectified.
  */
bool CObservationStereoImages::areImagesRectified() const
{
	return (leftCamera.dist.array()==0).all();
}

// Do an efficient swap of all data members of this object with "o".
void CObservationStereoImages::swap( CObservationStereoImages &o)
{
	CObservation::swap(o);

	imageLeft.swap(o.imageLeft);
	imageRight.swap(o.imageRight);
	imageDisparity.swap(o.imageDisparity);

	std::swap(hasImageDisparity, o.hasImageDisparity);
	std::swap(hasImageRight, o.hasImageRight);

	std::swap(leftCamera,o.leftCamera);
	std::swap(rightCamera, o.rightCamera);

	std::swap(cameraPose, o.cameraPose);
	std::swap(rightCameraPose, o.rightCameraPose);
}

void CObservationStereoImages::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << cameraPose.getHomogeneousMatrixVal() << endl
		<< "Camera pose: " << cameraPose << endl
		<< "Camera pose (YPR): " << CPose3D(cameraPose) << endl
		<< endl;

	mrpt::utils::TStereoCamera stParams;
	getStereoCameraParams(stParams);
	o << stParams.dumpAsText() << endl;

	o << "Right camera pose wrt left camera (YPR):" << endl << CPose3D(stParams.rightCameraPose) << endl;

	if (imageLeft.isExternallyStored())
		o << " Left image is stored externally in file: " << imageLeft.getExternalStorageFile() << endl;

	o << " Right image";
	if (hasImageRight )
	{
		if (imageRight.isExternallyStored())
			o << " is stored externally in file: " << imageRight.getExternalStorageFile() << endl;
	}
	else o << " : No.\n";

	o << " Disparity image";
	if (hasImageDisparity )
	{
		if (imageDisparity.isExternallyStored())
			o << " is stored externally in file: " << imageDisparity.getExternalStorageFile() << endl;
	}
	else o << " : No.\n";

	o << format(" Image size: %ux%u pixels\n", (unsigned int)imageLeft.getWidth(), (unsigned int)imageLeft.getHeight() );

	o << " Channels order: " << imageLeft.getChannelsOrder() << endl;

	o << format(" Rows are stored in top-bottom order: %s\n", imageLeft.isOriginTopLeft() ? "YES" : "NO");
}


