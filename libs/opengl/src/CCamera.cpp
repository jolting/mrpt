/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CCamera.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CCamera, CRenderizable, mrpt::opengl )


/*--------------------------------------------------------------
					CCamera
  ---------------------------------------------------------------*/
CCamera::CCamera()	:
	m_pointingX(0),m_pointingY(0),m_pointingZ(0),
	m_distanceZoom(10),
	m_azimuthDeg(45),m_elevationDeg(45),
	m_projectiveModel(true),
	m_projectiveFOVdeg(30),
	m_6DOFMode(false)
{
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
template <> void CSerializer<CCamera>::writeToStream(const CCamera& o, mrpt::utils::CStream &out,int *version)
{
	if (version)
		*version = 1;
	else
	{
		// Save data:
		out << o.m_pointingX << o.m_pointingY << o.m_pointingZ
			<< o.m_distanceZoom
			<< o.m_azimuthDeg << o.m_elevationDeg
			<< o.m_projectiveModel << o.m_projectiveFOVdeg;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
template <>
void CSerializer<CCamera>::readFromStream(CCamera &o, mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 1:
		{
			// Load data:
			in  >> o.m_pointingX >> o.m_pointingY >> o.m_pointingZ
				>> o.m_distanceZoom
				>> o.m_azimuthDeg >> o.m_elevationDeg
                >> o.m_projectiveModel >> o.m_projectiveFOVdeg;
		}
		break;
	case 0:
		{
			in  >> o.m_pointingX >> o.m_pointingY >> o.m_pointingZ
				>> o.m_distanceZoom
				>> o.m_azimuthDeg >> o.m_elevationDeg;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

}
}

/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
void CCamera::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = mrpt::math::TPoint3D(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());
}
