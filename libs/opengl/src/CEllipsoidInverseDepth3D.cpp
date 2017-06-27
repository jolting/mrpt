/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CEllipsoidInverseDepth3D.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

template <> const char * mrpt::utils::CSerializer<CEllipsoidInverseDepth3D>::getClassName() { return "CEllipsoidInverseDepth3D";}
template <> const char * mrpt::utils::CSerializer<CGeneralizedEllipsoidTemplate<3>>::getClassName() { return "CGeneralizedEllipsoidTemplate<3>";}

/*---------------------------------------------------------------
							transformFromParameterSpace
  ---------------------------------------------------------------*/
  void CEllipsoidInverseDepth3D::transformFromParameterSpace(
	const std::vector<BASE::array_parameter_t> &in_pts,
	std::vector<BASE::array_point_t> & out_pts) const
{
	MRPT_START

	// (inv_range,yaw,pitch) --> (x,y,z)
	const size_t N = in_pts.size();
	out_pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		const double inv_range   = in_pts[i][0];
		const double yaw   = in_pts[i][1];
		const double pitch = in_pts[i][2];

		const double range = inv_range<0 ? m_underflowMaxRange : (inv_range!=0 ? 1./inv_range : 0);

		out_pts[i][0] = range * cos(yaw)*cos(pitch);
		out_pts[i][1] = range * sin(yaw)*cos(pitch);
		out_pts[i][2] =-range * sin(pitch);
	}

	MRPT_END
}

namespace mrpt
{
namespace utils
{
/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
template <> void CSerializer<CEllipsoidInverseDepth3D>::writeToStream(const CEllipsoidInverseDepth3D& o, mrpt::utils::CStream &out,int *version)
{
	if (version)
		*version = 0;
	else
	{
		o.writeToStreamRender(out);
		o.thisclass_writeToStream(out);

		out << o.m_underflowMaxRange;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
template <>
void CSerializer<CEllipsoidInverseDepth3D>::readFromStream(CEllipsoidInverseDepth3D &o, mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			o.readFromStreamRender(in);
			o.thisclass_readFromStream(in);

			in >> o.m_underflowMaxRange;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	o.notifyChange();
}
}
}
