/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CStream.h>
#include <limits>

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
template <>
void  CSerializer<CPoint2D>::writeToStream(const CPoint2D &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 1;
	else
	{
		// The coordinates:
		out << o.m_coords[0] << o.m_coords[1];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
template <>
void  CSerializer<CPoint2D>::readFromStream(CPoint2D &o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			// The coordinates:
			float f;
			in >> f; o.m_coords[0]=f;
			in >> f; o.m_coords[1]=f;
		} break;
	case 1:
		{
			// The coordinates:
			in >> o.m_coords[0] >> o.m_coords[1];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
}
}

/*---------------------------------------------------------------
The operator D="this"-b is the pose inverse compounding operator.
   The resulting pose "D" is the diference between this pose and "b"
 ---------------------------------------------------------------*/
CPoint2D  CPoint2D::operator - (const CPose2D& b) const
{
	const double  ccos = cos(b.phi());
	const double  ssin = sin(b.phi());
	const double  Ax = x()-b.x();
	const double  Ay = y()-b.y();

	return CPoint2D( Ax * ccos + Ay * ssin, -Ax * ssin + Ay * ccos );
}

void CPoint2D::setToNaN()
{
	for (int i=0;i<2;i++)
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();
}

