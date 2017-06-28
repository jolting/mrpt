/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::detectors;

template <> const char * mrpt::utils::CSerializer<CDetectableObject>::getClassName() { return "CDetectableObject";}
template <> const char * mrpt::utils::CSerializer<CDetectable2D>::getClassName() { return "CDetectable2D";}
template <> const char * mrpt::utils::CSerializer<CDetectable3D>::getClassName() { return "CDetectable3D";}

namespace mrpt { namespace utils {
template <> void CSerializer<CDetectable2D>::readFromStream(CDetectable2D &o, mrpt::utils::CStream &, int )
{
}

template <> void CSerializer<CDetectable2D>::writeToStream(const CDetectable2D &o, mrpt::utils::CStream &, int *)
{
}

template <> void CSerializer<CDetectable3D>::readFromStream(CDetectable3D &o, mrpt::utils::CStream &, int )
{
}

template <> void CSerializer<CDetectable3D>::writeToStream(const CDetectable3D &o, mrpt::utils::CStream &, int *)
{
}

}}

CDetectable3D::CDetectable3D( const CDetectable2D::Ptr &object2d )
	: CDetectable2D( object2d.get() ), m_z(0)
{ 
}
