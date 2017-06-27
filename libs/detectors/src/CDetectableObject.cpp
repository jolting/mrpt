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

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CDetectableObject, CSerializable, mrpt::detectors)
template <> const char * mrpt::utils::CSerializer<CDetectable2D>::getClassName() { return "CDetectable2D";}
template <> const char * mrpt::utils::CSerializer<CDetectable3D>::getClassName() { return "CDetectable3D";}


void CDetectable2D::readFromStream(mrpt::utils::CStream &, int )
{
}

void CDetectable2D::writeToStream(mrpt::utils::CStream &, int *) const
{
}

void CDetectable3D::readFromStream(mrpt::utils::CStream &, int )
{
}

void CDetectable3D::writeToStream(mrpt::utils::CStream &, int *) const
{
}

CDetectable3D::CDetectable3D( const CDetectable2D::Ptr &object2d )
	: CDetectable2D( object2d.get() ), m_z(0)
{ 
}
