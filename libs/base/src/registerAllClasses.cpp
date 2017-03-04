/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/base.h>
#include <mrpt/utils/initializer.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_base_H
#	include "base-precomp.h"
#endif

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

MRPT_INITIALIZER(registerAllClasses_mrpt_base)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	// Abstract classes are not registered since they can not be
	//   instanciated, nor loaded from streams.
	RuntimeClassRegistry::registerClass<  CMatrix >() ;
	RuntimeClassRegistry::registerClass<  CMatrixD >() ;
	RuntimeClassRegistry::registerClass<  CMatrixB >() ;
	RuntimeClassRegistry::registerClass<  CPolygon >() ;

//   Hack to enable compatibility with an older name of this class:
	RuntimeClassRegistry::registerClass<  CImage >() ;
	RuntimeClassRegistry::registerClassCustomName<CImage>("CMRPTImage") ;

	RuntimeClassRegistry::registerClass<  CSimpleDatabase >() ;
	RuntimeClassRegistry::registerClass<  CSimpleDatabaseTable >() ;
	RuntimeClassRegistry::registerClass<  CPropertiesValuesList >() ;
	RuntimeClassRegistry::registerClass<  CMHPropertiesValuesList >() ;
	RuntimeClassRegistry::registerClass<  CTypeSelector >() ;
	RuntimeClassRegistry::registerClass<  CMemoryChunk >() ;

	RuntimeClassRegistry::registerClass<  CPoint2D >() ;
	RuntimeClassRegistry::registerClass<  CPoint3D >() ;
	RuntimeClassRegistry::registerClass<  CPose2D >() ;
	RuntimeClassRegistry::registerClass<  CPose3D >() ;
	RuntimeClassRegistry::registerClass<  CPose3DQuat >() ;
	RuntimeClassRegistry::registerClass<  CPoses2DSequence >() ;
	RuntimeClassRegistry::registerClass<  CPoses3DSequence >() ;


	RuntimeClassRegistry::registerClass<  CPosePDF >() ;
	RuntimeClassRegistry::registerClass<  CPosePDFGaussian >() ;
	RuntimeClassRegistry::registerClass<  CPosePDFGaussianInf >() ;
	RuntimeClassRegistry::registerClass<  CPosePDFParticles >() ;
	RuntimeClassRegistry::registerClass<  CPosePDFGrid >() ;
	RuntimeClassRegistry::registerClass<  CPosePDFSOG >() ;

	RuntimeClassRegistry::registerClass<  CPointPDF >() ;
	RuntimeClassRegistry::registerClass<  CPointPDFGaussian >() ;
	RuntimeClassRegistry::registerClass<  CPointPDFParticles >() ;
	RuntimeClassRegistry::registerClass<  CPointPDFSOG >() ;

	RuntimeClassRegistry::registerClass<  CPosePDF >() ;
	RuntimeClassRegistry::registerClass<  CPose3DPDF >() ;
	RuntimeClassRegistry::registerClass<  CPose3DQuatPDF >() ;
	RuntimeClassRegistry::registerClass<  CPose3DPDFGaussian >() ;
	RuntimeClassRegistry::registerClass<  CPose3DPDFGaussianInf >() ;
	RuntimeClassRegistry::registerClass<  CPose3DPDFParticles >() ;
	RuntimeClassRegistry::registerClass<  CPose3DPDFSOG >() ;

	RuntimeClassRegistry::registerClass<  CPose3DQuatPDF >() ;
	RuntimeClassRegistry::registerClass<  CPose3DQuatPDFGaussian >() ;
	RuntimeClassRegistry::registerClass<  CPose3DQuatPDFGaussianInf >() ;

	RuntimeClassRegistry::registerClass<  CPose3DInterpolator >() ;

	RuntimeClassRegistry::registerClass<  TCamera >() ;
	RuntimeClassRegistry::registerClass<  TStereoCamera >() ;
	RuntimeClassRegistry::registerClass<  CSplineInterpolator1D  >() ;
	RuntimeClassRegistry::registerClass<  CStringList >() ;
#endif
}

