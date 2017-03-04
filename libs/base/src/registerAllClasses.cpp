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
	registerClass(  CMatrix ) ;
	registerClass(  CMatrixD ) ;
	registerClass(  CMatrixB ) ;
	registerClass(  CPolygon ) ;

//   Hack to enable compatibility with an older name of this class:
	registerClass(  CImage ) ;
	registerClassCustomName( "CMRPTImage",  CImage ) ;

	registerClass(  CSimpleDatabase ) ;
	registerClass(  CSimpleDatabaseTable ) ;
	registerClass(  CPropertiesValuesList ) ;
	registerClass(  CMHPropertiesValuesList ) ;
	registerClass(  CTypeSelector ) ;
	registerClass(  CMemoryChunk ) ;

	registerClass(  CPoint2D ) ;
	registerClass(  CPoint3D ) ;
	registerClass(  CPose2D ) ;
	registerClass(  CPose3D ) ;
	registerClass(  CPose3DQuat ) ;
	registerClass(  CPoses2DSequence ) ;
	registerClass(  CPoses3DSequence ) ;


	registerClass(  CPosePDF ) ;
	registerClass(  CPosePDFGaussian ) ;
	registerClass(  CPosePDFGaussianInf ) ;
	registerClass(  CPosePDFParticles ) ;
	registerClass(  CPosePDFGrid ) ;
	registerClass(  CPosePDFSOG ) ;

	registerClass(  CPointPDF ) ;
	registerClass(  CPointPDFGaussian ) ;
	registerClass(  CPointPDFParticles ) ;
	registerClass(  CPointPDFSOG ) ;

	registerClass(  CPosePDF ) ;
	registerClass(  CPose3DPDF ) ;
	registerClass(  CPose3DQuatPDF ) ;
	registerClass(  CPose3DPDFGaussian ) ;
	registerClass(  CPose3DPDFGaussianInf ) ;
	registerClass(  CPose3DPDFParticles ) ;
	registerClass(  CPose3DPDFSOG ) ;

	registerClass(  CPose3DQuatPDF ) ;
	registerClass(  CPose3DQuatPDFGaussian ) ;
	registerClass(  CPose3DQuatPDFGaussianInf ) ;

	registerClass(  CPose3DInterpolator ) ;

	registerClass(  TCamera ) ;
	registerClass(  TStereoCamera ) ;
	registerClass(  CSplineInterpolator1D  ) ;
	registerClass(  CStringList ) ;
#endif
}

