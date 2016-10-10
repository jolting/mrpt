/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CSERIALIZABLE_H
#define  CSERIALIZABLE_H

#include <mrpt/utils/CObject.h>
#include <mrpt/utils/TTypeName.h>
#include <mrpt/utils/types_simple.h>
#include <boost/variant.hpp>
#include <mrpt/math/CMatrix.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoses2DSequence.h>

#if MRPT_HAS_MATLAB
typedef struct mxArray_tag mxArray; //!< Forward declaration for mxArray (avoid #including as much as possible to speed up compiling)
#endif

namespace mrpt
{
namespace utils
{
  typedef boost::variant<
    mrpt::math::CMatrix,
    mrpt::poses::CPoint2DPDFGaussian,
    mrpt::poses::CPoint2D,
    mrpt::math::TPolygon2D,
    mrpt::math::TPolygon3D,
    mrpt::poses::CPoses2DSequence,
    mrpt::poses::CPose2D
    > CSerializableOption;
}
} // End of namespace

#endif
