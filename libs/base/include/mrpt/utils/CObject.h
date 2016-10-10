/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_COBJECT_H
#define  MRPT_COBJECT_H

#include <mrpt/system/memory.h>
#include <mrpt/utils/safe_pointers.h>
#include <vector>

#include <memory>

namespace mrpt
{
	namespace utils
	{

		class BASE_IMPEXP CObject
		{};
		typedef std::shared_ptr<CObject> CObjectPtr;

	} // End of namespace
} // End of namespace


#endif
