/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_COBJECT_H
#define  MRPT_COBJECT_H

#include <mrpt/system/memory.h>
#include <mrpt/utils/safe_pointers.h>
#include <vector>
#include <memory>
#include <unordered_map>
#include <typeindex>
#include <mrpt/otherlibs/any/any.hpp>

namespace mrpt
{
	namespace utils
	{
		/** @name RTTI classes and functions
		    @{ */


		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is of the given class. */
		#define IS_CLASS( ptrObj, class_name )  (typeid(*ptrObj)==typeid(class_name))

		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is an instance of the given class or any of its derived classes. */
		#define IS_DERIVED( ptrObj, class_name )  (dynamic_cast<class_name *>(&*ptrObj) != nullptr)


		/** The virtual base class of all MRPT classes with a unified RTTI system.
		 *   For each class named <code>CMyClass</code>, a new type named <code>CMyClassPtr</code> will be created as a smart pointer suitable for
		 *    keeping referencing count smart pointers to objects of that class. By default the base class of all these smart pointers is CObject::Ptr.
		 * \sa  mrpt::utils::CSerializable \ingroup mrpt_base_grp
		 */
		void registerAllPendingClasses();

		class BASE_IMPEXP CObject : public mrpt::any
		{
		public:
			using mrpt::any::any;
			using Ptr = std::shared_ptr<CObject>;
			using ConstPtr = std::shared_ptr<const CObject>;
		};


	} // End of namespace
} // End of namespace

#endif
