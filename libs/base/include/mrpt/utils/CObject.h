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
#include <mrpt/otherlibs/any/any.hpp>

namespace mrpt
{
	namespace utils
	{
		/** @name RTTI classes and functions
		    @{ */

		/** Register a class into the MRPT internal list of "CSerializable" descendents.
		  * \sa getAllRegisteredClasses
		  */
		void BASE_IMPEXP registerClass(const std::type_info &pNewClass);

		/** Mostly for internal use within mrpt sources, to handle exceptional cases with multiple serialization names for backward compatibility (CMultiMetricMaps, CImage,...)
		  */
		void  BASE_IMPEXP registerClassCustomName(const char*customName, const std::type_info* pNewClass);

		/** Returns a list with all the classes registered in the system through mrpt::utils::registerClass.
		  * \sa registerClass, findRegisteredClass
		  */
		std::vector<const std::type_info*> BASE_IMPEXP getAllRegisteredClasses();

		/** Like getAllRegisteredClasses(), but filters the list to only include children clases of a given base one.
		  * \sa getAllRegisteredClasses(), getAllRegisteredClassesChildrenOf()  */
		std::vector<const std::type_info*> BASE_IMPEXP getAllRegisteredClassesChildrenOf(const std::type_info* parent_id);

		/** Return info about a given class by its name, or NULL if the class is not registered
		  * \sa registerClass, getAllRegisteredClasses
		  */
		const std::type_info BASE_IMPEXP * findRegisteredClass(const std::string &className);

		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is of the given class. */
		#define IS_CLASS( ptrObj, class_name )  (typeid(*ptrObj)==typeid(class_name))

		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is an instance of the given class or any of its derived classes. */
		#define IS_DERIVED( ptrObj, class_name )  (dynamic_cast<class_name *>(&*ptrObj) != nullptr)

		/** The virtual base class of all MRPT classes with a unified RTTI system.
		 *   For each class named <code>CMyClass</code>, a new type named <code>CMyClassPtr</code> will be created as a smart pointer suitable for
		 *    keeping referencing count smart pointers to objects of that class. By default the base class of all these smart pointers is CObject::Ptr.
		 * \sa  mrpt::utils::CSerializable \ingroup mrpt_base_grp
		 */

		class CObject : public mrpt::any
		{
		public:
			using mrpt::any::any;
			using Ptr = std::shared_ptr<CObject>;
			using ConstPtr = std::shared_ptr<const CObject>;
		};

		void BASE_IMPEXP registerAllPendingClasses();

	} // End of namespace
} // End of namespace

#endif
