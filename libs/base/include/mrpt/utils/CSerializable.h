/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CSERIALIZABLE_H
#define  CSERIALIZABLE_H

#include <mrpt/utils/CObject.h>
#include <mrpt/utils/TTypeName.h>
#include <mrpt/utils/types_simple.h>

#include <memory>

#if MRPT_HAS_MATLAB
typedef struct mxArray_tag mxArray; //!< Forward declaration for mxArray (avoid #including as much as possible to speed up compiling)
#endif

namespace mrpt
{
	namespace utils {
		class CStream;
	}

	/** Classes for serialization, sockets, ini-file manipulation, streams, list of properties-values, timewatch, extensions to STL.
	  * \ingroup mrpt_base_grp
	  */
	namespace utils
	{
		/** The virtual base class which provides a unified interface for all persistent objects in MRPT.
		 *  Many important properties of this class are inherited from mrpt::utils::CObject. See that class for more details.
		 *	 Refer to the tutorial about <a href="http://www.mrpt.org/Serialization" >serialization</a> online.
		 * \sa CStream
		 * \ingroup mrpt_base_grp
		 */
               class BASE_IMPEXP CSerializable
               {
		public:
			using Ptr = std::shared_ptr<CSerializable>;
			using ConstPtr = std::shared_ptr<const CSerializable>;
                       // This must be added to any CObject derived class:

                       virtual ~CSerializable() { }

                        /** Introduces a pure virtual method responsible for writing to a CStream.
                         *  This can not be used directly be users, instead use "stream << object;"
                         *   for writing it to a stream.
                         * \param out The output binary stream where object must be dumped.
                         * \param getVersion If nullptr, the object must be dumped. If not, only the
                         *             version of the object dump must be returned in this pointer. This enables
                         *     the versioning of objects dumping and backward compatibility with previously
                         *     stored data.
                         *     \exception std::exception On any error, see CStream::WriteBuffer
                         * \sa CStream
                         */
                       virtual void  writeToStream(mrpt::utils::CStream &out, int *getVersion) const = 0;

                        /** Introduces a pure virtual method responsible for loading from a CStream
                         *  This can not be used directly be users, instead use "stream >> object;"
                         *   for reading it from a stream or "stream >> object_ptr;" if the class is
                         *   unknown apriori.
                         * \param in The input binary stream where the object data must read from.
                         * \param version The version of the object stored in the stream: use this version
                         *                number in your code to know how to read the incoming data.
                         *     \exception std::exception On any error, see CStream::ReadBuffer
                         * \sa CStream
                         */
                       virtual void  readFromStream(mrpt::utils::CStream &in, int version) = 0;

               public:

                       /** Introduces a pure virtual method responsible for writing to a `mxArray` Matlab object,
                         * typically a MATLAB `struct` whose contents are documented in each derived class.
                         * \return A new `mxArray` (caller is responsible of memory freeing) or nullptr is class does not support conversion to MATLAB.
                         */
#if MRPT_HAS_MATLAB
                       virtual mxArray* writeToMatlab() const { return nullptr; }
#endif
               }; // End of class def.
		

		struct BASE_IMPEXP TRuntimeClassId
		{
			std::string className;
			const std::type_info &type_info;
			std::function<CSerializable::Ptr()> createObject;
		};

		/** Return info about a given class by its name, or NULL if the class is not registered
		  * \sa registerClass, getAllRegisteredClasses
		  */
		class BASE_IMPEXP RuntimeClassRegistry
		{
		public:
			static const TRuntimeClassId BASE_IMPEXP &findRegisteredClass(const std::string &className)
			{
				return inst().m_classinfoByName.at(className);
			}

			static const TRuntimeClassId BASE_IMPEXP &findRegisteredClass(const std::type_info &classType)
			{
				return inst().m_classinfo.at(std::type_index(classType));
			}
			
			/** Returns a list with all the classes registered in the system through mrpt::utils::registerClass.
		 	 * \sa registerClass, findRegisteredClass
		  	*/
			std::vector<const TRuntimeClassId&> BASE_IMPEXP getAllRegisteredClasses();

			template<typename T>
			static void BASE_IMPEXP registerClass()
			{
				registerClassCustomName<T>(typeid(T).name());
			}

			template<typename T>
			static void  BASE_IMPEXP registerClassCustomName(const char* customName,
				typename std::enable_if<!std::is_abstract<T>::value>::type * = nullptr)
			{
				inst().m_classinfo.emplace(
					std::make_pair(std::type_index(typeid(T)),
						TRuntimeClassId{
							std::string(customName), //class name
							typeid(T),
							[](){return CSerializable::Ptr(new T());} //create function
						}					
					)
				);
				inst().m_classinfoByName.emplace(
					std::make_pair(std::string(customName),
						TRuntimeClassId{
							std::string(customName), //class name
							typeid(T),
							[](){return CSerializable::Ptr(new T());} //create function
						}					
					)
				);
			}

			template<typename T>
			static void  BASE_IMPEXP registerClassCustomName(const char* customName,
				typename std::enable_if<std::is_abstract<T>::value>::type* = nullptr)
			{
				inst().m_classinfo.emplace(
					std::make_pair(std::type_index(typeid(T)),
						TRuntimeClassId{
							std::string(customName), //class name
							typeid(T),
							[](){
								THROW_EXCEPTION("Can't Create abstract class");
								return CSerializable::Ptr();
							}
						}					
					)
				);
				inst().m_classinfoByName.emplace(
					std::make_pair(std::string(customName),
						TRuntimeClassId{
							std::string(customName), //class name
							typeid(T),
							[](){
								THROW_EXCEPTION("Can't Create abstract class");
								return CSerializable::Ptr();
							} //create function
						}					
					)
				);
			}
			
			static RuntimeClassRegistry &inst() {static RuntimeClassRegistry s_reg; return s_reg;}
		private:
			RuntimeClassRegistry();
			std::unordered_map<std::type_index, const TRuntimeClassId> m_classinfo;
			std::unordered_map<std::string, const TRuntimeClassId> m_classinfoByName;
		};

		/** \addtogroup noncstream_serialization Non-CStream serialization functions (in #include <mrpt/utils/CSerializable.h>)
		  * \ingroup mrpt_base_grp
		  * @{ */

		/** Used to pass MRPT objects into a CORBA-like object (strings). See doc about "Integration with BABEL".
		 * \param o The object to be serialized.
		 * \return The string containing the binay version of object.
		 * \sa StringToObject, <a href="http://www.mrpt.org/Integration_with_BABEL" >Integration with BABEL</a>
		 */
		std::string BASE_IMPEXP ObjectToString(const CSerializable *o);

		/** Used to pass CORBA-like objects (strings) into a MRPT object.
		 * \param str An string generated with ObjectToString
		 * \param obj A currently empty pointer, where a pointer to the newly created object will be stored.
		 * \exception None On any internal exception, this function returns nullptr.
		 * \sa ObjectToString, <a href="http://www.mrpt.org/Integration_with_BABEL" >Integration with BABEL</a>
		 */
		void BASE_IMPEXP StringToObject(const std::string &str, CSerializable::Ptr &obj);

		/** Converts (serializes) an MRPT object into an array of bytes.
		 * \param o The object to be serialized.
		 * \param out_vector The vector which at return will contain the data. Size will be set automatically.
		 * \sa OctetVectorToObject, ObjectToString
		 */
		void BASE_IMPEXP ObjectToOctetVector(const CSerializable *o, vector_byte & out_vector);

		/** Converts back (de-serializes) a sequence of binary data into a MRPT object, without prior information about the object's class.
		 * \param in_data The serialized input data representing the object.
		 * \param obj The newly created object will be stored in this smart pointer.
		 * \exception None On any internal exception, this function returns a nullptr pointer.
		 * \sa ObjectToOctetVector, StringToObject
		 */
		void BASE_IMPEXP OctetVectorToObject(const vector_byte & in_data, CSerializable::Ptr &obj);

		/** Converts (serializes) an MRPT object into an array of bytes within a std::string, without codifying to avoid nullptr characters.
		 *  This is therefore more efficient than ObjectToString
		 * \param o The object to be serialized.
		 * \param out_vector The string which at return will contain the data. Size will be set automatically.
		 * \sa RawStringToObject, ObjectToOctetVector
		 */
		void BASE_IMPEXP ObjectToRawString(const CSerializable *o, std::string & out_str);

		/** Converts back (de-serializes) a sequence of binary data within a std::string into a MRPT object, without prior information about the object's class.
		 * \param in_data The serialized input data representing the object.
		 * \param obj The newly created object will be stored in this smart pointer.
		 * \exception None On any internal exception, this function returns a nullptr pointer.
		 * \sa ObjectToRawString
		 */
		void BASE_IMPEXP RawStringToObject(const std::string & in_str, CSerializable::Ptr &obj);

		/** @} */
	} // End of namespace
} // End of namespace

#endif
