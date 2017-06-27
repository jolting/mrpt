/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CPropertiesValuesList.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <stdio.h>
#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::system;

// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CPropertiesValuesList>::getClassName() { return "CPropertiesValuesList";}

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
namespace mrpt { namespace utils {
template <>
void CSerializer<CPropertiesValuesList>::writeToStream(const CPropertiesValuesList &o, mrpt::utils::CStream &out, int *out_Version)
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		uint32_t	i,n = (uint32_t)o.size();
		uint8_t		isNull;
		out << n;

		for (i=0;i<n;i++)
		{
			// Name:
			out << o.m_properties[i].name.c_str();

			// Object:
			isNull = o.m_properties[i].value ? 1:0;
			out << isNull;

			if (o.m_properties[i].value)
				out << *o.m_properties[i].value;
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
template <> void CSerializer<CPropertiesValuesList>::readFromStream(CPropertiesValuesList& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;
			uint8_t		isNull;

			// Erase previous contents:
			o.clear();

			in >> n;

			o.m_properties.resize(n);
			for (i=0;i<n;i++)
			{
				char	nameBuf[1024];
				// Name:
				in >> nameBuf;
				o.m_properties[i].name = nameBuf;

				// Object:
				in >> isNull;

				if (isNull)
					o.m_properties[i].value.reset(  static_cast<CSerializable*>(nullptr) );
				else
					in >> o.m_properties[i].value;
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
}}

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::CPropertiesValuesList()
{
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::~CPropertiesValuesList()
{
	clear();
}

/*---------------------------------------------------------------
					Copy Constructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::CPropertiesValuesList(const CPropertiesValuesList &o) :
	m_properties	( o.m_properties )
{
	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.reset(dynamic_cast<CSerializable*>(it->value->clone()));
}

/*---------------------------------------------------------------
					Copy
 ---------------------------------------------------------------*/
CPropertiesValuesList & CPropertiesValuesList::operator = (const CPropertiesValuesList &o)
{
	if (this!=&o) return *this;

	m_properties = o.m_properties;
	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.reset(dynamic_cast<CSerializable *>(it->value->clone()));
	return *this;
}


/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::clear()
{
	MRPT_START
	m_properties.clear();
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CSerializable::Ptr  CPropertiesValuesList::get(const std::string &propertyName)const
{
	for (std::vector<TPropertyValuePair>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if (!os::_strcmpi(propertyName.c_str(),it->name.c_str()))
			return it->value;
	}
	// Not found:
	return CSerializable::Ptr();
}


/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::set(const std::string &propertyName, const CSerializable::Ptr &obj)
{
	MRPT_START

	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if (!os::_strcmpi(propertyName.c_str(),it->name.c_str()))
		{
			// Delete current contents:
			// Copy new value:
			if (!obj)	it->value.reset();
			else		it->value = obj; //->clone();
			return;
		}
	}

	// Insert:
	TPropertyValuePair	newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP( \
		printf("Exception while setting annotation '%s'",propertyName.c_str()); \
		);
}

/*---------------------------------------------------------------
						size
 ---------------------------------------------------------------*/
size_t  CPropertiesValuesList::size()const
{
	return m_properties.size();
}

/*---------------------------------------------------------------
						getPropertyNames
 ---------------------------------------------------------------*/
std::vector<std::string>  CPropertiesValuesList::getPropertyNames()const
{
	std::vector<std::string>	ret;

	for (std::vector<TPropertyValuePair>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
		ret.push_back(it->name);

	return ret;
}
