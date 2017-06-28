/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

using namespace mrpt::hmtslam;
using namespace mrpt::utils;


template <> const char * mrpt::utils::CSerializer<THypothesisIDSet>::getClassName() { return "THypothesisIDSet";}

namespace mrpt { namespace utils {
/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
template <> void CSerializer<THypothesisIDSet>::writeToStream(const THypothesisIDSet &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	N = (uint32_t) o.size();
		out << N;
		for (THypothesisIDSet::const_iterator it=o.begin();it!=o.end();++it)
			out << *it;
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
template <> void CSerializer<THypothesisIDSet>::readFromStream(THypothesisIDSet& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,N;
			in >> N;

			o.clear();
			for (i=0;i<N;i++)
			{
				THypothesisID tmp;
				in >> tmp;
				o.insert(tmp);
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
}}
