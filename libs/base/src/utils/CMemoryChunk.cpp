/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CMemoryChunk.h>

using namespace mrpt::utils;

namespace mrpt { namespace utils {
/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
template <>
void CSerializer<CMemoryChunk>::writeToStream(const CMemoryChunk &o, mrpt::utils::CStream &out, int *out_Version)
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		out << static_cast<uint64_t>(o.m_bytesWritten);
		if (o.m_bytesWritten)
		{
			ASSERT_(o.m_memory.get())
			out.WriteBuffer(o.m_memory.get(),o.m_bytesWritten);
		}
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
template <> void CSerializer<CMemoryChunk>::readFromStream(CMemoryChunk& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint64_t   N;
			in >> N;
			o.resize(N);

			o.m_bytesWritten = N;
			o.m_position     = 0;
			if (N)
				in.ReadBuffer( o.m_memory.get(), N );

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

}}
