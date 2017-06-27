/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationRawDAQ.h>
#include <mrpt/utils/CStream.h>
#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
template <> const char * mrpt::utils::CSerializer<CObservationRawDAQ>::getClassName() { return "CObservationRawDAQ";}

namespace mrpt{
namespace utils{

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationRawDAQ>::writeToStream(const CObservationRawDAQ &o, mrpt::utils::CStream &out, int *version)
{
	if (version)
		*version = 0;
	else
	{
		out << o.sensorLabel << o.timestamp << o.sample_rate
			<< o.AIN_8bits << o.AIN_16bits << o.AIN_32bits << o.AIN_float << o.AIN_double
			<< o.AIN_channel_count << o.AIN_interleaved
			<< o.AOUT_8bits << o.AOUT_16bits << o.AOUT_float << o.AOUT_double << o.DIN << o.DOUT << o.CNTRIN_32bits << o.CNTRIN_double;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
template <> void CSerializer<CObservationRawDAQ>::readFromStream(CObservationRawDAQ& o, mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in  >> o.sensorLabel >> o.timestamp >> o.sample_rate
				>> o.AIN_8bits >> o.AIN_16bits >> o.AIN_32bits >> o.AIN_float >> o.AIN_double
				>> o.AIN_channel_count >> o.AIN_interleaved
				>> o.AOUT_8bits >> o.AOUT_16bits >> o.AOUT_float >> o.AOUT_double >> o.DIN >> o.DOUT >> o.CNTRIN_32bits >> o.CNTRIN_double;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
}
}

void CObservationRawDAQ::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	cout << "Sample rate             : " << sample_rate << " Hz" << endl;
	cout << "Analog IN Channel count : " << AIN_channel_count << endl;
	cout << "Analog IN interleaved?  : " << (AIN_interleaved ? "yes":"no")<< endl;

#define RAWDAQ_SHOW_FIRSTS(_VEC) \
	cout << "Raw data in " #_VEC " ("<< _VEC.size() <<" entries): First values ["; \
	if (!_VEC.empty()) { \
		for (size_t i=1;i<=std::min(_VEC.size(),static_cast<size_t>(10));i++) \
		cout << _VEC[i-1] << " "; \
		cout << " ... "; \
	} \
	cout << "]\n";

	RAWDAQ_SHOW_FIRSTS(AIN_8bits)
	RAWDAQ_SHOW_FIRSTS(AIN_16bits)
	RAWDAQ_SHOW_FIRSTS(AIN_32bits)
	RAWDAQ_SHOW_FIRSTS(AIN_float)
	RAWDAQ_SHOW_FIRSTS(AIN_double)
	RAWDAQ_SHOW_FIRSTS(AOUT_8bits)
	RAWDAQ_SHOW_FIRSTS(AOUT_16bits)
	RAWDAQ_SHOW_FIRSTS(AOUT_float)
	RAWDAQ_SHOW_FIRSTS(AOUT_double)
	RAWDAQ_SHOW_FIRSTS(DIN)
	RAWDAQ_SHOW_FIRSTS(DOUT)
	RAWDAQ_SHOW_FIRSTS(CNTRIN_32bits)
	RAWDAQ_SHOW_FIRSTS(CNTRIN_double)

	cout << endl;
}

