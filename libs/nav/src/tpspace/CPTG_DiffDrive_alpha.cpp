/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

template <> const char * mrpt::utils::CSerializer<CPTG_DiffDrive_alpha>::getClassName() { return "CPTG_DiffDrive_alpha";}

void CPTG_DiffDrive_alpha::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	CPTG_DiffDrive_CollisionGridBased ::loadFromConfigFile(cfg,sSection);
	
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(cte_a0v_deg,double, cte_a0v, cfg,sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(cte_a0w_deg,double, cte_a0w, cfg,sSection);
}
void CPTG_DiffDrive_alpha::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;
	CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(cfg,sSection);

	cfg.write(sSection,"cte_a0v_deg",mrpt::utils::RAD2DEG(cte_a0v),   WN,WV, "Contant for vel profile [deg].");
	cfg.write(sSection,"cte_a0w_deg",mrpt::utils::RAD2DEG(cte_a0v),   WN,WV, "Contant for omega profile [deg].");

	MRPT_END
}

std::string CPTG_DiffDrive_alpha::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"CPTG_DiffDrive_alpha,av=%udeg,aw=%udeg",(int)RAD2DEG(cte_a0v),(int)RAD2DEG(cte_a0w) );
	return std::string(str);
}

namespace mrpt { namespace utils {
template <> void CSerializer<CPTG_DiffDrive_alpha>::readFromStream(CPTG_DiffDrive_alpha &o, mrpt::utils::CStream &in, int version)
{
	o.internal_readFromStream(in);

	switch (version)
	{
	case 0:
		in  >> o.cte_a0v >> o.cte_a0w;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

template <> void CSerializer<CPTG_DiffDrive_alpha>::writeToStream(const CPTG_DiffDrive_alpha &o, mrpt::utils::CStream &out, int *version)
{
	if (version) 
	{
		*version = 0;
		return;
	}

	o.internal_writeToStream(out);
	out << o.cte_a0v << o.cte_a0w;
}
}}

/*---------------------------------------------------------------
						ptgDiffDriveSteeringFunction
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(t); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
    float At_a = alpha - phi;

    while (At_a>M_PI) At_a -= (float) M_2PI;
    while (At_a<-M_PI) At_a += (float) M_2PI;

    v = V_MAX * exp(-square( At_a / cte_a0v ));
    w=  W_MAX * (-0.5f + (1/(1+exp(-At_a/cte_a0w))));
}


void CPTG_DiffDrive_alpha::loadDefaultParams()
{
	CPTG_DiffDrive_CollisionGridBased::loadDefaultParams();

	cte_a0v = mrpt::utils::DEG2RAD(45.0);
	cte_a0w = mrpt::utils::DEG2RAD(45.0);
}
