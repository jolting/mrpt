/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

using namespace mrpt::nav;

template <> const char * mrpt::utils::CSerializer<CLogFileRecord>::getClassName() { return "CLogFileRecord";}

/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CLogFileRecord::CLogFileRecord() :
	nPTGs     ( 0 ),
	nSelectedPTG(-1),
	robotPoseLocalization(0,0,0),
	robotPoseOdometry(0,0,0),
	relPoseSense(0,0,0),
	relPoseVelCmd(0,0,0),
	WS_targets_relative(),
	cur_vel(0,0,0),
	cur_vel_local(0,0,0),
	robotShape_radius(.0),
	ptg_index_NOP(-1),
	ptg_last_k_NOP(0),
	rel_cur_pose_wrt_last_vel_cmd_NOP(0,0,0),
	rel_pose_PTG_origin_wrt_sense_NOP(0,0,0)
{
	infoPerPTG.clear();
	WS_Obstacles.clear();
}

namespace mrpt { namespace utils {
/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
template <> void CSerializer<CLogFileRecord>::writeToStream(const CLogFileRecord& o, mrpt::utils::CStream &out,int *version)
{
	if (version)
		*version = 26;
	else
	{
		uint32_t	i,n;

		// Version 0 ---------
		n = o.infoPerPTG.size();
		out << n;
		for (i=0;i<n;i++)
		{
			out << o.infoPerPTG[i].PTG_desc.c_str();

			uint32_t m = o.infoPerPTG[i].TP_Obstacles.size();
			out << m;
			if (m) out.WriteBuffer((const void*)&(*o.infoPerPTG[i].TP_Obstacles.begin()), m * sizeof(o.infoPerPTG[i].TP_Obstacles[0]));

			out << o.infoPerPTG[i].TP_Targets;  // v8: CPoint2D -> TPoint2D. v26: vector
			out << o.infoPerPTG[i].TP_Robot; // v17
			out << o.infoPerPTG[i].timeForTPObsTransformation << o.infoPerPTG[i].timeForHolonomicMethod; // made double in v12
			out << o.infoPerPTG[i].desiredDirection << o.infoPerPTG[i].desiredSpeed << o.infoPerPTG[i].evaluation; // made double in v12
			// removed in v23: out << o.evaluation_org << o.evaluation_priority; // added in v21
			out << o.infoPerPTG[i].HLFR;

			// Version 9: Removed security distances. Added optional field with PTG info.
			const bool there_is_ptg_data = o.infoPerPTG[i].ptg ? true : false;
			out << there_is_ptg_data;
			if (there_is_ptg_data)
				out << o.infoPerPTG[i].ptg;

			// Was: out << o.infoPerPTG[i].clearance.raw_clearances; // v19
			o.infoPerPTG[i].clearance.writeToStream(out); // v25
		}
		out << o.nSelectedPTG << o.WS_Obstacles;
		out << o.WS_Obstacles_original; // v20
		
		// Removed v24: out << o.robotOdometryPose;
		out << o.robotPoseLocalization << o.robotPoseOdometry; // v24

		out << o.WS_targets_relative; //v8, v26: vector
		// v16:
		out << o.ptg_index_NOP << o.ptg_last_k_NOP; 
		out << o.rel_cur_pose_wrt_last_vel_cmd_NOP << o.rel_pose_PTG_origin_wrt_sense_NOP; // v24: CPose2D->TPose2D
		
		// Removed: v24. out << ptg_last_curRobotVelLocal; // v17
		o.ptg_last_navDynState.writeToStream(out); // v24

		if (o.ptg_index_NOP<0)
			out << o.cmd_vel /*v10*/ << o.cmd_vel_original; // v15

		// Previous values: REMOVED IN VERSION #6
		n = o.robotShape_x.size();
		out << n;
		if (n) {
			out.WriteBuffer((const void*)&(*o.robotShape_x.begin()), n*sizeof(o.robotShape_x[0]));
			out.WriteBuffer((const void*)&(*o.robotShape_y.begin()), n*sizeof(o.robotShape_y[0]));
		}

		// Version 1 ---------
		out << o.cur_vel<< o.cur_vel_local; /*v10*/
		//out << o.estimatedExecutionPeriod; // removed v13

		// Version 3 ----------
		for (i=0;i<o.infoPerPTG.size();i++) {
			out << o.infoPerPTG[i].evalFactors; // v22: this is now a TParameters
		}

		out << o.nPTGs; // v4
		// out << o.timestamp; // removed v13
		out << o.robotShape_radius; // v11
		//out << o.cmd_vel_filterings; // added v12: Removed in v15

		out << o.values << o.timestamps; // v13

		out << o.relPoseSense << o.relPoseVelCmd; // v14, v24 changed CPose2D->TPose2D

		// v15: cmd_vel converted from std::vector<double> into CSerializable
		out << o.additional_debug_msgs; // v18

		o.navDynState.writeToStream(out); // v24
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
template <> void CSerializer<CLogFileRecord>::readFromStream(CLogFileRecord &o, mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
	case 17:
	case 18:
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
	case 24:
	case 25:
	case 26:
		{
			// Version 0 --------------
			uint32_t  i,n;

			o.infoPerPTG.clear();

			in >> n;
			o.infoPerPTG.resize(n);
			for (i=0;i<n;i++)
			{
				auto &ipp = o.infoPerPTG[i];
				char str[256];
				in >> str;
				ipp.PTG_desc = std::string(str);

				int32_t m;
				in >> m;
				ipp.TP_Obstacles.resize(m);
				if (m) in.ReadBufferFixEndianness( &(*ipp.TP_Obstacles.begin()), m );

				ipp.TP_Targets.clear();
				if (version >= 8)
				{
					if (version >= 26) {
						in >> ipp.TP_Targets;
					}
					else {
						mrpt::math::TPoint2D trg;
						in >> trg;
						ipp.TP_Targets.push_back(trg);
					}
				}
				else
				{
					mrpt::poses::CPoint2D pos;
					in >> pos;
					ipp.TP_Targets.push_back(mrpt::math::TPoint2D(pos));
				}
				if (version >= 17)
					in >> ipp.TP_Robot;
				else ipp.TP_Robot = mrpt::math::TPoint2D(0, 0);

				if (version>=12) {
					in >> ipp.timeForTPObsTransformation >> ipp.timeForHolonomicMethod;
					in >> ipp.desiredDirection >> ipp.desiredSpeed >> ipp.evaluation;
				} else {
					in.ReadAsAndCastTo<float,double>(ipp.timeForTPObsTransformation);
					in.ReadAsAndCastTo<float,double>(ipp.timeForHolonomicMethod);
					in.ReadAsAndCastTo<float,double>(ipp.desiredDirection);
					in.ReadAsAndCastTo<float,double>(ipp.desiredSpeed);
					in.ReadAsAndCastTo<float,double>(ipp.evaluation);
				}
				if (version >= 21 && version <23) {
					double evaluation_org, evaluation_priority;
					in >> evaluation_org >> evaluation_priority;
				}

				in >> ipp.HLFR;

				if (version>=9) // Extra PTG info
				{
					ipp.ptg.reset();

					bool there_is_ptg_data;
					in >> there_is_ptg_data;
					if (there_is_ptg_data)
						ipp.ptg = std::dynamic_pointer_cast<CParameterizedTrajectoryGenerator>( in.ReadObject() );
				}

				if (version >= 19)
				{
					if (version < 25) {
						std::vector<std::map<double,double> > raw_clearances;
						in >> raw_clearances;
						ipp.clearance.resize(raw_clearances.size(), raw_clearances.size());
						for (size_t i = 0; i < raw_clearances.size(); i++)
							ipp.clearance.get_path_clearance_decimated(i) = raw_clearances[i];
					}
					else {
						ipp.clearance.readFromStream(in);
					}
				}
				else {
					ipp.clearance.clear();
				}
			}

			in >> o.nSelectedPTG >> o.WS_Obstacles;
			if (version >= 20) {
				in >> o.WS_Obstacles_original; // v20
			}
			else {
				o.WS_Obstacles_original = o.WS_Obstacles;
			}

			if (version < 24) {
				mrpt::poses::CPose2D robotOdometryPose;
				in >> robotOdometryPose;
				o.robotPoseOdometry     = robotOdometryPose;
				o.robotPoseLocalization = robotOdometryPose;
			}
			else
			{
				in >> o.robotPoseLocalization >> o.robotPoseOdometry; // v24
			}

			o.WS_targets_relative.clear();
			if (version >= 8)
			{
				if (version >= 26)
				{
					in >> o.WS_targets_relative;
				}
				else
				{
					mrpt::math::TPoint2D trg;
					in >> trg;
					o.WS_targets_relative.push_back(trg);
				}
			}
			else
			{
				mrpt::poses::CPoint2D pos;
				in >> pos;
				o.WS_targets_relative.push_back(mrpt::math::TPoint2D(pos));
			}

			if (version >= 16) {
				in >> o.ptg_index_NOP >> o.ptg_last_k_NOP;
				if (version >= 24) {
					in >> o.rel_cur_pose_wrt_last_vel_cmd_NOP >> o.rel_pose_PTG_origin_wrt_sense_NOP;
				}
				else
				{
					mrpt::poses::CPose2D crel_cur_pose_wrt_last_vel_cmd_NOP,crel_pose_PTG_origin_wrt_sense_NOP;
					in >> crel_cur_pose_wrt_last_vel_cmd_NOP >> crel_pose_PTG_origin_wrt_sense_NOP;
					o.rel_cur_pose_wrt_last_vel_cmd_NOP = crel_cur_pose_wrt_last_vel_cmd_NOP;
					o.rel_pose_PTG_origin_wrt_sense_NOP = crel_pose_PTG_origin_wrt_sense_NOP;
				}
			}
			else {
				o.ptg_index_NOP = -1;
			}
			if (version >= 17 && version < 24) {
				in >> o.ptg_last_navDynState.curVelLocal;
			}
			if (version >= 24) {
				o.ptg_last_navDynState.readFromStream(in);
			}

			if (version >= 10) {
				if (version >= 15) {
					if (o.ptg_index_NOP<0)
						in >> o.cmd_vel;
				}
				else {
					std::vector<double> vel;
					in >> vel;
					if (vel.size() == 2)
						o.cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
					else o.cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(new mrpt::kinematics::CVehicleVelCmd_Holo);
					for (size_t i = 0; i < o.cmd_vel->getVelCmdLength(); i++)
						o.cmd_vel->setVelCmdElement(i, vel[i]);
				}
			}
			else {
				float v, w;
				in >> v >> w;
				o.cmd_vel = mrpt::kinematics::CVehicleVelCmd::Ptr(new mrpt::kinematics::CVehicleVelCmd_DiffDriven);
				o.cmd_vel->setVelCmdElement(0, v);
				o.cmd_vel->setVelCmdElement(0, w);
			}

			if (version>=15 && o.ptg_index_NOP<0)
				in >> o.cmd_vel_original;

			if (version < 13) {
				float old_exec_time;  in >> old_exec_time;
				o.values["executionTime"] = old_exec_time;
			}

			if (version<6)
			{
				mrpt::math::CVectorFloat prevV,prevW,prevSelPTG;

				// Previous values: (Removed in version 6)
				in >> n;
				prevV.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevV.begin()),n);

				in >> n;
				prevW.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevW.begin()),n);

				in >> n;
				prevSelPTG.resize(n);
				if (n) in.ReadBufferFixEndianness( &(*prevSelPTG.begin()),n);
			}

			in >> n;
			o.robotShape_x.resize(n);
			o.robotShape_y.resize(n);
			if (n) {
				in.ReadBufferFixEndianness( &(*o.robotShape_x.begin()), n);
				in.ReadBufferFixEndianness( &(*o.robotShape_y.begin()), n);
			}

			if (version > 0)
			{	// Version 1 --------------
				if (version >= 10) {
					in >> o.cur_vel >> o.cur_vel_local;
				}
				else {
					float actual_v, actual_w;
					in >> actual_v >> actual_w;
					o.cur_vel = mrpt::math::TTwist2D(0,0,0);
					o.cur_vel_local= mrpt::math::TTwist2D(actual_v, .0, actual_w );
				}
			}
			else
			{	// Default values for old versions:
				o.cur_vel = mrpt::math::TTwist2D(0,0,0);
			}

			if (version < 13 && version>1) {
				float old_estim_period;  in >> old_estim_period;
				o.values["estimatedExecutionPeriod"] = old_estim_period;
			}

			for (i = 0; i < o.infoPerPTG.size(); i++) {
				o.infoPerPTG[i].evalFactors.clear();
			}
			if (version > 2)
			{
				// Version 3..22 ----------
				for (i=0;i<o.infoPerPTG.size();i++)
				{
					if (version < 22) {
						in >> n;
						for (unsigned int j = 0; j < n; j++) {
							float f;
							in >> f; 
							o.infoPerPTG[i].evalFactors[mrpt::format("f%u", j)] = f;
						}
					}
					else {
						in >> o.infoPerPTG[i].evalFactors;
					}
				}

			}

			if (version > 3)
			{
				// Version 4 ----------
				in >> o.nPTGs;
				if (version <9)  // Old "securityDistances", now unused.
				{
					in >> n;
					float dummy;
					for (i=0;i<n;i++)
						in >> dummy;
				}
			}
			else
			{
				o.nPTGs = o.infoPerPTG.size();
			}

			if (version > 4)
			{
				if (version < 10)
				{
					int32_t navigatorBehavior; // removed in v10
					in >> navigatorBehavior;
				}

				if (version<6)  // Removed in version 6:
				{
					mrpt::poses::CPoint2D doorCrossing_P1,doorCrossing_P2;
					in >> doorCrossing_P1 >> doorCrossing_P2;
				}
			}

			if (version>6 && version<13) {
				mrpt::system::TTimeStamp tt; in >> tt;
				o.timestamps["tim_start_iteration"] = tt;
			}

			if (version>=11) {
				in >> o.robotShape_radius;
			} else {
				o.robotShape_radius = 0.5;
			}

			if (version >= 12 && version<15) {
				std::vector<std::vector<double> > dummy_cmd_vel_filterings;
				in >> dummy_cmd_vel_filterings;
			}

			if (version >= 13) {
				in >> o.values >> o.timestamps;
			}
			else {
				o.values.clear();
				o.timestamps.clear();
			}

			if (version >= 14) {
				if (version >= 24) {
					in >> o.relPoseSense >> o.relPoseVelCmd;
				}
				else {
					mrpt::poses::CPose2D crelPoseSense, crelPoseVelCmd;
					in >> crelPoseSense >> crelPoseVelCmd;
					o.relPoseSense = crelPoseSense;
					o.relPoseVelCmd = crelPoseVelCmd;
				}
			}
			else {
				o.relPoseSense = o.relPoseVelCmd = mrpt::math::TPose2D();
			}

			if (version>=18) 
			     in >> o.additional_debug_msgs;
			else o.additional_debug_msgs.clear();

			if (version>=24)
				 o.navDynState.readFromStream(in);
			else {
				o.navDynState = CParameterizedTrajectoryGenerator::TNavDynamicState();
				o.navDynState.curVelLocal = o.cur_vel_local;
				if (!o.WS_targets_relative.empty()) 
					o.navDynState.relTarget = o.WS_targets_relative[0];
			}


		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
}}
