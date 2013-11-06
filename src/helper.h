#include <sot/core/matrix-homogeneous.hh>
#include <kdl/expressiontree.hpp>
#ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
#define __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
using namespace dynamicgraph::sot;
KDL::Rotation mlHom2KDLRot (const MatrixHomogeneous &  M);



const std::string name_joints_sot_order[] ={
		 "TrunkYaw",
		 "NeckYaw",
		 "NeckPitch",
		 "HeadPitch",
		 "HeadRoll",
		 "LShoulderPitch",
		 "LShoulderYaw",
		 "LElbowRoll",
		 "LElbowYaw",
		 "LWristRoll",
		 "LWristYaw",
		 "LWristPitch",
		 "RShoulderPitch",
		 "RShoulderYaw",
		 "RElbowRoll",
		 "RElbowYaw",
		 "RWristRoll",
		 "RWristYaw",
		 "RWristPitch",
		 "LHipYaw",
		 "LHipRoll",
		 "LHipPitch",
		 "LKneePitch",
		 "LAnklePitch",
		 "LAnkleRoll",
		 "LToePitch",
		 "RHipYaw",
		 "RHipRoll",
		 "RHipPitch",
		 "RKneePitch",
		 "RAnklePitch",
		 "RAnkleRoll",
		 "RToePitch",
		 "LEyeYaw",
		 "LEyePitch",
		 "REyeYaw",
		 "REyePitch"
};
#define N_OF_JOINTS 37

#endif







