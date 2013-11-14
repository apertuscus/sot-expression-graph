#include <sot/core/matrix-homogeneous.hh>
#include <kdl/expressiontree.hpp>
#ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
#define __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
using namespace dynamicgraph::sot;
KDL::Rotation mlHom2KDLRot (const MatrixHomogeneous &  M);



const std::string name_joints_sot_order[] ={
		 "Waist_X",
		 "Waist_Y",
		 "Waist_Z",
		 "Waist_Phi",
		 "Waist_Theta",
		 "Waist_Psi",
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
		 "RToePitch"
};
#define N_OF_JOINTS 39

#endif







