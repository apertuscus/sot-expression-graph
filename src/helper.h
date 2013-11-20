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



namespace geometric_primitive
{
using namespace KDL;

struct point
{
	Expression<KDL::Frame>::Ptr o;
	Expression<KDL::Vector>::Ptr p;
};
//no check done on versor in building up
struct versor
{
	Expression<Frame>::Ptr o;
	Expression<Vector>::Ptr v;
};
struct line
{
	Expression<Frame>::Ptr o;
	Expression<Vector>::Ptr dir;
	Expression<Vector>::Ptr p;
};
struct plane
{
	Expression<Frame>::Ptr o;
	Expression<Vector>::Ptr normal;
	Expression<Vector>::Ptr p;
};

/*for all the expressions we soppose that the frames are expressed in a common reference*/
Expression<double>::Ptr point_point_distance(point p1,point p2);
Expression<double>::Ptr line_point_distance(point p, line l);
Expression<double>::Ptr projection_of_point_on_line(point p, line l);
Expression<double>::Ptr distance_from_lines (line l1, line l2);

 /* Expression<double>::Ptr distance_o1_f1 (line l1, line l2);
 * Expression<double>::Ptr distance_o2_f2 (line l1, line l2);
 * Expression<double>::Ptr point_plane_distance (point p, plane pl);
//angles
 * Expression<double>::Ptr angle_btw_versors (versor v1, versor v2);
 * Expression<double>::Ptr incident_angle (versor v, plane pl);
 * Expression<double>::Ptr angles_btw_planes (plane pl1, plane pl2);
 * */
}
#endif






