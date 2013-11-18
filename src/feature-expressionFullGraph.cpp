/*
 * Copyright 2013,
 * Gianni Borghesan
 *
 *	KULEUVEN
 *
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>
#include "feature-expressionFullGraph.h"
#include "helper.h"
#include <sot/core/exception-feature.hh>
#include <dynamic-graph/all-commands.h>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/factory.hh>
#include <ros/package.h>

using namespace dynamicgraph::sot;
using namespace std;
using namespace dynamicgraph;
using namespace KDL;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureExpressionFullGraph,"FeatureExpressionFullGraph");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

ExpressionMap FeatureExpressionFullGraph::create_fk_from_urdf(
		Context::Ptr ctx, const std::string & op1,
		const std::string & op2, const std::string & label)
{
	UrdfExpr2 urdf;
	std::string path_to_file=
			ros::package::getPath("romeo_description")
			+"/urdf/romeo_small.urdf";

	if (!urdf.readFromFile(path_to_file))
		throw ("Unable to read file"+path_to_file);


	//addTransform gives a name (first arg.)
	//  to the transformations between base_link (3° arg.) and the
	//  left/right gripper tool frames ("second arg")
	if (! urdf.addTransform(label, op1, op2)  )
		throw ("Unable to create the transformation between " + op1 + " and " + op2);

	//adding transformation for all the kinematic tree, so that all the joints are added
	ExpressionMap r = urdf.getExpressions(ctx);

	return r;
}

/* * this function creates a vector of indexes from
 *  the map name_joints (the ordered list of joint of stack of task)
 *  to a list of joints that are in the Context.
 *  */
std::vector<int> FeatureExpressionFullGraph::index_lookup_table
(const Context::Ptr ctx, const std::string name_joints[],const int n_of_joints)
{
	std::vector<int> jointndx;
	for (int i=0;i<n_of_joints; ++i) {
		int nr = ctx->getScalarNdx(name_joints[i]);
		if (nr==-1) //none of the relations depends on the transformations defined
		{
			cout<< nr <<"\t"<< name_joints[i] << "  Not Found"  << endl;
		}
		else
		{
			jointndx.push_back(i);
			cout<< i <<"\t"<< name_joints[i] << "  saved in position\t"  <<jointndx.size()	- 1 << endl;
		}
	}
	return jointndx;
}

FeatureExpressionFullGraph::
FeatureExpressionFullGraph( const string& ExpressionGraph )
: FeatureAbstract( ExpressionGraph )
,jointSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(vector)::joint" )
//,w_T_ee_SIN( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position_ee" )
,w_T_obj_SIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(matrixHomo)::position_obj" )
//,articularJacobianSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(matrix)::Jq" )
,positionRefSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(double)::positionRef" )
,oneStepOfControlSignal(boost::bind(&FeatureExpressionFullGraph::oneStepOfControl,this,_1,_2),
		jointSIN << w_T_obj_SIN << positionRefSIN,
		"sotfeatureExpressionFullGraph("+name+")::output(double)::oneStepOfControlSignal")
{

	//the jacobian depends by
	errorSOUT.addDependency(jointSIN);
	errorSOUT.addDependency(oneStepOfControlSignal);
	errorSOUT.addDependency( positionRefSIN );

	jacobianSOUT.addDependency(w_T_obj_SIN);
	jacobianSOUT.addDependency(oneStepOfControlSignal);
	jacobianSOUT.addDependency( positionRefSIN );//this one is not probably necessary...

	//signalRegistration(  jointSIN<<<<w_T_obj_SIN<<articularJacobianSIN<<positionRefSIN );
	signalRegistration(jointSIN<<w_T_obj_SIN<<positionRefSIN );

	/***
	 * init of expression graph
	 * */
	ctx_ = create_context();
	ctx_->addType("robot");

	// ...
	initCommands();
}

void FeatureExpressionFullGraph::setChain
(const std::string & label, const std::string & op1, const std::string & op2)
{
	try
	{
	ExpressionMap expr_map=create_fk_from_urdf(ctx_, op1, op2, label);

	//build up index table
	ind_to_sot=index_lookup_table(ctx_,name_joints_sot_order ,N_OF_JOINTS);

	//save the initial position for saving data that are not joint values (e.g. external frame, input)
	//TODO this stuff should be substituted by variable type.
	st_ind_ex=ind_to_sot.size();
	q_ex.resize(st_ind_ex,0);

	//frame of the robot ee, w.r.t a world frame
	w_T_ee = expr_map[label];
	Expression<KDL::Vector>::Ptr w_p_obj=KDL::vector(
			input(st_ind_ex),
			input(st_ind_ex+1),
			input(st_ind_ex+2));
	Expression<KDL::Rotation>::Ptr w_R_obj= inputRot(st_ind_ex+3);
	//frame of the the object, w.r.t the same world frame
	w_T_obj=cached<Frame> (frame(w_R_obj,  w_p_obj));
	//in and out
	Sreference= input(st_ind_ex+7);

	geometric_primitive::point p1{w_T_obj,Constant(KDL::Vector(0,0,0))};
	geometric_primitive::point p2{w_T_ee,Constant(KDL::Vector(0,0,0))};
	Soutput=geometric_primitive::point_point_distance(p1,p2);

	dimension_ = 1;
	}
	catch(std::string & error)
	{
		std::cerr << error << std::endl;
		return;
	}
}



/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureExpressionFullGraph::
getDimension( unsigned int & dim, int /*time*/ )
{
	return dimension_;
}



int& FeatureExpressionFullGraph::
oneStepOfControl( int& dummy,int time )
{
	//read signals!
	const MatrixHomogeneous &  w_T_obj=  w_T_obj_SIN (time);
	const double & pos_des = positionRefSIN(time);
	const ml::Vector & q_sot = jointSIN(time);

	//copy angles in the vector for the relation
	for (unsigned int i=0;i<st_ind_ex;i++)
		q_ex[i]=q_sot(ind_to_sot[i]);

	// joint positions
	Soutput->setInputValues(q_ex);

	/// TODO use variable type for not controllable objects
	//copy object pose and external output
	for( int i=0;i<3;++i )
		Soutput->setInputValue(st_ind_ex+i, w_T_obj.elementAt( i,3 ));

	// 4 dof for the rotation
	Soutput->setInputValue(st_ind_ex+3,mlHom2KDLRot(w_T_obj));

	//copy reference
	Soutput->setInputValue(st_ind_ex+7,pos_des);

	return dummy;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
ml::Matrix& FeatureExpressionFullGraph::
computeJacobian( ml::Matrix& J,int time )
{
	sotDEBUG(15)<<"# In {"<<endl;

	// triger one computation
	oneStepOfControlSignal(time);

	//evaluate once to update the tree
	Soutput->value();

	/* compute the Jacobian starting to a zero matrix,
	 * 	i fill in only the lines that are referred in the index map
	 * 	*/
	J.resize(dimension_, N_OF_JOINTS);
	J.setZero();
	for (int i=0;i<st_ind_ex;++i)
		J(0,ind_to_sot[i])=Soutput->derivative(i);

	//return result
	sotDEBUG(15)<<"# Out }"<<endl;
	return J;
}

/** Compute the error between two visual features from a subset
 *a the possible features.
 */
ml::Vector&
FeatureExpressionFullGraph::computeError( ml::Vector& res,int time )
{
	sotDEBUGIN(15);

	//triger one computation
	oneStepOfControlSignal(time);

	// resize the result vector and
	res.resize(dimension_);

	//evaluate the result.
	res(0)=Soutput->value();

	sotDEBUGOUT(15);
	return res ;
}

std::string FeatureExpressionFullGraph::getDocString () const
{
	return ("FeatureExpressionFullGraph");
}


void FeatureExpressionFullGraph::initCommands()
{
	namespace dc = dynamicgraph::command;
	addCommand("setChain",
	 dc::makeCommandVoid3(*this, &FeatureExpressionFullGraph::setChain,
	 "chain named [arg1] corresponding to the expression of [arg2] expressed in frame [arg3]"));
//	addCommand("setType",
//	 dc::makeCommandVoid3(*this, &FeatureExpressionFullGraph::setType,
//	 ""));
}
