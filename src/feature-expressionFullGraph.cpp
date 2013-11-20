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

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/factory.hh>
#include <ros/package.h>

using namespace dynamicgraph::sot;
using namespace std;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureExpressionFullGraph,"FeatureExpressionFullGraph");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#define CHECK( a ) \
		{ bool result = a; \
		if (!result) { \
			std::cerr << __FILE__ << " : " << __LINE__ << " : FAILED: " #a << std::endl;\
			exit(1);\
		} \
		}

ExpressionMap FeatureExpressionFullGraph::create_fk_from_urdf(Context::Ptr ctx)
{
	UrdfExpr2 urdf;
	std::string path_to_file=
			ros::package::getPath("romeo_description")
			+"/urdf/romeo_small.urdf";

	CHECK( urdf.readFromFile(path_to_file));


	//addTransform gives a name (first arg.)
	//  to the transformations between base_link (3° arg.) and the
	//  left/right gripper tool frames ("second arg")

	CHECK( urdf.addTransform("left_hand","l_wrist","l_sole")  );
	//CHECK( urdf.addTransform("right_hand","r_wrist","l_sole")  );

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


Expression<double>::Ptr FeatureExpressionFullGraph::distance_btw_origins
	(Expression<KDL::Frame>::Ptr o1,Expression<KDL::Frame>::Ptr o2)
{
	Expression<double>::Ptr d=norm(origin(o1)-origin(o2));
	return d;
}


FeatureExpressionFullGraph::
FeatureExpressionFullGraph( const string& ExpressionGraph )
: FeatureAbstract( ExpressionGraph )
,jointSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(vector)::joint" )
//,w_T_ee_SIN ( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position_ee" )
,w_T_obj_SIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(matrixHomo)::position_obj" )
//,articularJacobianSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(matrix)::Jq" )
,positionRefSIN( NULL,"sotfeatureExpressionFullGraph("+name+")::input(double)::positionRef" )
{

	//the jacobian depends by
	  errorSOUT.addDependency( jointSIN );
	//jacobianSOUT.addDependency( w_T_ee_SIN );
	jacobianSOUT.addDependency( w_T_obj_SIN );
	jacobianSOUT.addDependency( positionRefSIN );//this one is not probably necessary...
	//jacobianSOUT.addDependency( articularJacobianSIN );
	//the output depends by
	//jacobianSOUT.addDependency( w_T_ee_SIN );
	jacobianSOUT.addDependency( w_T_obj_SIN );
	errorSOUT.addDependency( positionRefSIN );

	//signalRegistration(  jointSIN<<w_T_ee_SIN<<w_T_obj_SIN<<articularJacobianSIN<<positionRefSIN );
	signalRegistration(jointSIN<<w_T_obj_SIN<<positionRefSIN );

	/***
	 * init of expression graph
	 * */
	Context::Ptr ctx = create_context();
	ctx->addType("robot");

	ExpressionMap r=create_fk_from_urdf(ctx);

	//build up index table
	ind_to_sot=index_lookup_table(ctx,name_joints_sot_order ,N_OF_JOINTS);

	//save the initial position for saving data that are not joint values (e.g. external frame, input)
	//TODO this stuff should be substituted by variable type.
	st_ind_ex=ind_to_sot.size();
	q_ex.resize(st_ind_ex,0);

	//frame of the robot ee, w.r.t a world frame
	w_T_ee=r["left_hand"];
	Expression<KDL::Vector>::Ptr w_p_obj=KDL::vector(
			input(st_ind_ex),
			input(st_ind_ex+1),
			input(st_ind_ex+2));
	Expression<KDL::Rotation>::Ptr w_R_obj= inputRot(st_ind_ex+3);
	//frame of the the object, w.r.t the same world frame
	w_T_obj=cached<Frame> (frame(w_R_obj,  w_p_obj));
	//in and out
	Sreference= input(st_ind_ex+6);


	/*here i must give the geometric primitives configuration
	 * possible primitive are :
	 * point,
	 * versor,
	 * line,
	 * plane.
	 *
	 * each primitive can be combined with specific relation
	 * */
	geometric_primitive::point p1{w_T_obj,Constant(Vector(0,0,0))};
	geometric_primitive::point p2{w_T_ee,Constant(Vector(0,0,0))};
	Soutput=geometric_primitive::point_point_distance(p1,p2);

}



/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureExpressionFullGraph::
getDimension( unsigned int & dim, int /*time*/ )
{
	sotDEBUG(25)<<"# In {"<<endl;

	return dim=1;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */


ml::Matrix& FeatureExpressionFullGraph::
computeJacobian( ml::Matrix& J,int time )
{
	sotDEBUG(15)<<"# In {"<<endl;

	//read signals!
	//const MatrixHomogeneous &  w_T_ee=  w_T_ee_SIN (time);
	const MatrixHomogeneous &  w_T_obj=  w_T_obj_SIN (time);
	//const ml::Matrix & Jq = articularJacobianSIN(time);
	const double & pos_des = positionRefSIN(time);
	const ml::Vector q_sot = jointSIN(time);

	//copy angles in the vector for the relation
	for (unsigned int i=0;i<st_ind_ex;i++)
	{
		q_ex[i]=q_sot(ind_to_sot[i]);
	}

	Soutput->setInputValues(q_ex);

	//TODO use variable type for not controllable objects
	//copy object pose and external output
	for( int i=0;i<3;++i )
		Soutput->setInputValue(st_ind_ex+i, w_T_obj.elementAt( i,3 ));
	Soutput->setInputValue(st_ind_ex+3,mlHom2KDLRot(w_T_obj));
	//copy reference
	Soutput->setInputValue(st_ind_ex+7,pos_des);


	//evaluate once to update the tree
	Soutput->value();

	//resize the matrices
	//J.resize(1,N_OF_JOINTS);//todo jonit are 39 not 37

	J.resize(1,39);

	/*
	 * compute the Jacobian starting to a zero matrix,
	 * 	i fill in only the lines that are referred in the index map
	 * 	*/
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


	//read signals!
	//const MatrixHomogeneous &  w_T_ee=  w_T_ee_SIN (time);
	const MatrixHomogeneous &  w_T_obj=  w_T_obj_SIN (time);
	//const ml::Matrix & Jq = articularJacobianSIN(time);
	const double & pos_des = positionRefSIN(time);
	const ml::Vector q_sot = jointSIN(time);
	//cout<< "q_sot.size(): "<<q_sot.size()<<endl;
	//cout<< "q_ex.size(): "<<q_ex.size()<<endl;
	//cout<< "st_ind_ex: "<<st_ind_ex<<endl;
	//copy angles in the vector for the relation
	for (unsigned int i=0;i<st_ind_ex;i++)
	{
		q_ex[i]=q_sot(ind_to_sot[i]);
		//cerr<<"accessing: "<<ind_to_sot[i]<<endl;
	}

	Soutput->setInputValues(q_ex);

	//TODO use variable type for not controllable objects
	//copy object pose and external output
	for( int i=0;i<3;++i )
		Soutput->setInputValue(st_ind_ex+i, w_T_obj.elementAt( i,3 ));
	Soutput->setInputValue(st_ind_ex+3,mlHom2KDLRot(w_T_obj));
	//copy reference
	Soutput->setInputValue(st_ind_ex+7,pos_des);



	// resize the result vector and
	res.resize(1);
	//evaluate the result.
	res(0)=Soutput->value();


	sotDEBUGOUT(15);
	return res ;
}

void FeatureExpressionFullGraph::
display( std::ostream& os ) const
{
	os <<"featureExpressionFullGraph <"<<name<<">";
}



void FeatureExpressionFullGraph::
commandLine( const std::string& cmdLine,
		std::istringstream& cmdArgs,
		std::ostream& os )
{
	if( cmdLine=="help" )
	{
		os << "featureExpressionFullGraph: "<<endl;
		Entity::commandLine( cmdLine,cmdArgs,os );
	}
	else  //FeatureAbstract::
		Entity::commandLine( cmdLine,cmdArgs,os );

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
