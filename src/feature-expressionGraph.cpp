/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
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
#include "feature-expressionGraph.h"
#include <sot/core/exception-feature.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace std;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureExpressionGraph,"FeatureExpressionGraph");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureExpressionGraph::
FeatureExpressionGraph( const string& ExpressionGraph )
  : FeatureAbstract( ExpressionGraph )
    ,w_T_ee_SIN ( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position_ee" )
    ,w_T_obj_SIN( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position_obj" )
    ,articularJacobianSIN( NULL,"sotFeatureExpressionGraph("+name+")::input(matrix)::Jq" )
    ,positionRefSIN( NULL,"sotFeatureExpressionGraph("+name+")::input(double)::positionRef" )
{
  //the jacobian depends by
  jacobianSOUT.addDependency( w_T_ee_SIN );
  jacobianSOUT.addDependency( w_T_obj_SIN );
  jacobianSOUT.addDependency( positionRefSIN );//this one is not probably necessary...
  jacobianSOUT.addDependency( articularJacobianSIN );
  //the output depends by
  jacobianSOUT.addDependency( w_T_ee_SIN );
  jacobianSOUT.addDependency( w_T_obj_SIN );
  errorSOUT.addDependency( positionRefSIN );

  signalRegistration( w_T_ee_SIN<<w_T_obj_SIN
                      <<articularJacobianSIN<<positionRefSIN );

  /***
   * init of expression graph
   * */
  Expression<KDL::Vector>::Ptr w_p_ee=KDL::vector(input(0), input(1),input(2));
  Expression<KDL::Rotation>::Ptr w_R_ee = inputRot(3);
  //frame of the robot ee, w.r.t a world frame
  w_T_ee= frame(w_R_ee,  w_p_ee);
  Expression<KDL::Vector>::Ptr w_p_obj=KDL::vector(input(6), input(7),input(8));
  Expression<KDL::Rotation>::Ptr w_R_obj= inputRot(9);
  //frame of the the object, w.r.t the same world frame
  w_T_obj=cached<Frame> (frame(w_R_obj,  w_p_obj));
  //in and out
  Sreference= input(12);
  Soutput=cached<double>(Sreference-norm(w_p_ee-w_p_obj));
  //declare dependecies
  //copy positions


  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureExpressionGraph::
getDimension( unsigned int & dim, int /*time*/ )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=1;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */

KDL::Rotation mlHom2KDLRot (const MatrixHomogeneous &  M)
{
	KDL::Rotation Rkdl;
	for( int i=0;i<3;++i )
	    for( int j=0;j<3;++j )
	    	Rkdl( i,j ) = M.elementAt( i,j );
	//M.extract(R);
	//R(i,j);
	return Rkdl;

}
ml::Matrix& FeatureExpressionGraph::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  //read signals!
  const MatrixHomogeneous &  w_T_ee=  w_T_ee_SIN (time);
  const MatrixHomogeneous &  w_T_obj=  w_T_obj_SIN (time);
  const ml::Matrix & Jq = articularJacobianSIN(time);
  const double & vectdes = positionRefSIN(time);
  //copy positions
  for( int i=0;i<3;++i )
  {
	  Soutput->setInputValue(i,w_T_ee.elementAt( i,3 ));
	  Soutput->setInputValue(i+6, w_T_obj.elementAt( i,3 ));
  }
  //copy rotations
  Soutput->setInputValue(3,mlHom2KDLRot(w_T_ee));
  Soutput->setInputValue(9,mlHom2KDLRot(w_T_obj));
  //copy reference
  Soutput->setInputValue(12,vectdes);


  //evaluate once to update the tree
  Soutput->value();

  //resize the matrices
  J.resize(1,Jq.nbCols());
  ml::Matrix Jtask(1,6);


  //compute the interaction matrix

  for (int i=0;i<6;++i)
	  Jtask(0,i)=Soutput->derivative(i);

  //multiplication!
  J=Jtask*Jq;

//return result
  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
FeatureExpressionGraph::computeError( ml::Vector& res,int time )
{
  sotDEBUGIN(15);


  const MatrixHomogeneous &  w_T_ee=  w_T_ee_SIN (time);
  const MatrixHomogeneous &  w_T_obj=  w_T_obj_SIN (time);
  const double & val_desired = positionRefSIN(time);

  sotDEBUG(15) << "w_T_obj = " << w_T_ee << std::endl;
  sotDEBUG(15) << "w_T_obj = " << w_T_obj << std::endl;
  sotDEBUG(15) << "vd = " << val_desired << std::endl;



  //copy positions
  for( int i=0;i<3;++i )
  {
	  Soutput->setInputValue(i,w_T_ee.elementAt( i,3 ));
	  Soutput->setInputValue(i+6, w_T_obj.elementAt( i,3 ));
  }
  //copy rotations
  Soutput->setInputValue(3,mlHom2KDLRot(w_T_ee));
  Soutput->setInputValue(9,mlHom2KDLRot(w_T_obj));
  //copy reference
  Soutput->setInputValue(12,val_desired);


  res.resize(1);
  //evaluate the result.
  res(0)=Soutput->value();


  sotDEBUGOUT(15);
  return res ;
}

void FeatureExpressionGraph::
display( std::ostream& os ) const
{
  os <<"FeatureExpressionGraph <"<<name<<">";
}



void FeatureExpressionGraph::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeatureExpressionGraph: "<<endl;
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
