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
#include "feature-point-to-surface.h"
#include "helper.h"

#include <sot/core/exception-feature.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace KDL;
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePointToSurface,"FeaturePointToSurface");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeaturePointToSurface::
FeaturePointToSurface( const string& ExpressionGraph )
: FeatureAbstract( ExpressionGraph )
, jacobian_ee_SIN ( NULL,"FeaturePointToSurface("+name+")::input(matrix)::J_ee" )
, w_T_ee_SIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::frame_ee" )
, position_ee_SIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::position_ee" )

, jacobian_obj_SIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::J_obj" )
, w_T_obj_SIN( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::frame_obj" )
, position_obj_SIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::position_obj" )
, normalSIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::normal" )
{
  //the jacobian depends by
//  jacobianSOUT.addDependency( w_T_ee_SIN );
//  jacobianSOUT.addDependency( w_T_obj_SIN );
  jacobianSOUT.addDependency( jacobian_ee_SIN );//this one is not probably necessary...
  jacobianSOUT.addDependency( jacobian_obj_SIN );

  //the output depends by
//  jacobianSOUT.addDependency( w_T_ee_SIN );
//  jacobianSOUT.addDependency( w_T_obj_SIN );
//  errorSOUT.addDependency( positionRefSIN );

  signalRegistration(jacobian_ee_SIN << w_T_ee_SIN << position_ee_SIN);
  signalRegistration(jacobian_obj_SIN << w_T_obj_SIN << position_obj_SIN << normalSIN);

  // init of expression graph
  //frame of the robot ee, w.r.t a world frame
  Expression<KDL::Vector>::Ptr w_p_ee = KDL::vector(input(0), input(1),input(2));
  Expression<KDL::Rotation>::Ptr w_R_ee = inputRot(3);
  w_T_ee_ = frame(w_R_ee,  w_p_ee);
  Expression<KDL::Vector>::Ptr pos_ee = KDL::vector(input(7), input(8),input(9));

  //frame of the the object, w.r.t the same world frame
  Expression<KDL::Vector>::Ptr w_p_obj =KDL::vector(input(10), input(11),input(12));
  Expression<KDL::Rotation>::Ptr w_R_obj= inputRot(13);
  w_T_obj_ = frame(w_R_obj,  w_p_obj);
  Expression<KDL::Vector>::Ptr normal = KDL::vector(input(17), input(18),input(19));
  Expression<KDL::Vector>::Ptr pos_obj = KDL::vector(input(20), input(21),input(22));

  geometric_primitive::Point pt   = {w_T_ee_, pos_ee}; /// TODO
  geometric_primitive::Plane surf = {w_T_obj_, normal, pos_obj}; /// TODO

  Soutput_ = geometric_primitive::surface_point_distance(pt, surf);
  //declare dependecies
  //copy positions


  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeaturePointToSurface::
getDimension( unsigned int & dim, int /*time*/ )
{
  dim = 1;
  return dim;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
int & FeaturePointToSurface::
oneStepOfControl( int& dummy,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  //read signals!
  const MatrixHomogeneous &  w_T_ee_Sig=  w_T_ee_SIN (time);
  const MatrixHomogeneous &  w_T_obj_Sig=  w_T_obj_SIN (time);
  const ml::Matrix & Jq_ee = jacobian_ee_SIN(time);
  const ml::Vector & normal  = normalSIN(time);
  const ml::Vector & pos_ee  = position_ee_SIN(time);
  const ml::Vector & pos_obj = position_obj_SIN(time);

  //copy positions
  for( int i=0;i<3;++i )
    Soutput_->setInputValue(i,w_T_ee_Sig.elementAt( i,3 ));
  Soutput_->setInputValue(3,mlHom2KDLRot(w_T_ee_Sig));

  for( int i=0;i<3;++i )
    Soutput_->setInputValue(7+i,pos_ee(i));

  for( int i=0;i<3;++i )
    Soutput_->setInputValue(10+i, w_T_obj_Sig.elementAt( i,3 ));
  Soutput_->setInputValue(13,mlHom2KDLRot(w_T_obj_Sig));

  //copy normal
  for( int i=0;i<3;++i )
    Soutput_->setInputValue(17+i,normal(i));

  for( int i=0;i<3;++i )
    Soutput_->setInputValue(20+i,pos_obj(i));


  //evaluate once to update the tree
  error_.resize(1);
  error_(0) = Soutput_->value();

  //compute the interaction matrix
  ml::Matrix Jtask(1,6);
  for (int i=0;i<6;++i)
    Jtask(0,i)=Soutput_->derivative(i);

  //multiplication!
  if (!jacobian_obj_SIN.isPluged())
    jacobian_ = Jtask * Jq_ee;
  else
  {
    const ml::Matrix & Jq_obj = jacobian_obj_SIN(time);
    jacobian_ = Jtask * (Jq_ee - Jq_obj);
  }

  // debug
  {
    std::cout << "normal " << normal << std::endl;
    geometric_primitive::Point pt   = {w_T_ee_,  KDL::Constant(KDL::Vector(pos_ee(0),pos_ee(1),pos_ee(2)))}; /// TODO
    geometric_primitive::Plane surf = {w_T_obj_,
                                       KDL::Constant(KDL::Vector(normal(0),normal(1),normal(2))),
                                       KDL::Constant(KDL::Vector(pos_obj(0),pos_obj(1),pos_obj(2)))}; /// TODO

    geometric_primitive::surface_point_distance(pt, surf);
  }

  dummy = 1;
  return dummy;
}


ml::Matrix& FeaturePointToSurface::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;
  int dummy = 0;
  oneStepOfControl(dummy, time);

  J = jacobian_;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
FeaturePointToSurface::computeError( ml::Vector& res,int time )
{
  sotDEBUGIN(15);
  int dummy = 0;
  oneStepOfControl(dummy, time);

  res = error_;

  sotDEBUGOUT(15);
  return res ;
}
