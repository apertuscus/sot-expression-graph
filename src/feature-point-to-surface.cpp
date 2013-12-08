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
: FeatureExprGraphAbstract( ExpressionGraph )
, p1_SIN( NULL,"sotFeaturePoint6d("+name+")::input(vector)::p1" )
, p2_SIN( NULL,"sotFeaturePoint6d("+name+")::input(vector)::p2" )
, normalSIN ( NULL,"FeaturePointToSurface("+name+")::input(matrixHomo)::normal_o2" )
, referenceSIN( NULL,"FeatureAnglesBtwPlanes("+name+")::input(double)::reference" )
{
  //the jacobian depends by
  jacobianSOUT.addDependency( p1_SIN );
  jacobianSOUT.addDependency( p2_SIN );

  //the output depends by
  errorSOUT.addDependency( p1_SIN );
  errorSOUT.addDependency( p2_SIN );

  signalRegistration(p1_SIN);
  signalRegistration(p2_SIN << normalSIN);

  // init of expression graph
  //frame of the robot ee, w.r.t a world frame
  Expression<KDL::Vector>::Ptr p1 = KDL::vector(input(13), input(14),input(15));
  Expression<KDL::Vector>::Ptr p2 = KDL::vector(input(16), input(17),input(18));
  Expression<KDL::Vector>::Ptr normal = KDL::vector(input(19), input(20),input(21));

  geometric_primitive::Point pt   = {w_T_o1, p1}; /// TODO
  geometric_primitive::Plane surf = {w_T_o2, normal, p2}; /// TODO

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
  updateInputValues(Soutput_, time);

  //read signals!
  const ml::Vector & p_o1 = p1_SIN(time);
  const ml::Vector & p_o2 = p2_SIN(time);
  const ml::Vector & normal  = normalSIN(time);

  // copty that into input
  for( int i=0;i<3;++i )
  {
    Soutput_->setInputValue(13+i, p_o1(i));
    Soutput_->setInputValue(16+i, p_o2(i));
    Soutput_->setInputValue(19+i, normal(i));
  }

  //evaluate once to update the tree
  error_.resize(1);
  error_(0) = Soutput_->value();

  evaluateJacobian(jacobian_, Soutput_, time);
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
  double reference = referenceSIN(time);

  int dummy = 0;
  oneStepOfControl(dummy, time);

  res = error_ - reference;

  sotDEBUGOUT(15);
  return res ;
}
