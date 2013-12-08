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
#include "feature-angle-btw-plane-versor.h"
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

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureAngleBtwPlaneAndVersor,"FeatureAngleBtwPlaneAndVersor");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureAngleBtwPlaneAndVersor::
FeatureAngleBtwPlaneAndVersor( const string& name )
: FeatureExprGraphAbstract( name )
, p1_SIN( NULL,"FeatureAnglesBtwPlanes("+name+")::input(vector)::p1" )
, norm1_SIN( NULL,"FeatureAnglesBtwPlanes("+name+")::input(vector)::norm1" )
, versor2_SIN( NULL,"FeatureAnglesBtwPlanes("+name+")::input(vector)::v2" )
{
  //the Jacobian depends by
  jacobianSOUT.addDependency( p1_SIN );
  jacobianSOUT.addDependency( norm1_SIN );
  jacobianSOUT.addDependency( versor2_SIN );

  //the output depends by
  errorSOUT.addDependency( p1_SIN );
  errorSOUT.addDependency( norm1_SIN );
  errorSOUT.addDependency( versor2_SIN );

  signalRegistration( p1_SIN  <<norm1_SIN<<versor2_SIN);

  Expression<KDL::Vector>::Ptr p1 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX),
		  input(EXP_GRAPH_BASE_INDEX+1),
		  input(EXP_GRAPH_BASE_INDEX+2));
  Expression<KDL::Vector>::Ptr norm1 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX+3),
		  input(EXP_GRAPH_BASE_INDEX+4),
		  input(EXP_GRAPH_BASE_INDEX+5));
  Expression<KDL::Vector>::Ptr versor2 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX+6),
		  input(EXP_GRAPH_BASE_INDEX+7),
		  input(EXP_GRAPH_BASE_INDEX+8));


  /*here goes the expressions!!!*/
  geometric_primitive::Plane plane ;
  plane.o=w_T_o1;plane.p=p1;plane.normal=norm1;

  geometric_primitive::Versor versor ;
  versor.o=w_T_o2;versor.v=versor2;
  Soutput_=Sreference-geometric_primitive::angle_btw_plane_and_versor(plane,versor);

  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureAngleBtwPlaneAndVersor::
getDimension( unsigned int & dim, int /*time*/ )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=1;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */

void FeatureAngleBtwPlaneAndVersor::updateInputValues(KDL::Expression<double>::Ptr Soutput, int time)
{
  FeatureExprGraphAbstract::updateInputValues(Soutput, time);
  //same order that in the constructor!
  readPositionVector(p1_SIN, EXP_GRAPH_BASE_INDEX, Soutput,time);
  FeatureExprGraphAbstract::readVersorVector  (norm1_SIN,EXP_GRAPH_BASE_INDEX+3,	Soutput,time);
  FeatureExprGraphAbstract::readVersorVector  (versor2_SIN,EXP_GRAPH_BASE_INDEX+6,	Soutput,time);
}


ml::Matrix& FeatureAngleBtwPlaneAndVersor::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;
  updateInputValues(Soutput_, time);

  //evaluate once to update the tree
  Soutput_->value();

  //resize the matrices
  evaluateJacobian(J, Soutput_, time);

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
FeatureAngleBtwPlaneAndVersor::computeError( ml::Vector& res,int time )
{
  sotDEBUGIN(15);
  updateInputValues(Soutput_, time);

  res.resize(1);
  //evaluate the result.
  res(0) = Soutput_->value();

  sotDEBUGOUT(15);
  return res ;
}
