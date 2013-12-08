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
#include "feature-versor-to-versor-angle.h"
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

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureVersorToVersor,"FeatureVersorToVersor");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureVersorToVersor::
FeatureVersorToVersor( const string& name )
: FeatureExprGraphAbstract( name )
, v1_SIN( NULL,"FeatureVersorToVersor("+name+")::input(vector)::v1" )
, v2_SIN( NULL,"FeatureVersorToVersor("+name+")::input(vector)::v2" )
{
  //the Jacobian depends by
  jacobianSOUT.addDependency( v1_SIN );
  jacobianSOUT.addDependency( v2_SIN );

  //the output depends by
  errorSOUT.addDependency( v1_SIN );
  errorSOUT.addDependency( v2_SIN );

  signalRegistration( v1_SIN << v2_SIN );

  Expression<KDL::Vector>::Ptr p1 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX),
		  input(EXP_GRAPH_BASE_INDEX+1),
		  input(EXP_GRAPH_BASE_INDEX+2));
  Expression<KDL::Vector>::Ptr p2 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX+3),
		  input(EXP_GRAPH_BASE_INDEX+4),
		  input(EXP_GRAPH_BASE_INDEX+5));

  /*here goes the expressions!!!*/
  geometric_primitive::Versor versor1;  versor1.o=w_T_o1;  versor1.v= p1;
  geometric_primitive::Versor versor2;  versor2.o=w_T_o2;  versor2.v= p2;

  Soutput_=Sreference-geometric_primitive::angle_btw_versors(versor1,versor2);

  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureVersorToVersor::
getDimension( unsigned int & dim, int /*time*/ )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=1;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */

void FeatureVersorToVersor::updateInputValues(KDL::Expression<double>::Ptr Soutput, int time)
{
  FeatureExprGraphAbstract::updateInputValues(Soutput, time);

  readPositionVector(v1_SIN,EXP_GRAPH_BASE_INDEX,	 Soutput,time);
  readPositionVector(v2_SIN,EXP_GRAPH_BASE_INDEX+3,Soutput,time);
}


ml::Matrix& FeatureVersorToVersor::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;
  updateInputValues(Soutput_, time);

  //evaluate once to update the tree
  Soutput_->value();

  //resize the matrices
  evaluateJacobian(J, Soutput_, time);

  // WARNING: DIVERGE WHEN Error equals 0
  if (isnan(J(0,0)))
    J=Jprev_;
  else
    Jprev_=J;

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
FeatureVersorToVersor::computeError( ml::Vector& res,int time )
{
  sotDEBUGIN(15);
  updateInputValues(Soutput_, time);

  res.resize(1);
  //evaluate the result.
  res(0) = Soutput_->value();
  if (isnan(res(0)))
    res(0)=0;

  sotDEBUGOUT(15);
  return res ;
}
