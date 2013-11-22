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
#include "feature-point-to-point.h"
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

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePointToPoint,"FeaturePointToPoint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeaturePointToPoint::
FeaturePointToPoint( const string& name )
: FeatureExprGraphAbstract( name )
, p1_SIN( NULL,"sotFeaturePoint6d("+name+")::input(vector)::p1" )
, p2_SIN( NULL,"sotFeaturePoint6d("+name+")::input(vector)::p2" )
{
  //the Jacobian depends by
  jacobianSOUT.addDependency( p1_SIN );
  jacobianSOUT.addDependency( p2_SIN );

  //the output depends by
  errorSOUT.addDependency( p1_SIN );
  errorSOUT.addDependency( p2_SIN );

  signalRegistration( p1_SIN << p2_SIN );

  Expression<KDL::Vector>::Ptr p1 = KDL::vector(input(13), input(14),input(15));
  Expression<KDL::Vector>::Ptr p2 = KDL::vector(input(16), input(17),input(18));

  /*here goes te expressions!!!*/
  geometric_primitive::Point point1 = {w_T_o1, p1};
  geometric_primitive::Point point2 = {w_T_o2, p2};
  Soutput_=Sreference-geometric_primitive::point_point_distance(point1,point2);

  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeaturePointToPoint::
getDimension( unsigned int & dim, int /*time*/ )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=1;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */

void FeaturePointToPoint::updateInputValues(KDL::Expression<double>::Ptr Soutput, int time)
{
  FeatureExprGraphAbstract::updateInputValues(Soutput, time);

  if(p1_SIN.isPluged())
  {
    const ml::Vector & p1 = p1_SIN(time);
    for( int i=0;i<3;++i )
      Soutput_->setInputValue(13+i, p1(i));
  }
  else
  {
    for( int i=0;i<3;++i )
      Soutput_->setInputValue(13+i, 0);
  }


  if(p2_SIN.isPluged())
  {
    const ml::Vector & p2 = p2_SIN(time);
    for( int i=0;i<3;++i )
      Soutput_->setInputValue(16+i, p2(i));
  }
  else
  {
    for( int i=0;i<3;++i )
      Soutput->setInputValue(16+i, 0);
  }
}


ml::Matrix& FeaturePointToPoint::
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
FeaturePointToPoint::computeError( ml::Vector& res,int time )
{
  sotDEBUGIN(15);
  updateInputValues(Soutput_, time);

  res.resize(1);
  //evaluate the result.
  res(0) = Soutput_->value();

  sotDEBUGOUT(15);
  return res ;
}
