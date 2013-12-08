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
, p1_SIN( NULL,"FeaturePointToPoint("+name+")::input(vector)::p1" )
, p2_SIN( NULL,"FeaturePointToPoint("+name+")::input(vector)::p2" )
, referenceVector_SIN( NULL,"FeaturePointToPoint("+name+")::input(vector)::referenceVector" )
{
  //the Jacobian depends by
  jacobianSOUT.addDependency( p1_SIN );
  jacobianSOUT.addDependency( p2_SIN );
  jacobianSOUT.addDependency( referenceVector_SIN );

  //the output depends by
  errorSOUT.addDependency( p1_SIN );
  errorSOUT.addDependency( p2_SIN );
  errorSOUT.addDependency( referenceVector_SIN );

  signalRegistration( p1_SIN << p2_SIN << referenceVector_SIN);

  Expression<KDL::Vector>::Ptr p1 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX),
		  input(EXP_GRAPH_BASE_INDEX+1),
		  input(EXP_GRAPH_BASE_INDEX+2));
  Expression<KDL::Vector>::Ptr p2 = KDL::vector(
		  input(EXP_GRAPH_BASE_INDEX+3),
		  input(EXP_GRAPH_BASE_INDEX+4),
		  input(EXP_GRAPH_BASE_INDEX+5));

	Expression<KDL::Vector>::Ptr refVector = KDL::vector(
			input(EXP_GRAPH_BASE_INDEX+6),
			input(EXP_GRAPH_BASE_INDEX+7),
			input(EXP_GRAPH_BASE_INDEX+8));

  /*here goes the expressions!!!*/
  geometric_primitive::Point point1;  point1.o=w_T_o1;  point1.p= p1;
  geometric_primitive::Point point2;  point2.o=w_T_o2;  point2.p= p2;

  Soutput_=geometric_primitive::point_point_difference(point1,point2, refVector);

  //end init
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeaturePointToPoint::
getDimension( unsigned int & dim, int time)
{
  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  for( int i=0;i<3;++i ) if( fl(i) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */

void FeaturePointToPoint::updateInputValues(KDL::Expression<KDL::Vector>::Ptr Soutput, int time)
{
  //read signals!
  const MatrixHomogeneous &  w_Tm_o1=  w_T_o1_SIN (time);
  const MatrixHomogeneous &  w_Tm_o2=  w_T_o2_SIN (time);
  const double & vectdes = referenceSIN(time);
  //copy positions
  for( int i=0;i<3;++i )
  {
    Soutput->setInputValue(i,w_Tm_o1.elementAt( i,3 ));
    Soutput->setInputValue(i+6, w_Tm_o2.elementAt( i,3 ));
  }
  //copy rotations
  //TODO use variable type for not controllable objects
  Soutput->setInputValue(3,mlHom2KDLRot(w_Tm_o1));
  Soutput->setInputValue(9,mlHom2KDLRot(w_Tm_o2));
  //copy reference
  Soutput->setInputValue(12,vectdes);


  readPositionVector(p1_SIN,EXP_GRAPH_BASE_INDEX,	 Soutput,time);
  readPositionVector(p2_SIN,EXP_GRAPH_BASE_INDEX+3,Soutput,time);
  readPositionVector(referenceVector_SIN,EXP_GRAPH_BASE_INDEX+6,Soutput,time);
}


void FeaturePointToPoint::
evaluateJacobian( ml::Matrix& res, KDL::Expression<KDL::Vector>::Ptr Soutput, int time )
{
  const MatrixHomogeneous &  w_Tm_o1=  w_T_o1_SIN (time);
  const MatrixHomogeneous &  w_Tm_o2=  w_T_o2_SIN (time);
	const Flags & fl = selectionSIN(time);

  if(w_J_o1_SIN.isPlugged())
  {
    const ml::Matrix & w_J_o1 = w_J_o1_SIN(time);

    ml::Matrix Jtask1(dimensionSOUT(time),6);
    ml::Matrix ad1(6,6);

    //compute the interaction matrices, that are expressed one in o1 applied
    unsigned index = 0;
    for (unsigned j=0;j<3;++j)
    {
      if(fl(j))
      {
        for (unsigned i=0;i<6;++i)
        {
          Jtask1(index,i)=Soutput->derivative(i)[j];
        }
        ++index;
      }
    }

    //multiplication!
    /*
     * we need the inverse of the adjoint matrix that brings
     * from o1(o2) frame to w
     * this is equal to the adjoint from w to o1(o2)
     * the matrix is then composed by
     * w_R_o1| 0
     * ------+-----
     *   0   |w_R_o1
     *    */
    ad1.setZero();//inverse of ad

    for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
        ad1(i,j) = ad1(i+3,j+3) = w_Tm_o1(i,j);

    res = Jtask1*ad1*w_J_o1;
  }
  if(w_J_o2_SIN.isPlugged())
  {
    const ml::Matrix & w_J_o2 = w_J_o2_SIN(time);
    ml::Matrix Jtask2(dimensionSOUT(time),6);
    ml::Matrix ad2(6,6);
    ad2.setZero();

    //compute the interaction matrices, that are expressed one in o1 applied
    unsigned index2 = 0;
    for (int j=0;j<3;++j)
    {
      if(fl(index2))
      {
        for (int i=0;i<6;++i)
          Jtask2(index2,i)=Soutput->derivative(i+6)[j];
        ++index2;
      }
    }

    for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
        ad2(i,j) = ad2(i+3,j+3) = w_Tm_o2(i,j);

    if(w_J_o1_SIN.isPlugged())
      res += Jtask2*ad2*w_J_o2;
    else
      res = Jtask2*ad2*w_J_o2;
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
  const Flags & selec = selectionSIN(time);

  sotDEBUGIN(15);
  updateInputValues(Soutput_, time);

  res= convert(Soutput_->value(),selec, dimensionSOUT(time));
  sotDEBUGOUT(15);
  return res ;
}
