/*
 * Copyright 2010,
 * ,
 * ,
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

#ifndef __SOT_FEATURE_EXPRESSION_GRAPH_ABSTRACT_HH__
#define __SOT_FEATURE_EXPRESSION_GRAPH_ABSTRACT_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>


#include <kdl/expressiontree.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \classFeatureExprGraphAbstract
  \brief Class that defines example of expression graps
*/
class FeatureExprGraphAbstract
: public FeatureAbstract
{

  DECLARE_NO_REFERENCE;

protected:
  /*variables for expression graphs */
  KDL::Expression<KDL::Frame>::Ptr w_T_o1;
  KDL::Expression<KDL::Frame>::Ptr w_T_o2;
  KDL::Expression<double>::Ptr Sreference;
  //end

  /* --- SIGNALS ------------------------------------------------------------ */
public:
 // dg::SignalPtr< ml::Vector,int > vectorSIN;//what is that for?

  //pose o1
  dg::SignalPtr< MatrixHomogeneous,int > w_T_o1_SIN;
  //robot jacobian w_J_o1
  dg::SignalPtr< ml::Matrix,int > w_J_o1_SIN;
  //pose of the object
  dg::SignalPtr< MatrixHomogeneous,int > w_T_o2_SIN;
  //robot jacobian w_J_o1
  dg::SignalPtr< ml::Matrix,int > w_J_o2_SIN;

  dg::SignalPtr< double,int > referenceSIN;

 public:
  FeatureExprGraphAbstract( const std::string& name );
  virtual ~FeatureExprGraphAbstract( void ) {}

 protected:
  //TODO: template?
  void updateInputValues(KDL::Expression<double>::Ptr, int time);

  void evaluateJacobian(ml::Matrix& res, KDL::Expression<double>::Ptr, int time );
} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_EXPRESSION_GRAPH_ABSTRACT_HH__

