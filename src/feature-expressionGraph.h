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

#ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HH__
#define __SOT_FEATURE_EXPRESSIONGRAPH_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>


#include <kdl/expressiontree.hpp>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_vector3_EXPORTS)
#    define SOTFeatureExpressionGraph_EXPORT __declspec(dllexport)
#  else
#    define SOTFeatureExpressionGraph_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFeatureExpressionGraph_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
using namespace KDL;
namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureExpressionGraph
  \brief Class that defines example of expression graps
*/
class SOTFeatureExpressionGraph_EXPORT FeatureExpressionGraph
: public FeatureAbstract
{


 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  DECLARE_NO_REFERENCE;

  /*variables for expression graphs */
  Expression<KDL::Frame>::Ptr w_T_o1;
  Expression<KDL::Frame>::Ptr w_T_o2;
  Expression<double>::Ptr Soutput;
  Expression<double>::Ptr Sreference;
  //end

 protected:

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



  dg::SignalPtr< double,int > positionRefSIN;

  using FeatureAbstract::selectionSIN;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;

 public:
  FeatureExpressionGraph( const std::string& name );
  virtual ~FeatureExpressionGraph( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

  virtual void display( std::ostream& os ) const;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
