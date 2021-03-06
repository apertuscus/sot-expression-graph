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

#ifndef __SOT_FEATURE_POINT_TOWARD_HH__
#define __SOT_FEATURE_POINT_TOWARD_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include "feature-expr-graph-abstract.h"
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>


#include <kdl/expressiontree.hpp>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_vector3_EXPORTS)
#    define SOTFeaturePointToward_EXPORT __declspec(dllexport)
#  else
#    define SOTFeaturePointToward_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFeaturePointToward_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeaturePointToward
  \brief Class that defines example of expression graps
*/
class FeaturePointToward
: public FeatureExprGraphAbstract
{
public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  DECLARE_NO_REFERENCE;

public:
  FeaturePointToward( const std::string& name );
  virtual ~FeaturePointToward( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

public:
  // versor  1 with respect to the frame o1
  dg::SignalPtr< ml::Vector,int > v1_SIN;
  // point  1 with respect to the frame o1 (origin to which i will align)
  dg::SignalPtr< ml::Vector,int > p1_SIN;
  // point 2 with respect to the frame o2 (target to which i will align)
  dg::SignalPtr< ml::Vector,int > p2_SIN;

  // reference (cos of angle)
  dg::SignalPtr< ml::Vector,int > referenceSIN;

private:
  virtual void updateInputValues(KDL::Expression<double>::Ptr Soutput, int time);

private:
  KDL::Expression<double>::Ptr Soutput_;
  ml::Matrix Jprev_;

} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_POINT_TO_POINT_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
