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

#ifndef __SOT_FEATURE_POINT_TO_SURFACE_HH__
#define __SOT_FEATURE_POINT_TO_SURFACE_HH__

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
  \class FeaturePointToSurface
  \brief
*/
class FeaturePointToSurface
: public FeatureAbstract
{

public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  DECLARE_NO_REFERENCE;

public:
 FeaturePointToSurface( const std::string& name );
 virtual ~FeaturePointToSurface( void ) {}

 virtual unsigned int& getDimension( unsigned int & dim, int time );

 virtual ml::Vector& computeError( ml::Vector& res,int time );
 virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

private:
 // triggers one step of computation, and saved the computated data.
 // used to avoid useless recomputation.
 int & oneStepOfControl(int & dummy, int time);

public:

  /*variables for expression graphs */
  KDL::Expression<KDL::Frame>::Ptr w_T_ee_;
  KDL::Expression<KDL::Frame>::Ptr w_T_obj_;

  KDL::Expression<double>::Ptr Soutput_;
  //end

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  //jacobian of the robot end effector
  dg::SignalPtr< ml::Matrix,int > jacobian_ee_SIN;

  //pose of the robot end effector
  dg::SignalPtr< MatrixHomogeneous,int > w_T_ee_SIN;

  dg::SignalPtr< ml::Vector,int > position_ee_SIN;



  //jacobian of the robot end effector
  dg::SignalPtr< ml::Matrix,int > jacobian_obj_SIN;

  //frame of the surface
  dg::SignalPtr< MatrixHomogeneous,int > w_T_obj_SIN;

  // a point belonging to the surface
  dg::SignalPtr< ml::Vector,int > position_obj_SIN;

  // normal of the surface (expressed in the frame of the surface)
  dg::SignalPtr< ml::Vector,int > normalSIN;


//  using FeatureAbstract::selectionSIN;
//  using FeatureAbstract::jacobianSOUT;
//  using FeatureAbstract::errorSOUT;


// internal data (to avoid memory allocation)
private:
  ml::Matrix jacobian_;
  ml::Vector error_;
} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
