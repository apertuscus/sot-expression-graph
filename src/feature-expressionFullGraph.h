
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

#ifndef __SOT_FEATURE_EXPRESSIONFULLGRAPH_HH__
#define __SOT_FEATURE_EXPRESSIONFULLGRAPH_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-homogeneous.hh>


#include <kdl/expressiontree.hpp>
#include <expressiongraph_tf/context.hpp>
#include <expressiongraph_tf/urdfexpressions2.hpp>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_vector3_EXPORTS)
#    define SOTfeatureExpressionFullGraph_EXPORT __declspec(dllexport)
#  else
#    define SOTfeatureExpressionFullGraph_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTfeatureExpressionFullGraph_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class featureExpressionFullGraph
  \brief Class that defines example of expression graps
 */
class SOTfeatureExpressionFullGraph_EXPORT FeatureExpressionFullGraph
: public FeatureAbstract
  {
public:
	static const std::string CLASS_NAME;
	virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	DECLARE_NO_REFERENCE;

	/*variables for expression graphs */
	KDL::Expression<KDL::Frame>::Ptr w_T_ee;
	KDL::Expression<KDL::Frame>::Ptr w_T_obj;
	KDL::Expression<double>::Ptr Soutput;
	KDL::Expression<double>::Ptr Sreference;

	std::map<int,int> index_to_sot;
	unsigned int st_ind_ex;
	std::vector<double> q_ex;
	KDL::Context::Ptr ctx_;
	unsigned dimension_;     // dimension
	//end

protected:

	/* --- SIGNALS ------------------------------------------------------------ */
public:
	// joint angles
	dg::SignalPtr< ml::Vector,int > jointSIN;

	//pose of the object
	dg::SignalPtr< MatrixHomogeneous,int > w_T_obj_SIN;

	// ...
	dg::SignalPtr< double,int > positionRefSIN;
	dg::SignalTimeDependent<int,int>  oneStepOfControlSignal;

	using FeatureAbstract::selectionSIN;
	using FeatureAbstract::jacobianSOUT;
	using FeatureAbstract::errorSOUT;

	void setChain
	(const std::string & op1, const std::string & op2, const std::string & label);

	KDL::ExpressionMap create_fk_from_urdf(
			KDL::Context::Ptr ctx , const std::string & op1,
			const std::string & op2, const std::string & label);

	std::map<int,int> index_lookup_table
			(const KDL::Context::Ptr ctx,
			 const std::string name_joints[],const int n_of_joints);

public:
	FeatureExpressionFullGraph( const std::string& name );
	virtual ~FeatureExpressionFullGraph( void ) {}

	virtual unsigned int& getDimension( unsigned int & dim, int time );

	virtual ml::Vector& computeError( ml::Vector& res,int time );
	virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

	std::string getDocString () const;

	void initCommands();

private:
	int & oneStepOfControl(int & dummy, int time);
} ;

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_FEATURE_EXPRESSIONFULLGRAPH_HH__
