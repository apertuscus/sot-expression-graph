#ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
#define __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__

#include <dynamic-graph/signal-ptr.h>
#include <sot/core/flags.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <kdl/expressiontree.hpp>

KDL::Rotation mlHom2KDLRot (const dynamicgraph::sot::MatrixHomogeneous &  M);
ml::Vector convert (const KDL::Vector &  v);
ml::Vector convert (const KDL::Vector & v, const dynamicgraph::sot::Flags & fl, unsigned int dim);


template<class T>
bool readHomogeneousMatrix(
    dynamicgraph::SignalPtr< dynamicgraph::sot::MatrixHomogeneous,int > & SIN,
    unsigned int base,
    const T & exp,
    const int time);

/*
 * readPositionVector:
 * given:
 * \param[in] 	SIN		the signal to be read (a vector of 3)
 * \param[in]	base	the base index that corresponds to the beginnini of the vector with the input value
 * \param[in]	exp		the expression in which the values should be copied
 * \param[in]	time	the time seed
 *
 * as a side effect it setInputValue of the given values, and
 *
 * \return	true if the signal SIN is plugged, false otherwise
 * if it is not plugged the vector is set to zero
 * TODO in future, test new data to avoid setInputValue on constant data.
 *  */
template<class T>
bool readPositionVector(
    dynamicgraph::SignalPtr< ml::Vector,int >& SIN,
    unsigned int base,
    const T & exp,
    const int time);

namespace geometric_primitive
{
using namespace KDL;


struct Point
{
	//! \brief origin of the point
	Expression<KDL::Frame>::Ptr o;
	//! \brief position of the point, in the frame o
	Expression<KDL::Vector>::Ptr p;
};

//no check done on versor in building up
struct Versor
{
	//! \brief frame
	Expression<Frame>::Ptr o;
	//! \brief versor (aka normed vector), expressed in the frame o
	Expression<Vector>::Ptr v;
};
struct Line
{
	//! \brief origin of the point
	Expression<Frame>::Ptr o;
	//! \brief dir?
	Expression<Vector>::Ptr dir;
	//! \brief pos?
	Expression<Vector>::Ptr p;
};

struct Plane
{
	Expression<Frame>::Ptr o;
	Expression<Vector>::Ptr normal;
	Expression<Vector>::Ptr p;
};

/* for all the expressions,
 *  we suppose that the frames are expressed
 *  in a common reference
 *  */
Expression<double>::Ptr point_point_distance(const Point & p1,const Point& p2);
Expression<KDL::Vector>::Ptr point_point_difference(const Point & p1, const Point & p2);
Expression<double>::Ptr line_point_distance(const Point&  p, const Line& l);
Expression<double>::Ptr projection_of_point_on_line(const Point& p, const Line& l);
Expression<double>::Ptr line_line_distance (const Line& l1, const Line& l2);
Expression<double>::Ptr surface_point_distance(const Point & p, const Plane & plan);


//angles

Expression<double>::Ptr angle_btw_planes (const Plane & pl1, const Plane & pl2);
Expression<double>::Ptr angle_btw_versors (const Versor & v1, const Versor& v2);
Expression<double>::Ptr angle_btw_plane_and_versor (const Plane & p, const Versor& v);
/*This function computes the angle btw
 * the direction of the line and the versor that goes from line1.p to point2.p*/
Expression<double>::Ptr angle_btw_line_and_point (const Line & line1, const Point& point2);

//helper functions
Expression<Vector>::Ptr ExpressInBase(const Point & p);
Expression<Vector>::Ptr ExpressInBase(const Versor & v);
Expression<Vector>::Ptr ExpressOriginInBase(const Plane & p);
Expression<Vector>::Ptr ExpressOriginInBase(const Line & l);
Expression<Vector>::Ptr ExpressDirectionInBase(const Plane & p);
Expression<Vector>::Ptr ExpressDirectionInBase(const Line & l);
}

# include "helper.t.cpp"

#endif // __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__


