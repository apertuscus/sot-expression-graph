#include "helper.h"
using namespace KDL;
KDL::Rotation mlHom2KDLRot (const dynamicgraph::sot::MatrixHomogeneous &  M)
{
	KDL::Rotation Rkdl;
	for( int i=0;i<3;++i )
		for( int j=0;j<3;++j )
			Rkdl( i,j ) = M.elementAt( i,j );
	//M.extract(R);
	//R(i,j);
	return Rkdl;
}
/*
 * class representing the geometricprimitve
 * */
namespace geometric_primitive
{

Expression<Vector>::Ptr ExpressInBase(const Point & p)
{
	return	p.o*p.p;
}
Expression<Vector>::Ptr ExpressInBase(const Versor & v)
{
	return rotation(v.o)*v.v;
}
Expression<Vector>::Ptr ExpressOriginInBase(const Plane & p)
{
	return p.o*p.p;
}
Expression<Vector>::Ptr ExpressOriginInBase(const Line & l)
{
	return l.o*l.p;
}
Expression<Vector>::Ptr ExpressDirectionInBase(const Plane & p)
{
	return rotation(p.o)*p.normal;
}
Expression<Vector>::Ptr ExpressDirectionInBase(const Line & l)
{
	return rotation(l.o)*l.dir;
}
using namespace KDL;
/*for all the expressions we soppose that the frames are expressed in a common reference*/
Expression<double>::Ptr point_point_distance(const Point & p1,const Point &  p2)
{
	Expression<Vector>::Ptr v1=ExpressInBase(p1);
	Expression<Vector>::Ptr v2=ExpressInBase(p2);
	return	norm(v1-v2);
	//return norm(p1.p - inv(p1.o)*p2.o*p2.p);
}
Expression<double>::Ptr line_point_distance(const Point &  p, const Line & l)
{
	Expression<Vector>::Ptr dist_vect=ExpressOriginInBase(l) - ExpressInBase(p); //vector from l.p to p.p (in common frame)
	Expression<Vector>::Ptr l_dir=ExpressDirectionInBase(l); //direction vector in common frame
	// TODO check that l_dir is still a versor...
	// TODO check if it works!
	//l_dir=l_dir*(Constant<double>(1.0)/norm(l_dir));
	Expression<double>::Ptr proj=dot(dist_vect,l_dir);
	//compute rejection
	Expression<Vector>::Ptr rj_vec=dist_vect-proj*l_dir;
	return	norm(rj_vec);
}
Expression<double>::Ptr projection_of_point_on_line(const Point &  p, const Line & l)
{
	Expression<Vector>::Ptr dist_vect= ExpressInBase(p)-ExpressOriginInBase(l) ; //vector from l.p to p.p (in common frame)
	Expression<Vector>::Ptr l_dir=ExpressDirectionInBase(l); //direction vector in common frame
	// TODO check that l_dir is still a versor...
	// TODO check if it works!
	//l_dir=l_dir*(Constant<double>(1.0)/norm(l_dir));
	Expression<double>::Ptr proj=dot(dist_vect,l_dir);
	return proj;
}
Expression<double>::Ptr line_line_distance (const Line &  l1, const Line & l2)
{
	/*bring all in common coordinates
	 * 	using the notation of
	 * 	http://mathworld.wolfram.com/Line-LineDistance.html
	 * NB THIS IS OK ONLY FOR SKEW (non parallel) LINES
	 *
	 * */
	Expression<Vector>::Ptr a=rotation(l1.o)*l1.dir;
	Expression<Vector>::Ptr b=rotation(l2.o)*l2.dir;
	Expression<Vector>::Ptr c=l2.o*l2.p-l1.o*l1.p;
	Expression<double>::Ptr num=abs(dot(c,cross(a,b)));
	Expression<double>::Ptr den=norm(cross(a,b));
	return num/den;

}


//done by Francois
Expression<double>::Ptr surface_point_distance(const Point & pt, const Plane & plan)
{
  Expression<Vector>::Ptr oa = (plan.o * plan.p) - (pt.o * pt.p);
  Expression<Vector>::Ptr planNormal = plan.o * plan.normal;
  return  dot(oa, planNormal);
}
Expression<double>::Ptr angle_btw_versors (const Versor & v1, const Versor& v2)
{
	Expression<Vector>::Ptr vrs1=ExpressInBase(v1);
	Expression<Vector>::Ptr vrs2=ExpressInBase(v2);
	return KDL::acos(dot(vrs1,vrs2));
}
Expression<double>::Ptr angles_btw_planes (const Plane & pl1, const Plane & pl2)
{
	Expression<Vector>::Ptr vrs1=ExpressDirectionInBase(pl1);
	Expression<Vector>::Ptr vrs2=ExpressDirectionInBase(pl2);
	return KDL::acos(dot(vrs1,vrs2));
}
}
