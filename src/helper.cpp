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


using namespace KDL;
/*for all the expressions we soppose that the frames are expressed in a common reference*/
Expression<double>::Ptr point_point_distance(const Point & p1,const Point &  p2)
{
	Expression<Vector>::Ptr v1=p1.o*p1.p;
	Expression<Vector>::Ptr v2=p2.o*p2.p;
	return	norm(v1-v2);
}
Expression<double>::Ptr line_point_distance(const Point &  p, const Line & l)
{
	Expression<Vector>::Ptr a=l.o*l.p - p.o*p.p; //vector from l.p to p.p (in common frame)
	Expression<Vector>::Ptr l_dir=rotation(l.o)*l.dir; //direction vector in common frame
	//check that l_dir is still a versor...
	l_dir=l_dir*(Constant<double>(1)/norm(l_dir));
	return	abs(dot(a,l_dir));
}
Expression<double>::Ptr projection_of_point_on_line(const Point &  p, const Line & l)
{
	Expression<Vector>::Ptr a=l.o*l.p - p.o*p.p; //vector from l.p to p.p (in common frame)
		Expression<Vector>::Ptr l_dir=l.o*l.dir; //direction vector in common frame
		//check that l_dir is still a versor...
		l_dir=l_dir*(Constant<double>(1)/norm(l_dir));
	Expression<double>::Ptr f=dot(a,l_dir);//feature coordinate of the point of nearest distance
	Expression<Vector>::Ptr pf=l.o*(l.dir*f);//point of nearest distance expressed in world
	return norm(pf-l.o*l.p);
}
Expression<double>::Ptr distance_from_lines (const Line &  l1, const Line & l2)
{
	/*bring all in common coordinates
	 * 	using the notation of
	 * 	http://mathworld.wolfram.com/Line-LineDistance.html
	 * NB THIS IS OK ONLY FOR SKEW (non parallel) LINES
	 *
	 * */
	Expression<Vector>::Ptr a=l1.o*l1.dir;
	Expression<Vector>::Ptr b=l2.o*l2.dir;
	Expression<Vector>::Ptr c=l2.o*l2.p-l1.o*l1.p;
	Expression<double>::Ptr num=abs(dot(c,cross(a,b)));
	Expression<double>::Ptr den=norm(cross(a,b));
	return num/den;

}



Expression<double>::Ptr surface_point_distance(const Point & pt, const Plane & plan)
{
  Expression<Vector>::Ptr oa = (plan.o * plan.p) - (pt.o * pt.p);
  Expression<Vector>::Ptr planNormal = plan.o * plan.normal;

  return  dot(oa, planNormal);
}

}
