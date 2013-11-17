#include "helper.h"
using namespace KDL;
KDL::Rotation mlHom2KDLRot (const MatrixHomogeneous &  M)
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
Expression<double>::Ptr point_point_distance(point p1,point p2)
{
	Expression<Vector>::Ptr v1=p1.o*p1.p;
	Expression<Vector>::Ptr v2=p2.o*p2.p;
	return	norm(v1-v2);
}
Expression<double>::Ptr line_point_distance(point p, line l)
{
	Expression<Vector>::Ptr a=l.o*l.p-p.o*p.p; //vector from l.p to p.p
	Expression<Vector>::Ptr b=l.o*l.dir; //direction vector in common frame
	//check that v2 is still a versor...
	b=b*(Constant<double>(1)/norm(b));
	return	abs(dot(a,b));
}
Expression<double>::Ptr projection_of_point_on_line(point p, line l)
{
	Expression<Vector>::Ptr a=l.o*l.p-p.o*p.p; //vector from l.p to p.p
	Expression<Vector>::Ptr b=l.o*l.dir; //direction vector in common frame
	//check that v2 is still a versor...
	b=b*(Constant<double>(1)/norm(b));
	Expression<double>::Ptr f=dot(a,b);//feature coordinate of the point of nearest distance
	Expression<Vector>::Ptr pf=l.o*(l.dir*f);//point of nearest distance expressed in world
	return norm(pf-l.o*l.p);
}
}
