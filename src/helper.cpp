#include "helper.h"

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

