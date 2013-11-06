#include <sot/core/matrix-homogeneous.hh>
#include <kdl/expressiontree.hpp>
#ifndef __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
#define __SOT_FEATURE_EXPRESSIONGRAPH_HELPER_HH__
using namespace dynamicgraph::sot;
KDL::Rotation mlHom2KDLRot (const MatrixHomogeneous &  M);
#endif
