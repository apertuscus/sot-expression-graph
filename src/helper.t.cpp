template<class T>
bool readHomogeneousMatrix(
    dynamicgraph::SignalPtr< dynamicgraph::sot::MatrixHomogeneous,int > & SIN,
    unsigned int base,
    const T & exp,
    const int time)
{
  if(SIN.isPlugged())
  {
    const dynamicgraph::sot::MatrixHomogeneous & homo = SIN(time);
    for( int i=0;i<3;++i )
      exp->setInputValue(base+i, homo.elementAt( i,3 ));
    exp->setInputValue(base+3,mlHom2KDLRot(homo));
    return true;
  }
  else
  {
    for( int i=0;i<6;++i )
      exp->setInputValue(base+i, 0);
    return false;
  }
}


template<class T>
bool readPositionVector(
    dynamicgraph::SignalPtr< ml::Vector,int >& SIN,
    unsigned int base,
    const T & exp,
    const int time)
{
  if(SIN.isPlugged())
  {
    const ml::Vector & p = SIN(time);
    for( int i=0;i<3;++i )
      exp->setInputValue(base+i, p(i));
    return true;
  }
  else
  {
    for( int i=0;i<3;++i )
      exp->setInputValue(base+i, 0);
    return false;
  }
}
