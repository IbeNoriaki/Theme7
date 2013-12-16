#include "ConstantForce.h"

ConstantForce::ConstantForce( const Vector2s& const_force )
: Force()
, m_const_force(const_force)
{
  assert( (const_force.array()==const_force.array()).all() );
  assert( (const_force.array()!=std::numeric_limits<scalar>::infinity()).all() );
}

ConstantForce::~ConstantForce()
{}
  
void ConstantForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size()%2 == 0 );

  // Assume 0 potential is at origin
  for( int i = 0; i < x.size()/2; ++i ) E -= m_const_force.dot(x.segment<2>(2*i));
}

void ConstantForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );

  for( int i = 0; i < x.size()/2; ++i ) gradE.segment<2>(2*i) -= m_const_force;
}

void ConstantForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  // Nothing to do.
}

void ConstantForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );
  // Nothing to do.
}

Force* ConstantForce::createNewCopy()
{
  return new ConstantForce(*this);
}
