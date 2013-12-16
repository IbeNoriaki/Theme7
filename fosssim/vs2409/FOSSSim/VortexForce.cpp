#include "VortexForce.h"

VortexForce::VortexForce( const std::pair<int,int>& particles, const scalar& kbs, const scalar& kvc )
: Force()
, m_particles(particles)
, m_kbs(kbs)
, m_kvc(kvc)
{
  assert( m_particles.first >= 0 );
  assert( m_particles.second >= 0 );
  assert( m_particles.first != m_particles.second );
}

VortexForce::~VortexForce()
{}
  
void VortexForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size()%2 == 0 );
  assert( m_particles.first >= 0 );  assert( m_particles.first < x.size()/2 );
  assert( m_particles.second >= 0 ); assert( m_particles.second < x.size()/2 );

//  scalar r = (x.segment<2>(2*m_particles.second)-x.segment<2>(2*m_particles.first)).norm();
//  scalar m1 = m(2*m_particles.second);
//  scalar m2 = m(2*m_particles.first);
//  E += -m_G*m1*m2/r;

  std::cerr << outputmod::startred << "ERROR IN VORTEXFORCE: " << outputmod::endred << "No energy defined for VortexForce." << std::endl;
  exit(1);
}

void VortexForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == gradE.size() );
  assert( x.size()%2 == 0 );
  assert( m_particles.first >= 0 );  assert( m_particles.first < x.size()/2 );
  assert( m_particles.second >= 0 ); assert( m_particles.second < x.size()/2 );

  Vector2s rhat = x.segment<2>(2*m_particles.second)-x.segment<2>(2*m_particles.first); 
  scalar r = rhat.norm(); 
  assert( r != 0.0 ); 
  rhat /= r;
  rhat *= m_kbs;
  // Rotate rhat 90 degrees clockwise
  scalar temp = rhat.x();
  rhat.x() = -rhat.y();
  rhat.y() = temp;
  
  rhat -= v.segment<2>(2*m_particles.second)-v.segment<2>(2*m_particles.first);
  
  rhat /= r*r;
  rhat *= m_kvc;

  gradE.segment<2>(2*m_particles.first)  -= -rhat;
  gradE.segment<2>(2*m_particles.second) += -rhat;
}

void VortexForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );

//  scalar m1 = m(2*m_particles.second);
//  scalar m2 = m(2*m_particles.first);
//  
//  Vector2s nhat = x.segment<2>(2*m_particles.second)-x.segment<2>(2*m_particles.first); 
//  scalar r = nhat.norm(); 
//  assert( r != 0.0 ); 
//  nhat /= r;
//
//  Matrix2s entry = Matrix2s::Identity()-3.0*nhat*nhat.transpose();
//  entry *= m_G*m1*m2/r*r*r;
//  
//  hessE.block<2,2>(2*m_particles.first,2*m_particles.first)   += entry;
//  hessE.block<2,2>(2*m_particles.second,2*m_particles.second) += entry;
//  hessE.block<2,2>(2*m_particles.first,2*m_particles.second)  -= entry;
//  hessE.block<2,2>(2*m_particles.second,2*m_particles.first)  -= entry;

  std::cerr << outputmod::startred << "ERROR IN VORTEXFORCE: " << outputmod::endred << "No addHessXToTotal defined for VortexForce." << std::endl;
  exit(1);
}

void VortexForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( x.size() == hessE.cols() );
  assert( x.size()%2 == 0 );

  std::cerr << outputmod::startred << "ERROR IN VORTEXFORCE: " << outputmod::endred << "No addHessXToTotal defined for VortexForce." << std::endl;
  exit(1);
}

Force* VortexForce::createNewCopy()
{
  return new VortexForce(*this);
}
