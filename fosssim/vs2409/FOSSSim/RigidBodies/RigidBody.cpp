#include "RigidBody.h"

RigidBody::RigidBody( const VectorXs& vertices, const VectorXs& masses, const scalar& radius )
: m_cm(computeCenterOfMass(vertices,masses))
, m_theta(0.0)
, m_vertices(vertices)
, m_r(radius)
{
  for( int i = 0; i < m_vertices.size()/2; ++i ) m_vertices.segment<2>(2*i) -= m_cm;
  
//  std::cout << "cm: " << m_cm.transpose() << std::endl;
//  std::cout << "theta: " << m_theta << std::endl;
//  std::cout << "points: " << m_vertices.transpose() << std::endl;
  
  assert( m_r >= 0.0 );
}


int RigidBody::getNumVertices() const
{
  assert( m_vertices.size()%2 == 0 );
  return m_vertices.size()/2;
}

const scalar& RigidBody::getRadius() const
{
  return m_r;
}


// For now just return the body-space coordinate
Vector2s RigidBody::getWorldSpaceVertex( int i ) const
{
  assert( i >= 0 );
  assert( i < getNumVertices() );

  // Rotate counter-clockwise by theta radians
  scalar c = cos(m_theta);
  scalar s = sin(m_theta);
  
  Vector2s vrt;
  vrt << c*m_vertices.segment<2>(2*i).x() - s*m_vertices.segment<2>(2*i).y(), s*m_vertices.segment<2>(2*i).x() + c*m_vertices.segment<2>(2*i).y();
  
  // Translate by center of mass
  vrt += m_cm;
  
  return vrt;
}

Vector2s RigidBody::computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const
{
  assert( vertices.size()%2 == 0 );
  assert( masses.size()%2 == 0 );
  assert( 2*masses.size() == vertices.size() );
  
  Vector2s cm(0.0,0.0);
  for( int i = 0; i < vertices.size()/2; ++i ) cm += masses(i)*vertices.segment<2>(2*i);
  
  scalar M = 0.0;
  for( int i = 0; i < masses.size(); ++i ) 
  {
    assert( masses(i) > 0.0 );
    M += masses(i);
  }
  
  assert( M > 0.0 );
  cm /= M;

  return cm;
}
