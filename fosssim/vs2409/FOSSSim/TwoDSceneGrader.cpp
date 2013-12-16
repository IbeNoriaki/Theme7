#include "TwoDSceneGrader.h"

TwoDSceneGrader::TwoDSceneGrader()
: m_accumulated_position_residual(0.0)
, m_accumulated_velocity_residual(0.0)
, m_max_position_residual(0.0)
, m_max_velocity_residual(0.0)
, m_acceptable_accumulated_position_residual(1.0e-6)
, m_acceptable_accumulated_velocity_residual(1.0e-6)
, m_acceptable_max_position_residual(1.0e-10)
, m_acceptable_max_velocity_residual(1.0e-10)
, m_collisions_passed(true)
{}

void TwoDSceneGrader::addToAccumulatedResidual( const TwoDScene& oracle_scene, const TwoDScene& testing_scene )
{
  assert(   oracle_scene.getNumParticles() == testing_scene.getNumParticles() );
  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getX().size() );
  assert( 2*oracle_scene.getNumParticles() == testing_scene.getX().size() );
  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getV().size() );
  assert( 2*oracle_scene.getNumParticles() == testing_scene.getV().size() );
  
  const VectorXs& oracle_x = oracle_scene.getX();
  const VectorXs& testing_x = testing_scene.getX();
  
  const VectorXs& oracle_v = oracle_scene.getV();
  const VectorXs& testing_v = testing_scene.getV();
  
  for( int i = 0; i < oracle_scene.getNumParticles(); ++i )
  {
    scalar x_resid = (oracle_x.segment<2>(2*i)-testing_x.segment<2>(2*i)).norm();
    assert( x_resid >= 0.0 );
    scalar v_resid = (oracle_v.segment<2>(2*i)-testing_v.segment<2>(2*i)).norm();
    assert( v_resid >= 0.0 );
    m_accumulated_position_residual += x_resid;
    m_accumulated_velocity_residual += v_resid;
    if( x_resid > m_max_position_residual ) m_max_position_residual = x_resid;
    if( v_resid > m_max_velocity_residual ) m_max_velocity_residual = v_resid;
  }
}

scalar TwoDSceneGrader::getAccumulatedPositionResidual() const
{
  return m_accumulated_position_residual;
}

scalar TwoDSceneGrader::getAccumulatedVelocityResidual() const
{
  return m_accumulated_velocity_residual;
}

scalar TwoDSceneGrader::getMaxPositionResidual() const
{
  return m_max_position_residual;
}

scalar TwoDSceneGrader::getMaxVelocityResidual() const
{
  return m_max_velocity_residual;
}

bool TwoDSceneGrader::accumulatedPositionResidualPassed() const
{
  return m_accumulated_position_residual < m_acceptable_accumulated_position_residual;
}

bool TwoDSceneGrader::accumulatedVelocityResidualPassed() const
{
  return m_accumulated_velocity_residual < m_acceptable_accumulated_velocity_residual;
}

bool TwoDSceneGrader::maxPositionResidualPassed() const
{
  return m_max_position_residual < m_acceptable_max_position_residual;
}

bool TwoDSceneGrader::maxVelocityResidualPassed() const
{
  return m_max_velocity_residual < m_acceptable_max_velocity_residual;
}

bool TwoDSceneGrader::collisionsPassed() const
{
  return m_collisions_passed;
}

void TwoDSceneGrader::setCollisionsFailed() {m_collisions_passed = false;}
