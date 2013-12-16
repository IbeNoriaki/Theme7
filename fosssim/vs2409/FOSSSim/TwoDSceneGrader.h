#ifndef __TWO_D_SCENE_GRADER_H__
#define __TWO_D_SCENE_GRADER_H__

#include "TwoDScene.h"

class TwoDSceneGrader
{
public:
  
  TwoDSceneGrader();
  
  void addToAccumulatedResidual( const TwoDScene& oracle_scene, const TwoDScene& testing_scene );
  void setCollisionsFailed();

  scalar getAccumulatedPositionResidual() const;
  scalar getAccumulatedVelocityResidual() const;
  scalar getMaxPositionResidual() const;
  scalar getMaxVelocityResidual() const;
  
  bool accumulatedPositionResidualPassed() const;
  bool accumulatedVelocityResidualPassed() const;
  bool maxPositionResidualPassed() const;
  bool maxVelocityResidualPassed() const;  

  bool collisionsPassed() const;
  
private:
  scalar m_accumulated_position_residual;
  scalar m_accumulated_velocity_residual;
  scalar m_max_position_residual;
  scalar m_max_velocity_residual;

  scalar m_acceptable_accumulated_position_residual;
  scalar m_acceptable_accumulated_velocity_residual;
  scalar m_acceptable_max_position_residual;
  scalar m_acceptable_max_velocity_residual;
  bool m_collisions_passed;
};

#endif
