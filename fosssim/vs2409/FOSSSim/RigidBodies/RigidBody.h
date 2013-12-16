#ifndef __RIGID_BODY_H__
#define __RIGID_BODY_H__

#include <Eigen/Core>
#include "FOSSSim/MathDefs.h"

#include <iostream>

class RigidBody
{
public:

  RigidBody( const VectorXs& vertices, const VectorXs& masses, const scalar& radius );

  int getNumVertices() const;
  const scalar& getRadius() const;

  Vector2s getWorldSpaceVertex( int i ) const;

  // TODO: Need an eigien_aligned_operator_new here?

private:
  Vector2s computeCenterOfMass( const VectorXs& vertices, const VectorXs& masses ) const;

  // Center of mass of the rigid body.
  Vector2s m_cm;
  // Orientation of the rigid body. Simply an angle in 2d. 
  scalar m_theta;
  // Array containing body-space coordinates of vertices that compose the rigid body.
  VectorXs m_vertices;
  // Thickness of the rigid body. Rigid body is Minkowski sum of circle of this radius and 'centerline' of rigid body.
  scalar m_r;
};

#endif
