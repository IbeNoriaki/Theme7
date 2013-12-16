#ifndef __RIGID_BODY_SCENE_H__
#define __RIGID_BODY_SCENE_H__

#include "FOSSSim/MathDefs.h"
#include <iostream>
#include "RigidBody.h"

class RigidBodyScene
{
public:
  RigidBodyScene();

  const std::vector<RigidBody>& getRigidBodies() const;
  
  void addRigidBody( const RigidBody& rb );
  
private:
  // Rigid bodies contained in the scene.
  std::vector<RigidBody> m_rbs;
};

#endif
