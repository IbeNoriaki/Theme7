#include "RigidBodyScene.h"

RigidBodyScene::RigidBodyScene()
{
}

const std::vector<RigidBody>& RigidBodyScene::getRigidBodies() const
{
  return m_rbs;
}

void RigidBodyScene::addRigidBody( const RigidBody& rb )
{
  m_rbs.push_back(rb);
}

