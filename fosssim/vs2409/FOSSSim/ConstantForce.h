#ifndef __CONSTANT_FORCE_H__
#define __CONSTANT_FORCE_H__

#include <Eigen/Core>
#include <iostream>
#include <cassert>

#include "Force.h"
#include "MathDefs.h"

class ConstantForce : public Force
{
public:

  ConstantForce( const Vector2s& const_force );

  virtual ~ConstantForce();
  
  virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
  
  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
  
  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
  
  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
  
  virtual Force* createNewCopy();

private:
  Vector2s m_const_force;
};

#endif
