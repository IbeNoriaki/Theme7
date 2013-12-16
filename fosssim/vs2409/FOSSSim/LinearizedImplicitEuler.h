#ifndef __LINEARIZED_IMPLICIT_EULER__
#define __LINEARIZED_IMPLICIT_EULER__

#include <Eigen/Dense>
#include <iostream>

#include "SceneStepper.h"
#include "MathUtilities.h"
#include "StringUtilities.h"

class LinearizedImplicitEuler : public SceneStepper
{
public:
  LinearizedImplicitEuler();
  
  virtual ~LinearizedImplicitEuler();
  
  virtual bool stepScene( TwoDScene& scene, scalar dt );
  
  virtual std::string getName() const;

private:
  void zeroFixedDoFs( const TwoDScene& scene, VectorXs& vec );
  void setFixedRowsAndColsToIdentity( const TwoDScene& scene, MatrixXs& mat );
};

#endif
