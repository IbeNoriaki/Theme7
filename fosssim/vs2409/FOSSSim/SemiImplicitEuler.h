#ifndef __SEMI_IMPLICIT_EULER__
#define __SEMI_IMPLICIT_EULER__

#include "SceneStepper.h"

class SemiImplicitEuler : public SceneStepper
{
public:
  SemiImplicitEuler();
  
  virtual ~SemiImplicitEuler();
  
  virtual bool stepScene( TwoDScene& scene, scalar dt );
  
  virtual std::string getName() const;
};

#endif
