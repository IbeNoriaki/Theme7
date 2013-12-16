#ifndef __EXECUTABLE_SIMULATION_H__
#define __EXECUTABLE_SIMULATION_H__

#include <string>
#include <Eigen/Core>
#include "MathDefs.h"
#include "RenderingUtilities.h"

class ExecutableSimulation
{
public:

  virtual ~ExecutableSimulation()
  {}

  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions
  
  virtual void stepSystem( const scalar& dt ) = 0;

  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions

  virtual void updateSceneComparison() = 0;
  virtual void printErrorInformation( bool print_pass ) = 0;
  
  /////////////////////////////////////////////////////////////////////////////
  // OpenGL Rendering Functions
  
  // TODO: Combine both of these
  virtual void renderSceneOpenGL() = 0;
  
  // TODO: Scrap these two functions, they are no longer needed
  virtual void renderSceneDifferencesOpenGL() = 0;
  virtual void initializeOpenGLRenderer() = 0;
  
  virtual void updateOpenGLRendererState() = 0;

  virtual void computeCameraCenter( renderingutils::Viewport& view ) = 0;
  
  /////////////////////////////////////////////////////////////////////////////
  // SVG Rendering Functions

  virtual void renderSceneSVG( const std::string& name ) = 0;

  virtual void updateSVGRendererState() = 0;

  /////////////////////////////////////////////////////////////////////////////
  // Serialization Functions
  
  virtual void copyComparisonSceneToScene() = 0;
  virtual void serializeScene( std::ofstream& outputstream ) = 0;
  virtual void loadComparisonScene( std::ifstream& inputstream ) = 0;

  /////////////////////////////////////////////////////////////////////////////
  // Status Functions

  virtual std::string getSolverName() = 0;
  virtual std::string getCollisionHandlerName() = 0;
};

#endif
