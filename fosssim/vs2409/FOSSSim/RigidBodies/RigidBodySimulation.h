#ifndef __RIGID_BODY_SIMULATION_H__
#define __RIGID_BODY_SIMULATION_H__

#include "FOSSSim/ExecutableSimulation.h"

#include "RigidBodyScene.h"

class RigidBodySimulation : public ExecutableSimulation
{
public:

//  ParticleSimulation( TwoDScene* scene, TwoDScene* comparison_scene, CollisionHandler* collisions_handler, SceneStepper* scene_stepper, TwoDSceneRenderer* scene_renderer, TwoDSceneSVGRenderer* svg_renderer )
//  : m_scene(scene)
//  , m_comparison_scene(comparison_scene)
//  , m_scene_stepper(scene_stepper)
//  , m_collision_handler(collisions_handler)
//  , m_comparison_impulses()
//  , m_scene_renderer(scene_renderer)
//  , m_svg_renderer(svg_renderer)
//  , m_scene_grader(NULL)
//  {
//    if( comparison_scene != NULL ) m_scene_grader = new TwoDSceneGrader;
//  }

  RigidBodySimulation( RigidBodyScene* scene, TwoDSceneRenderer* scene_renderer )
  : m_scene(scene)
  , m_scene_renderer(scene_renderer)
  {}
  
  virtual ~RigidBodySimulation()
  {}
  
  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions

  virtual void stepSystem( const scalar& dt )
  {
    std::cout << "Stepping rigid body simulation" << std::endl;
    //assert( m_scene != NULL );
    //assert( m_scene_stepper != NULL );

    // Step the simulated scene forward
    //m_scene_stepper->stepScene( *m_scene, dt );

    // Do collision response
    //if( m_collision_handler != NULL )
    //{
    //  ...
    //}
 
    // Check for obvious problems in the simulated scene
    //#ifdef DEBUG
    //  m_scene->checkConsistency();
    //#endif
  }

  /////////////////////////////////////////////////////////////////////////////
  // Scene Benchmarking Functions
  
  virtual void updateSceneComparison()
  {
    std::cout << "Updating Grading Comparison" << std::endl;
    //assert( m_scene != NULL );
    //assert( m_comparison_scene != NULL );
    //assert( m_scene_grader != NULL );
    //m_scene_grader->addToAccumulatedResidual( *m_scene, *m_comparison_scene );
  }

  virtual void printErrorInformation( bool print_pass )
  {
    std::cout << "Printing Error Information" << std::endl;
    //assert( m_scene_grader != NULL );
    //
    //scalar accumulatedPositionResidual = m_scene_grader->getAccumulatedPositionResidual();
    //scalar accumulatedVelocityResidual = m_scene_grader->getAccumulatedVelocityResidual();
    //scalar maxPositionResidual = m_scene_grader->getMaxPositionResidual();
    //scalar maxVelocityResidual = m_scene_grader->getMaxVelocityResidual();
    //
    //if( !print_pass )
    //{
    //  std::cout << "Accumulated Position Residual: " << accumulatedPositionResidual << std::endl;
    //  std::cout << "Accumulated Velocity Residual: " << accumulatedVelocityResidual << std::endl;
    //  std::cout << "Maximum Position Residual: "     << maxPositionResidual         << std::endl;
    //  std::cout << "Maximum Velocity Residual: "     << maxVelocityResidual         << std::endl;
    //
    //  std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink;
    //  std::cout << "Simulation did not run to completion. No pass/fail status will be reported." << std::endl;      
    //}
    //else
    //{
    //  bool accumulatedPositionResidualAcceptable = m_scene_grader->accumulatedPositionResidualPassed();
    //  bool accumulatedVelocityResidualAcceptable = m_scene_grader->accumulatedVelocityResidualPassed();
    //  bool maxPositionResidualAcceptable = m_scene_grader->maxPositionResidualPassed();
    //  bool maxVelocityResidualAcceptable = m_scene_grader->maxVelocityResidualPassed();
    //  
    //  bool total_success = accumulatedPositionResidualAcceptable && accumulatedVelocityResidualAcceptable && maxPositionResidualAcceptable && maxVelocityResidualAcceptable;
    //  
    //  std::cout << "Accumulated Position Residual: " << accumulatedPositionResidual << "   " << (accumulatedPositionResidualAcceptable?"Passed.":"Failed.") << std::endl;
    //  std::cout << "Accumulated Velocity Residual: " << accumulatedVelocityResidual << "   " << (accumulatedVelocityResidualAcceptable?"Passed.":"Failed.") << std::endl;
    //  std::cout << "Maximum Position Residual: "     << maxPositionResidual << "   "         << (maxPositionResidualAcceptable?"Passed.":"Failed.") << std::endl;
    //  std::cout << "Maximum Velocity Residual: "     << maxVelocityResidual << "   "         << (maxVelocityResidualAcceptable?"Passed.":"Failed.") << std::endl;
    //
    //  std::cout << "Overall success: " << (total_success?"Passed.":"Failed.") << std::endl;
    //  
    //  std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saved residual to residual.txt." << std::endl;
    //
    //  std::ofstream residfile;
    //  residfile.open("residual.txt");
    //  residfile << accumulatedPositionResidualAcceptable << "   " << accumulatedPositionResidual << std::endl;
    //  residfile << accumulatedVelocityResidualAcceptable << "   " << accumulatedVelocityResidual << std::endl;
    //  residfile << maxPositionResidualAcceptable << "   " << maxPositionResidual << std::endl;
    //  residfile << maxVelocityResidualAcceptable << "   " << maxVelocityResidual << std::endl;
    //  residfile.close();      
    //}
  }

  /////////////////////////////////////////////////////////////////////////////
  // Rendering Functions

  // TODO: Scrap these two functions
  virtual void initializeOpenGLRenderer()
  {}
  virtual void renderSceneDifferencesOpenGL()
  {}
  

  virtual void renderSceneOpenGL()
  {
    assert( m_scene != NULL );
    assert( m_scene_renderer != NULL );
    
    m_scene_renderer->renderRigdBodySimulation(*m_scene);
    
    //assert( m_scene_renderer != NULL );
    //m_scene_renderer->renderScene();
    //
    //if( m_comparison_scene != NULL )
    //{
    //  std::vector<ImpulseInfo> comparison_impulses;
    //  m_scene_renderer->circleMajorResiduals(*m_scene,*m_comparison_scene,NULL,&comparison_impulses);
    //}
  }

  virtual void updateOpenGLRendererState()
  {
    std::cout << "Updating OpenGL Rendering State" << std::endl;
    
    //assert( m_scene_renderer != NULL );
    //m_scene_renderer->updateState();
  }

  virtual void computeCameraCenter( renderingutils::Viewport& view )
  {
    const std::vector<RigidBody>& rbs = m_scene->getRigidBodies();

    scalar max_x = -std::numeric_limits<scalar>::infinity();
    scalar min_x =  std::numeric_limits<scalar>::infinity();
    scalar max_y = -std::numeric_limits<scalar>::infinity();
    scalar min_y =  std::numeric_limits<scalar>::infinity();    

    // For each rigid body
    for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i )
    {
      for( int j = 0; j < rbs[i].getNumVertices(); ++j )
      {
        Vector2s vrt = rbs[i].getWorldSpaceVertex(j);

        if( vrt.x() > max_x ) max_x = vrt.x();
        if( vrt.x() < min_x ) min_x = vrt.x();
        if( vrt.y() > max_y ) max_y = vrt.y();
        if( vrt.y() < min_y ) min_y = vrt.y();        
      }
    }

    // Set center of view to center of bounding box
    view.cx = 0.5*(max_x+min_x);
    view.cy = 0.5*(max_y+min_y);

    // Set the zoom such that all particles are in view
    view.rx = 0.5*(max_x-min_x);
    if( view.rx == 0.0 ) view.rx = 1.0;
    view.ry = 0.5*(max_y-min_y);
    if( view.ry == 0.0 ) view.ry = 1.0;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // SVG Rendering Functions

  virtual void renderSceneSVG( const std::string& name )
  {
    std::cout << "svg rendering called" << std::endl;
    //assert( m_svg_renderer != NULL );
    //
    //if( m_comparison_scene == NULL ) 
    //{
    //  m_svg_renderer->renderScene(name);
    //}
    //else 
    //{
    //  m_svg_renderer->renderComparisonScene( name, *m_comparison_scene,
    //                                        m_collision_handler != NULL ? &m_collision_handler->getImpulses() : NULL,
    //                                        &m_comparison_impulses );
    //}
  }

  virtual void updateSVGRendererState()
  {
    std::cout << "update svg rendering called" << std::endl;
    //assert( m_svg_renderer != NULL );
    //m_svg_renderer->updateState();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Serialization Functions
  
  virtual void copyComparisonSceneToScene()
  {
    std::cout << "copy comparison scene to scene called" << std::endl;
    //assert( m_scene != NULL );
    //assert( m_comparison_scene != NULL );
    //m_scene->copyState( *m_comparison_scene );
  }

  virtual void serializeScene( std::ofstream& outputstream )
  {
    std::cout << "serialize scene called" << std::endl;
    //assert( m_scene != NULL );
    //m_scene_serializer.serializeScene( *m_scene, outputstream );
    //
    //if( m_collision_handler != NULL )
    //{
    //  m_collision_handler->serializeImpulses( outputstream );
    //}    
  }

  virtual void serializeHybridData(std::ofstream &ofs)
  {
  }
  virtual void loadHybridData(std::ifstream &ifs)
  {
  }

  virtual void loadComparisonScene( std::ifstream& inputstream )
  {
    std::cout << "load comparison scene called" << std::endl;
    //assert( m_comparison_scene != NULL );
    //m_scene_serializer.loadScene( *m_comparison_scene, inputstream );
    //
    //if( m_collision_handler != NULL )
    //{
    //  m_comparison_impulses.clear();
    //  if( m_collision_handler != NULL ) m_collision_handler->loadImpulses( m_comparison_impulses, inputstream );
    //}
  }

  /////////////////////////////////////////////////////////////////////////////
  // Status Functions
  
  virtual std::string getSolverName()
  {
    std::cout << "get solver name called" << std::endl;
    //assert( m_scene_stepper != NULL );
    //return m_scene_stepper->getName();
    return "Not implemented.";
  }

  virtual std::string getCollisionHandlerName()
  {
    std::cout << "get collision handler name called" << std::endl;
    //if( m_comparison_scene == NULL ) return "Collisions disabled";
    //return m_collision_handler->getName();
    return "Not implemented.";
  }
  
  
private:

  RigidBodyScene* m_scene; 
  TwoDSceneRenderer* m_scene_renderer;

};

#endif
