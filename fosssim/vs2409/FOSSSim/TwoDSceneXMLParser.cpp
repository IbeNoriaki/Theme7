#include "TwoDSceneXMLParser.h"


TwoDScene *TwoDSceneXMLParser::getScene() {
  return myScene;
}

std::vector<renderingutils::Color> *TwoDSceneXMLParser::getParticleColors() {
  return colors;
}

void TwoDSceneXMLParser::loadExecutableSimulation( const std::string& file_name, bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, scalar& max_time, scalar& steps_per_sec_cap, renderingutils::Color& bgcolor, std::string& description, std::string& scenetag )
{
  // Load the xml document
  std::vector<char> xmlchars;
  rapidxml::xml_document<> doc;
  loadXMLFile( file_name, xmlchars, doc );

  // Attempt to locate the root node
  rapidxml::xml_node<>* node = doc.first_node("scene");
  if( node == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse xml scene file. Failed to locate root <scene> node. Exiting." << std::endl;
    exit(1);
  }
  
  // Determine what simulation type this is (particle, rigid body, etc)
  std::string simtype;
  loadSimulationType( node, simtype );
  
  // Parse common state
  loadMaxTime( node, max_time );
  loadMaxSimFrequency( node, steps_per_sec_cap );
  loadViewport( node, view );
  loadBackgroundColor( node, bgcolor );
  loadSceneDescriptionString( node, description );
  loadSceneTag( node, scenetag );
  
  // Parse the user-requested simulation type. The default is a particle simulation.
  if( simtype == "" || simtype == "particle-system" )
  {
    loadParticleSimulation( simulate_comparison, rendering_enabled, display_controller, execsim, view, dt, bgcolor, node );
  }
  else if( simtype == "rigid-body" )
  {
    loadRigidBodySimulation( simulate_comparison, rendering_enabled, display_controller, execsim, view, dt, bgcolor, node );
    //std::cout << "MAX_TIME: " << max_time << std::endl;
    //std::cout << "DT: " << dt << std::endl;
  }
  else 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid simtype '" << simtype << "' specified. Valid options are 'particle-system' and 'rigid-body'. Exiting." << std::endl;
    exit(1);
  }
}

void TwoDSceneXMLParser::loadParticleSimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node )
{
  TwoDScene* scene = new TwoDScene;

  // Scene
  loadParticles( node, *scene );
  loadEdges( node, *scene );
  loadHalfplanes( node, *scene );

  // Forces
  loadSpringForces( node, *scene );
  loadSimpleGravityForces( node, *scene );
  loadGravitationalForces( node, *scene );
  loadConstantForces( node, *scene );
  loadDragDampingForces( node, *scene );
  loadVorexForces( node, *scene );

  // Integrator/solver
  SceneStepper* scene_stepper = NULL;
  loadIntegrator( node, &scene_stepper, dt );
  assert( scene_stepper != NULL );
  assert( dt > 0.0 );

  CollisionHandler* collision_handler = NULL;
  loadCollisionHandler(node, *scene, &collision_handler);

  // Rendering state
  std::vector<renderingutils::Color> particle_colors;
  particle_colors.resize(scene->getNumParticles(),renderingutils::Color(0.650980392156863,0.294117647058824,0.0));
  loadParticleColors( node, particle_colors );
  colors = &particle_colors;

  std::vector<renderingutils::Color> edge_colors;
  edge_colors.resize(scene->getNumEdges(),renderingutils::Color(0.0,0.388235294117647,0.388235294117647));
  loadEdgeColors( node, edge_colors );

  std::vector<renderingutils::Color> halfplane_colors;
  halfplane_colors.resize(scene->getNumHalfplanes(), renderingutils::Color(0,0,0));
  loadHalfplaneColors( node, halfplane_colors );

  std::vector<renderingutils::ParticlePath> particle_paths;
  loadParticlePaths( node, dt, particle_paths );  
  
  
  TwoDScene* comparison_scene = NULL;
  if( simulate_comparison )
  {
    comparison_scene = new TwoDScene;
    comparison_scene->copyState(*scene);
  }
  
  TwoDSceneRenderer* scene_renderer = NULL;
  if( rendering_enabled )
  {
    scene_renderer = new TwoDSceneRenderer(*scene,display_controller,particle_colors,edge_colors,halfplane_colors,particle_paths);
    scene_renderer->updateParticleSimulationState(*scene);
  }
  
  TwoDSceneSVGRenderer* svg_renderer = new TwoDSceneSVGRenderer(*scene,display_controller,particle_colors,edge_colors,halfplane_colors,particle_paths,512,512,bgcolor);
  svg_renderer->updateState();
  
  *execsim = new ParticleSimulation(scene, comparison_scene, collision_handler, scene_stepper, scene_renderer, svg_renderer);

  myScene = scene;
}

void TwoDSceneXMLParser::loadRigidBodySimulation( bool simulate_comparison, bool rendering_enabled, TwoDimensionalDisplayController& display_controller, ExecutableSimulation** execsim, renderingutils::Viewport& view, scalar& dt, renderingutils::Color& bgcolor, rapidxml::xml_node<>* node )
{
  RigidBodyScene* scene = new RigidBodyScene;

  // Load locations and masses of rigid body boundary vertices
  std::vector<Vector2s> rbvrts;
  std::vector<scalar> rbmasses;
  loadRigidBodyVertices( node, rbvrts, rbmasses );
  assert( rbvrts.size() == rbmasses.size() );

  // Load the rigid bodies themselves
  std::vector<RigidBody> rigidbodies;
  loadRigidBodies( node, rbvrts, rbmasses, rigidbodies );
  for( std::vector<RigidBody>::size_type i = 0; i < rigidbodies.size(); ++i ) scene->addRigidBody(rigidbodies[i]);
  rbvrts.clear(); rbmasses.clear(); rigidbodies.clear();

  TwoDSceneRenderer* scene_renderer = new TwoDSceneRenderer(display_controller);
  *execsim = new RigidBodySimulation(scene,scene_renderer);
}

void TwoDSceneXMLParser::loadXMLFile( const std::string& filename, std::vector<char>& xmlchars, rapidxml::xml_document<>& doc )
{
  // Attempt to read the text from the user-specified xml file
  std::string filecontents;
  if( !loadTextFileIntoString(filename,filecontents) )
  {
    std::cerr << "\033[31;1mERROR IN TWODSCENEXMLPARSER:\033[m XML scene file " << filename << ". Failed to read file." << std::endl;
    exit(1);
  }
  
  // Copy string into an array of characters for the xml parser
  for( int i = 0; i < (int) filecontents.size(); ++i ) xmlchars.push_back(filecontents[i]);
  xmlchars.push_back('\0');
  
  // Initialize the xml parser with the character vector
  doc.parse<0>(&xmlchars[0]);
}

bool TwoDSceneXMLParser::loadTextFileIntoString( const std::string& filename, std::string& filecontents )
{
  // Attempt to open the text file for reading
  std::ifstream textfile(filename.c_str(),std::ifstream::in);
  if(!textfile) return false;
  
  // Read the entire file into a single string
  std::string line;
  while(getline(textfile,line)) filecontents.append(line);
  
  textfile.close();
  
  return true;
}

void TwoDSceneXMLParser::loadSimulationType( rapidxml::xml_node<>* node, std::string& simtype )
{
  assert( node != NULL );
  rapidxml::xml_node<>* nd = node->first_node("simtype");
  
  if( node->first_node("simtype") ) if( node->first_node("simtype")->first_attribute("type") ) simtype = node->first_node("simtype")->first_attribute("type")->value();
}

void TwoDSceneXMLParser::loadRigidBodyVertices( rapidxml::xml_node<>* node, std::vector<Vector2s>& vertices, std::vector<scalar>& masses )
{
  assert( node != NULL );

  int vertex = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("rigidbodyvertex"); nd; nd = nd->next_sibling("rigidbodyvertex") )
  {
    // Extract the vertex's initial position
    Vector2s pos(0.0,0.0);
    if( nd->first_attribute("x") ) 
    {
      std::string attribute(nd->first_attribute("x")->value());
      if( !stringutils::extractFromString(attribute,pos.x()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of x attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse x attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
      exit(1);
    }

    if( nd->first_attribute("y") ) 
    {
      std::string attribute(nd->first_attribute("y")->value());
      if( !stringutils::extractFromString(attribute,pos.y()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of y attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse y attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
      exit(1);
    }
    vertices.push_back(pos);

    // Extract the particle's mass
    scalar mass = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("m") ) 
    {
      std::string attribute(nd->first_attribute("m")->value());
      if( !stringutils::extractFromString(attribute,mass) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of m attribute for rigidbodyvertex " << vertex << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse m attribute for rigidbodyvertex " << vertex << ". Exiting." << std::endl;
      exit(1);
    }
    masses.push_back(mass);   
    
//    // Extract the particle's initial velocity
//    Vector2s vel;
//    if( nd->first_attribute("vx") ) 
//    {
//      std::string attribute(nd->first_attribute("vx")->value());
//      if( !stringutils::extractFromString(attribute,vel.x()) )
//      {
//        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vx attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
//        exit(1);
//      }
//    }
//    else 
//    {
//      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vx attribute for particle " << particle << ". Exiting." << std::endl;
//      exit(1);
//    }
//    
//    if( nd->first_attribute("vy") ) 
//    {
//      std::string attribute(nd->first_attribute("vy")->value());
//      if( !stringutils::extractFromString(attribute,vel.y()) )
//      {
//        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vy attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
//        exit(1);
//      }
//    }
//    else 
//    {
//      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vy attribute for particle " << particle << ". Exiting." << std::endl;
//      exit(1);
//    }
//    twodscene.setVelocity( particle, vel );
//
//    // Determine if the particle is fixed
//    bool fixed;
//    if( nd->first_attribute("fixed") ) 
//    {
//      std::string attribute(nd->first_attribute("fixed")->value());
//      if( !stringutils::extractFromString(attribute,fixed) )
//      {
//        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fixed attribute for particle " << particle << ". Value must be boolean. Exiting." << std::endl;
//        exit(1);
//      }
//    }
//    else 
//    {
//      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fixed attribute for particle " << particle << ". Exiting." << std::endl;
//      exit(1);
//    }
//    twodscene.setFixed( particle, fixed );
//    
//    // Extract the particle's radius, if present
//    scalar radius = 0.1;
//    if( nd->first_attribute("radius") )
//    {
//      std::string attribute(nd->first_attribute("radius")->value());
//      if( !stringutils::extractFromString(attribute,radius) )
//      {
//        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for particle " << particle << ". Value must be scalar. Exiting." << std::endl;
//        exit(1);
//      }
//    }    
//    twodscene.setRadius( particle, radius );
//    
//    // Extract the particle's tag, if present
//    if( nd->first_attribute("tag") )
//    {
//      std::string tag(nd->first_attribute("tag")->value());
//      tags[particle] = tag;
//    }    
//    
//    //std::cout << "Particle: " << particle << "    x: " << pos.transpose() << "   v: " << vel.transpose() << "   m: " << mass << "   fixed: " << fixed << std::endl;
//    //std::cout << tags[particle] << std::endl;
//    
    ++vertex;
  }  
}

void TwoDSceneXMLParser::loadRigidBodies( rapidxml::xml_node<>* node, const std::vector<Vector2s>& vertices, const std::vector<scalar>& masses, std::vector<RigidBody>& rigidbodies )
{
  int body = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("rigidbody"); nd; nd = nd->next_sibling("rigidbody") )
  {
    // Extract the indices that make up this rigid body
    std::vector<int> vrtidxs;
    for( rapidxml::xml_attribute<>* vrtnd = nd->first_attribute("p"); vrtnd; vrtnd = vrtnd->next_attribute("p") )
    {
      vrtidxs.push_back(-1);
      if( !stringutils::extractFromString(vrtnd->value(),vrtidxs.back()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of p attribute for rigidbody " << body << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }
      // Ensure that the requested vertex actually was specified
      if( vrtidxs.back() < 0 || vrtidxs.back() >= (int) masses.size() )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of p attribute for rigidbody " << body << ". Value must be in range [0," << (masses.size()-1) << "] inclusive. Value given of " << vrtidxs.back() << ". Exiting." << std::endl;
        exit(1);
      }
    }

    // Must have at least 3 indices
    if( vrtidxs.size() < 3 ) 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse rigidbody " << body << ". Must specify at least 3 vertices. Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the radius of the rigid body
    scalar radius = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("r") ) 
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,radius) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbody " << body << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
      if( radius <= 0.0 )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for rigidbody " << body << ". Value must be positive. Value give of " << radius << ". Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse r attribute for rigidbody " << body << ". Exiting." << std::endl;
      exit(1);
    }

    // Create the rigid body
    VectorXs bodiesverts(2*vrtidxs.size());
    for( std::vector<int>::size_type i = 0; i < vrtidxs.size(); ++i ) bodiesverts.segment<2>(2*i) = vertices[vrtidxs[i]];
    VectorXs bodiesmasses(vrtidxs.size());
    for( std::vector<int>::size_type i = 0; i < vrtidxs.size(); ++i ) bodiesmasses(i) = masses[vrtidxs[i]];

    rigidbodies.push_back(RigidBody(bodiesverts,bodiesmasses,radius));
    
    ++body;
  }  
}





void TwoDSceneXMLParser::loadParticles( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  // Count the number of particles
  int numparticles = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("particle"); nd; nd = nd->next_sibling("particle") ) ++numparticles;
  
  twodscene.resizeSystem(numparticles);
  
  //std::cout << "Num particles " << numparticles << std::endl;
  
  std::vector<std::string>& tags = twodscene.getParticleTags();

  int particle = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("particle"); nd; nd = nd->next_sibling("particle") )
  {
    // Extract the particle's initial position
    Vector2s pos;
    if( nd->first_attribute("px") ) 
    {
      std::string attribute(nd->first_attribute("px")->value());
      if( !stringutils::extractFromString(attribute,pos.x()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse px attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }

    if( nd->first_attribute("py") ) 
    {
      std::string attribute(nd->first_attribute("py")->value());
      if( !stringutils::extractFromString(attribute,pos.y()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse py attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }
    twodscene.setPosition( particle, pos );

    // Extract the particle's initial velocity
    Vector2s vel;
    if( nd->first_attribute("vx") ) 
    {
      std::string attribute(nd->first_attribute("vx")->value());
      if( !stringutils::extractFromString(attribute,vel.x()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vx attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vx attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( nd->first_attribute("vy") ) 
    {
      std::string attribute(nd->first_attribute("vy")->value());
      if( !stringutils::extractFromString(attribute,vel.y()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of vy attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse vy attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }
    twodscene.setVelocity( particle, vel );

    // Extract the particle's mass
    scalar mass = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("m") ) 
    {
      std::string attribute(nd->first_attribute("m")->value());
      if( !stringutils::extractFromString(attribute,mass) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of m attribute for particle " << particle << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse m attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }
    twodscene.setMass( particle, mass );
    
    // Determine if the particle is fixed
    bool fixed;
    if( nd->first_attribute("fixed") ) 
    {
      std::string attribute(nd->first_attribute("fixed")->value());
      if( !stringutils::extractFromString(attribute,fixed) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fixed attribute for particle " << particle << ". Value must be boolean. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fixed attribute for particle " << particle << ". Exiting." << std::endl;
      exit(1);
    }
    twodscene.setFixed( particle, fixed );

    // Extract the particle's radius, if present
    scalar radius = 0.1;
    if( nd->first_attribute("radius") )
    {
      std::string attribute(nd->first_attribute("radius")->value());
      if( !stringutils::extractFromString(attribute,radius) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for particle " << particle << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }
    }    
    twodscene.setRadius( particle, radius );

    // Extract the particle's tag, if present
    if( nd->first_attribute("tag") )
    {
      std::string tag(nd->first_attribute("tag")->value());
      tags[particle] = tag;
    }    

    //std::cout << "Particle: " << particle << "    x: " << pos.transpose() << "   v: " << vel.transpose() << "   m: " << mass << "   fixed: " << fixed << std::endl;
    //std::cout << tags[particle] << std::endl;
    
    ++particle;
  }
}

void TwoDSceneXMLParser::loadSceneTag( rapidxml::xml_node<>* node, std::string& scenetag )
{
  assert( node != NULL );

  if( node->first_node("scenetag") )
  {
    if( node->first_node("scenetag")->first_attribute("tag") )
    {
      scenetag = node->first_node("scenetag")->first_attribute("tag")->value();
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of tag attribute for scenetag. Value must be string. Exiting." << std::endl;
      exit(1);
    }
  }
}

void TwoDSceneXMLParser::loadHalfplanes( rapidxml::xml_node<> *node, TwoDScene &twodscene)
{
  assert( node != NULL );

  twodscene.clearHalfplanes();

  int halfplane = 0;
  for( rapidxml::xml_node<> *nd = node->first_node("halfplane"); nd; nd = nd->next_sibling("halfplane") )
    {
      VectorXs x(2);
      VectorXs n(2);
      
      if( nd->first_attribute("px") )
	{
	  std::string attribute(nd->first_attribute("px")->value());
	  if( !stringutils::extractFromString(attribute, x[0]) )
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
	      exit(0);
	    }
	}
      else
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of px attribute for halfplane " << halfplane << ". Exiting." << std::endl;
	  exit(0);
	}
      if( nd->first_attribute("py") )
	{
	  std::string attribute(nd->first_attribute("py")->value());
	  if( !stringutils::extractFromString(attribute, x[1]) )
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
	      exit(0);
	    }
	}
      else
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of py attribute for halfplane " << halfplane << ". Exiting." << std::endl;
	  exit(0);
	}
      if( nd->first_attribute("nx") )
	{
	  std::string attribute(nd->first_attribute("nx")->value());
	  if( !stringutils::extractFromString(attribute, n[0]) )
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of nx attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
	      exit(0);
	    }
	}
      else
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of nx attribute for halfplane " << halfplane << ". Exiting." << std::endl;
	  exit(0);
	}
      if( nd->first_attribute("ny") )
	{
	  std::string attribute(nd->first_attribute("ny")->value());
	  if( !stringutils::extractFromString(attribute, n[1]) )
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ny attribute for halfplane " << halfplane << ". Value must be real number. Exiting." << std::endl;
	      exit(0);
	    }
	}
      else
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of ny attribute for halfplane " << halfplane << ". Exiting." << std::endl;
	  exit(0);
	}
      twodscene.insertHalfplane(std::pair<VectorXs, VectorXs>(x,n));

      ++halfplane;
    }
}

void TwoDSceneXMLParser::loadEdges( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );

  twodscene.clearEdges();
  
  int edge = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("edge"); nd; nd = nd->next_sibling("edge") )
  {
    std::pair<int,int> newedge;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,newedge.first) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edge " << edge << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edge " << edge << ". Exiting." << std::endl;
      exit(1);
    }

    if( nd->first_attribute("j") )
    {
      std::string attribute(nd->first_attribute("j")->value());
      if( !stringutils::extractFromString(attribute,newedge.second) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for edge " << edge << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for edge " << edge << ". Exiting." << std::endl;
      exit(1);
    }
    
    scalar radius = 0.015;
    if( nd->first_attribute("radius") )
    {
      std::string attribute(nd->first_attribute("radius")->value());
      if( !stringutils::extractFromString(attribute,radius) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse radius attribute for edge " << edge << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }
    }

    //std::cout << "Edge: " << edge << "    i: " << newedge.first << "   j: " << newedge.second << std::endl;
    
    twodscene.insertEdge( newedge, radius );
    
    ++edge;
  }
}

void TwoDSceneXMLParser::loadSpringForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  // Extract the edge the force acts across
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("springforce"); nd; nd = nd->next_sibling("springforce") )
  {
    int edge = -1;

    if( nd->first_attribute("edge") )
    {
      std::string attribute(nd->first_attribute("edge")->value());
      if( !stringutils::extractFromString(attribute,edge) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of edge attribute for springforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of edge attribute for springforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }      
  
    std::pair<int,int> newedge(twodscene.getEdge(edge));
    
    // Extract the spring stiffness
    scalar k = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("k") ) 
    {
      std::string attribute(nd->first_attribute("k")->value());
      if( !stringutils::extractFromString(attribute,k) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of k attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse k attribute for springforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the spring rest length
    scalar l0 = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("l0") ) 
    {
      std::string attribute(nd->first_attribute("l0")->value());
      if( !stringutils::extractFromString(attribute,l0) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of l0 attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse l0 attribute for springforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }

    // Extract the optional damping coefficient
    scalar b = 0.0;
    if( nd->first_attribute("b") ) 
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,b) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for springforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    
    //std::cout << "Springforce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   k: " << k << "   l0: " << l0 << std::endl;
    
    twodscene.insertForce(new SpringForce(newedge,k,l0,b));
    
    ++forcenum;
  }

  //SpringForce( const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0 )
}

void TwoDSceneXMLParser::loadGravitationalForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  // Extract the edge the force acts across
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("gravitationalforce"); nd; nd = nd->next_sibling("gravitationalforce") )
  {
    std::pair<int,int> newedge;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,newedge.first) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for gravitationalforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( nd->first_attribute("j") )
    {
      std::string attribute(nd->first_attribute("j")->value());
      if( !stringutils::extractFromString(attribute,newedge.second) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for gravitationalforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the gravitational constant
    scalar G = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("G") )
    {
      std::string attribute(nd->first_attribute("G")->value());
      if( !stringutils::extractFromString(attribute,G) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of G attribute for gravitationalforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse G attribute for gravitationalforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }

    //std::cout << "GravitationalForce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   G: " << G << std::endl;

    twodscene.insertForce(new GravitationalForce(newedge,G));
    
    ++forcenum;
  }
  
  //SpringForce( const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0 )  
}


void TwoDSceneXMLParser::loadConstantForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );

  // Load each constant force
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("constantforce"); nd; nd = nd->next_sibling("constantforce") )
  {
    Vector2s constforce;
    constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    
    // Extract the x component of the force
    if( nd->first_attribute("fx") ) 
    {
      std::string attribute(nd->first_attribute("fx")->value());
      if( !stringutils::extractFromString(attribute,constforce.x()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for constantforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }

    // Extract the y component of the force
    if( nd->first_attribute("fy") ) 
    {
      std::string attribute(nd->first_attribute("fy")->value());
      if( !stringutils::extractFromString(attribute,constforce.y()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for constantforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }

    //std::cout << "x: " << constforce.transpose() << std::endl;

    twodscene.insertForce(new ConstantForce(constforce));

    ++forcenum;
  }
}

void TwoDSceneXMLParser::loadDragDampingForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("dragdamping"); nd; nd = nd->next_sibling("dragdamping") )
  {
    Vector2s constforce;
    constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    
    // Extract the linear damping coefficient
    scalar b = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,b) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for dragdamping " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse b attribute for dragdamping " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    //std::cout << "x: " << constforce.transpose() << std::endl;
    
    twodscene.insertForce(new DragDampingForce(b));
    
    ++forcenum;
  }  
}

void TwoDSceneXMLParser::loadVorexForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  // Extract the edge the force acts across
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("vortexforce"); nd; nd = nd->next_sibling("vortexforce") )
  {
    std::pair<int,int> newedge;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,newedge.first) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for vortexforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( nd->first_attribute("j") )
    {
      std::string attribute(nd->first_attribute("j")->value());
      if( !stringutils::extractFromString(attribute,newedge.second) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for vortexforce " << forcenum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of j attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the 'Biot-Savart' constant
    scalar kbs = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("kbs") )
    {
      std::string attribute(nd->first_attribute("kbs")->value());
      if( !stringutils::extractFromString(attribute,kbs) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of kbs attribute for vortexforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse kbs attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the viscosity constant
    scalar kvc = std::numeric_limits<scalar>::signaling_NaN();
    if( nd->first_attribute("kvc") )
    {
      std::string attribute(nd->first_attribute("kvc")->value());
      if( !stringutils::extractFromString(attribute,kvc) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of kvc attribute for vortexforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse kvc attribute for vortexforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }    
    
    //std::cout << "VortexForce: " << forcenum << "    i: " << newedge.first << "   j: " << newedge.second << "   kbs: " << kbs << "   kvc: " << kvc << std::endl;
    
    twodscene.insertForce(new VortexForce(newedge,kbs,kvc));
    
    ++forcenum;
  }
}

void TwoDSceneXMLParser::loadSimpleGravityForces( rapidxml::xml_node<>* node, TwoDScene& twodscene )
{
  assert( node != NULL );
  
  // Load each constant force
  int forcenum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("simplegravity"); nd; nd = nd->next_sibling("simplegravity") )
  {
    Vector2s constforce;
    constforce.setConstant(std::numeric_limits<scalar>::signaling_NaN());
    
    // Extract the x component of the force
    if( nd->first_attribute("fx") ) 
    {
      std::string attribute(nd->first_attribute("fx")->value());
      if( !stringutils::extractFromString(attribute,constforce.x()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fx attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fx attribute for constantforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    // Extract the y component of the force
    if( nd->first_attribute("fy") ) 
    {
      std::string attribute(nd->first_attribute("fy")->value());
      if( !stringutils::extractFromString(attribute,constforce.y()) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of fy attribute for constantforce " << forcenum << ". Value must be numeric. Exiting." << std::endl;
        exit(1);
      }
    }
    else 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse fy attribute for constantforce " << forcenum << ". Exiting." << std::endl;
      exit(1);
    }
    
    //std::cout << "x: " << constforce.transpose() << std::endl;
    
    twodscene.insertForce(new SimpleGravityForce(constforce));
    
    ++forcenum;
  }  
}

void TwoDSceneXMLParser::loadCollisionHandler( rapidxml::xml_node<>* node, TwoDScene &scene, CollisionHandler **handler )
{
  assert( node != NULL );

  rapidxml::xml_node<>* nd = node->first_node("collision");
  if( nd == NULL )
    {
      *handler = NULL;
      return;
    }

  // Load value of COR
  double cor = 1.0;
  rapidxml::xml_attribute<> *cornd = nd->first_attribute("COR");
  if( cornd != NULL )
    { 
      if( !stringutils::extractFromString(std::string(cornd->value()),cor) )
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'COR' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
	  exit(1);
	}
    }
  
  // Attempt to load the collision handler type
  rapidxml::xml_attribute<> *typend = nd->first_attribute("type");
  if( typend == NULL )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No collision handler 'type' attribute specified. Exiting." << std::endl;
      exit(1);
    }
  std::string handlertype(typend->value());
  if(handlertype == "none") *handler = NULL;
  else if(handlertype == "simple") *handler = new SimpleCollisionHandler(cor);
  else if(handlertype == "continuous-time") *handler = new ContinuousTimeCollisionHandler(cor);
  else if(handlertype == "hybrid")
  {
      // Load max iteration attribute
      double maxiters=10;
      rapidxml::xml_attribute<> *maxiternd = nd->first_attribute("maxiters");
      if(maxiternd != NULL)
      {
          if(!stringutils::extractFromString(std::string(maxiternd->value()), maxiters))
          {
              std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'maxiters' attribute for collision handler. Value must be numeric. Exiting."<< std::endl;
              exit(1);
          }
      }
      *handler = new HybridCollisionHandler(maxiters, cor);
      
      // Attempt to load stiffness attribute
      double k=10;
      rapidxml::xml_attribute<> *knd = nd->first_attribute("k");
      if(knd != NULL)
      {
          if(!stringutils::extractFromString(std::string(knd->value()), k))
          {
              std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'k' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
              exit(1);
          }
      }
      
      double thickness=0.1;
      rapidxml::xml_attribute<> *thicknd = nd->first_attribute("thickness");
      if(thicknd != NULL)
      {
          if(!stringutils::extractFromString(std::string(thicknd->value()), thickness))
          {
              std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'thickness' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
              exit(1);
          }
      }
      scene.insertForce(new PenaltyForce(scene, k, thickness));
  } else if(handlertype == "penalty")
    {
      // Attempt to load stiffness attribute
      double k=100;
      rapidxml::xml_attribute<> *knd = nd->first_attribute("k");
      if(knd != NULL)
	{
	  if(!stringutils::extractFromString(std::string(knd->value()), k))
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'k' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
	      exit(1);
	    }
	}

      double thickness=0.01;
      rapidxml::xml_attribute<> *thicknd = nd->first_attribute("thickness");
      if(thicknd != NULL)
	{
	  if(!stringutils::extractFromString(std::string(thicknd->value()), thickness))
	    {
	      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'thickness' attribute for collision handler. Value must be numeric. Exiting." << std::endl;
	      exit(1);
	    }
	}
      *handler = NULL;
      scene.insertForce(new PenaltyForce(scene, k, thickness));
    }
  else
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid collision handler 'type' attribute specified. Exiting." << std::endl;
    exit(1);
  }

}

void TwoDSceneXMLParser::loadIntegrator( rapidxml::xml_node<>* node, SceneStepper** scenestepper, scalar& dt )
{
  assert( node != NULL );
  
  dt = -1.0;
  
  // Attempt to locate the integrator node
  rapidxml::xml_node<>* nd = node->first_node("integrator");
  if( nd == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator specified. Exiting." << std::endl;
    exit(1);
  }
  
  // Attempt to load the integrator type
  rapidxml::xml_attribute<>* typend = nd->first_attribute("type"); 
  if( typend == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator 'type' attribute specified. Exiting." << std::endl;
    exit(1);
  }
  std::string integratortype(typend->value());
  
  if( integratortype == "explicit-euler" ) *scenestepper = new ExplicitEuler;
  else if( integratortype == "forward-backward-euler" || integratortype == "symplectic-euler" ) *scenestepper = new SemiImplicitEuler;
  else if( integratortype == "implicit-euler" ) *scenestepper = new ImplicitEuler;
  else if( integratortype == "linearized-implicit-euler" ) *scenestepper = new LinearizedImplicitEuler;
  else
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Invalid integrator 'type' attribute specified. Exiting." << std::endl;
    exit(1);
  }

  // Attempt to load the timestep
  rapidxml::xml_attribute<>* dtnd = nd->first_attribute("dt"); 
  if( dtnd == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No integrator 'dt' attribute specified. Exiting." << std::endl;
    exit(1);
  }

  dt = std::numeric_limits<scalar>::signaling_NaN();
  if( !stringutils::extractFromString(std::string(dtnd->value()),dt) )
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'dt' attribute for integrator. Value must be numeric. Exiting." << std::endl;
    exit(1);
  }

  //std::cout << "Integrator: " << (*scenestepper)->getName() << "   dt: " << dt << std::endl;
}

void TwoDSceneXMLParser::loadMaxTime( rapidxml::xml_node<>* node, scalar& max_t )
{
  assert( node != NULL );

  // Attempt to locate the duraiton node
  rapidxml::xml_node<>* nd = node->first_node("duration");
  if( nd == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No duration specified. Exiting." << std::endl;
    exit(1);
  }
  
  // Attempt to load the duration value
  rapidxml::xml_attribute<>* timend = nd->first_attribute("time"); 
  if( timend == NULL ) 
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No duration 'time' attribute specified. Exiting." << std::endl;
    exit(1);
  }
  
  max_t = std::numeric_limits<scalar>::signaling_NaN();
  if( !stringutils::extractFromString(std::string(timend->value()),max_t) )
  {
    std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'time' attribute for duration. Value must be numeric. Exiting." << std::endl;
    exit(1);
  }
}

void TwoDSceneXMLParser::loadViewport(rapidxml::xml_node<>* node, renderingutils::Viewport &view)
{
  assert( node != NULL );

  if(node->first_node("viewport") )
    {
      rapidxml::xml_attribute<> *cx = node->first_node("viewport")->first_attribute("cx");
      if(cx == NULL)
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'cx' attribute specified. Exiting." << std::endl;
	  exit(1);
	}
      if(!stringutils::extractFromString(std::string(cx->value()),view.cx))
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'cx' attribute for viewport. Value must be scalar. Exiting." << std::endl;
	  exit(1);
	}
      rapidxml::xml_attribute<> *cy = node->first_node("viewport")->first_attribute("cy");
      if(cy == NULL)
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'cy' attribute specified. Exiting." << std::endl;
	  exit(1);
	}
      if(!stringutils::extractFromString(std::string(cy->value()),view.cy))
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'cy' attribute for viewport. Value must be scalar. Exiting." << std::endl;
	  exit(1);
	}
      rapidxml::xml_attribute<> *size = node->first_node("viewport")->first_attribute("size");
      if(size == NULL)
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No viewport 'size' attribute specified. Exiting." << std::endl;
	  exit(1);
	}
      if(!stringutils::extractFromString(std::string(size->value()),view.size))
	{
	  std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'size' attribute for viewport. Value must be scalar. Exiting." << std::endl;
	  exit(1);
	}
    }
}

void TwoDSceneXMLParser::loadMaxSimFrequency( rapidxml::xml_node<>* node, scalar& max_freq )
{
  assert( node != NULL );
  
  // Attempt to locate the duraiton node
  if( node->first_node("maxsimfreq") ) 
  {
    // Attempt to load the duration value
    rapidxml::xml_attribute<>* atrbnde = node->first_node("maxsimfreq")->first_attribute("max"); 
    if( atrbnde == NULL ) 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No maxsimfreq 'max' attribute specified. Exiting." << std::endl;
      exit(1);
    }

    if( !stringutils::extractFromString(std::string(atrbnde->value()),max_freq) )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse 'max' attribute for maxsimfreq. Value must be scalar. Exiting." << std::endl;
      exit(1);
    }
  }
}




void TwoDSceneXMLParser::loadBackgroundColor( rapidxml::xml_node<>* node, renderingutils::Color& color )
{
  if( rapidxml::xml_node<>* nd = node->first_node("backgroundcolor") )
  {
    // Read in the red color channel 
    double red = -1.0;
    if( nd->first_attribute("r") )
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,red) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Exiting." << std::endl;
      exit(1);
    }
    
    if( red < 0.0 || red > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the green color channel 
    double green = -1.0;
    if( nd->first_attribute("g") )
    {
      std::string attribute(nd->first_attribute("g")->value());
      if( !stringutils::extractFromString(attribute,green) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Exiting." << std::endl;
      exit(1);
    }
    
    if( green < 0.0 || green > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the blue color channel 
    double blue = -1.0;
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,blue) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Exiting." << std::endl;
      exit(1);
    }
    
    if( blue < 0.0 || blue > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for backgroundcolor. Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }

    //std::cout << red << "   " << green << "   " << blue << std::endl;

    color.r = red;
    color.g = green;
    color.b = blue;  
  }
}

void TwoDSceneXMLParser::loadParticleColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& particle_colors )
{
  int particlecolornum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("particlecolor"); nd; nd = nd->next_sibling("particlecolor") )
  {
    // Determine which particle this color corresponds to
    int particle = -1;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,particle) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( particle < 0 || particle >= (int) particle_colors.size() )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlecolor " << particlecolornum << ". Invalid particle specified. Valid range is " << 0 << "..." << particle_colors.size()-1 << std::endl;
      exit(1);
    }

      
    // Read in the red color channel 
    double red = -1.0;
    if( nd->first_attribute("r") )
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,red) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
      exit(1);
    }

    if( red < 0.0 || red > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the green color channel 
    double green = -1.0;
    if( nd->first_attribute("g") )
    {
      std::string attribute(nd->first_attribute("g")->value());
      if( !stringutils::extractFromString(attribute,green) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
      exit(1);
    }

    if( green < 0.0 || green > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the blue color channel 
    double blue = -1.0;
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,blue) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Exiting." << std::endl;
      exit(1);
    }

    if( blue < 0.0 || blue > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlecolor " << particlecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    particle_colors[particle] = renderingutils::Color(red,green,blue);
    
    ++particlecolornum;
  }
}

void TwoDSceneXMLParser::loadEdgeColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& edge_colors )
{
  int edgecolornum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("edgecolor"); nd; nd = nd->next_sibling("edgecolor") )
  {
    // Determine which particle this color corresponds to
    int edge = -1;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,edge) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( edge < 0 || edge > (int) edge_colors.size() )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for edgecolor " << edgecolornum << ". Invalid edge specified. Valid range is " << 0 << "..." << edge_colors.size()-1 << std::endl;
      exit(1);
    }
    
    
    // Read in the red color channel 
    double red = -1.0;
    if( nd->first_attribute("r") )
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,red) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( red < 0.0 || red > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the green color channel 
    double green = -1.0;
    if( nd->first_attribute("g") )
    {
      std::string attribute(nd->first_attribute("g")->value());
      if( !stringutils::extractFromString(attribute,green) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( green < 0.0 || green > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the blue color channel 
    double blue = -1.0;
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,blue) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( blue < 0.0 || blue > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for edgecolor " << edgecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    //std::cout << "edge: " << edge << " r: " << red << " g: " << green << " b: " << blue << std::endl;
    
    edge_colors[edge] = renderingutils::Color(red,green,blue);
    
    ++edgecolornum;
  }  
}

void TwoDSceneXMLParser::loadHalfplaneColors( rapidxml::xml_node<>* node, std::vector<renderingutils::Color>& halfplane_colors )
{
  int halfplanecolornum = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("halfplanecolor"); nd; nd = nd->next_sibling("halfplanecolor") )
  {
    // Determine which particle this color corresponds to
    int halfplane = -1;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,halfplane) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( halfplane < 0 || halfplane > (int) halfplane_colors.size() )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for halfplanecolor " << halfplanecolornum << ". Invalid half-plane specified. Valid range is " << 0 << "..." << halfplane_colors.size()-1 << std::endl;
      exit(1);
    }
    
    
    // Read in the red color channel 
    double red = -1.0;
    if( nd->first_attribute("r") )
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,red) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( red < 0.0 || red > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the green color channel 
    double green = -1.0;
    if( nd->first_attribute("g") )
    {
      std::string attribute(nd->first_attribute("g")->value());
      if( !stringutils::extractFromString(attribute,green) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( green < 0.0 || green > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the blue color channel 
    double blue = -1.0;
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,blue) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( blue < 0.0 || blue > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for halfplanecolor " << halfplanecolornum << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    halfplane_colors[halfplane] = renderingutils::Color(red,green,blue);
    
    ++halfplanecolornum;
  }  
}

void TwoDSceneXMLParser::loadParticlePaths( rapidxml::xml_node<>* node, double dt, std::vector<renderingutils::ParticlePath>& particle_paths )
{
  int numpaths = 0;
  for( rapidxml::xml_node<>* nd = node->first_node("particlepath"); nd; nd = nd->next_sibling("particlepath") )
  {
    // Determine which particle this color corresponds to
    int particle = -1;
    if( nd->first_attribute("i") )
    {
      std::string attribute(nd->first_attribute("i")->value());
      if( !stringutils::extractFromString(attribute,particle) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlepath " << numpaths << ". Value must be integer. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of i attribute for particlepath " << numpaths << ". Exiting." << std::endl;
      exit(1);
    }
    
    // How long the path should be buffered for
    double duration = -1.0;
    if( nd->first_attribute("duration") )
    {
      std::string attribute(nd->first_attribute("duration")->value());
      if( !stringutils::extractFromString(attribute,duration) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( duration < 0.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of duration attribute for particlepath " << numpaths << ". Value must be positive scalar." << std::endl;
      exit(1);
    }
    
    
    // Read in the red color channel 
    double red = -1.0;
    if( nd->first_attribute("r") )
    {
      std::string attribute(nd->first_attribute("r")->value());
      if( !stringutils::extractFromString(attribute,red) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( red < 0.0 || red > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of r attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the green color channel 
    double green = -1.0;
    if( nd->first_attribute("g") )
    {
      std::string attribute(nd->first_attribute("g")->value());
      if( !stringutils::extractFromString(attribute,green) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( green < 0.0 || green > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of g attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    
    // Read in the blue color channel 
    double blue = -1.0;
    if( nd->first_attribute("b") )
    {
      std::string attribute(nd->first_attribute("b")->value());
      if( !stringutils::extractFromString(attribute,blue) )
      {
        std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Value must be scalar. Exiting." << std::endl;
        exit(1);
      }        
    }
    else
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Exiting." << std::endl;
      exit(1);
    }
    
    if( blue < 0.0 || blue > 1.0 )
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m Failed to parse value of b attribute for particlepath " << numpaths << ". Invalid color specified. Valid range is " << 0.0 << "..." << 1.0 << std::endl;
      exit(1);
    }
    
    particle_paths.push_back(renderingutils::ParticlePath( particle, ceil(duration/dt), renderingutils::Color(red,green,blue) ));

    //std::cout << particle << " " << duration << " " << red << " " << green << " " << blue << std::endl;
    
    ++numpaths;
  }  
}

void TwoDSceneXMLParser::loadSceneDescriptionString( rapidxml::xml_node<>* node, std::string& description_string )
{
  assert( node != NULL );
  
  description_string = "No description specified.";
  
  // Attempt to locate the integrator node
  rapidxml::xml_node<>* nd = node->first_node("description");
  if( nd != NULL ) 
  {
    rapidxml::xml_attribute<>* typend = nd->first_attribute("text"); 
    if( typend == NULL ) 
    {
      std::cerr << "\033[31;1mERROR IN XMLSCENEPARSER:\033[m No text attribute specified for description. Exiting." << std::endl;
      exit(1);
    }
    description_string = typend->value();
  }
}



