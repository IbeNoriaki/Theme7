#include <Eigen/StdVector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include <tclap/CmdLine.h>

// TODO: Clean these up, we don't need them all anymore.
#include "TwoDScene.h"
#include "Force.h"
#include "SpringForce.h"
#include "ExplicitEuler.h"
#include "SemiImplicitEuler.h"
#include "TwoDimensionalDisplayController.h"
#include "TwoDSceneRenderer.h"
#include "TwoDSceneXMLParser.h"
#include "TwoDSceneSerializer.h"
#include "TwoDSceneGrader.h"
#include "StringUtilities.h"
#include "MathDefs.h"
#include "TimingUtilities.h"
#include "RenderingUtilities.h"
#include "TwoDSceneSVGRenderer.h"
#include "YImage.h"
#include "CollisionHandler.h"

#include "ExecutableSimulation.h"
#include "ParticleSimulation.h"


///////////////////////////////////////////////////////////////////////////////
//Adding the fourier transform shit that I need
char *fileName = "ellie.dat";
char fileBuf[100];
FILE *fp;
int sampleRate;
fftw_complex *in;
fftw_complex *out;
fftw_plan p;
int SIZE;
bool fileFinished = false;

///////////////////////////////////////////////////////////////////////////////
// Contains the actual simulation, renderer, parser, and serializer
ExecutableSimulation* g_executable_simulation;
TwoDScene* g_scene;

///////////////////////////////////////////////////////////////////////////////
// Rendering State
bool g_rendering_enabled = true;
double g_sec_per_frame;
double g_last_time = timingutils::seconds();

TwoDimensionalDisplayController g_display_controller(512,512);
renderingutils::Color g_bgcolor(1.0,1.0,1.0);


///////////////////////////////////////////////////////////////////////////////
// SVG Rendering State
bool g_svg_enabled = false;
std::string g_movie_dir;


///////////////////////////////////////////////////////////////////////////////
// Parser state
std::string g_xml_scene_file;
std::string g_description;
std::string g_scene_tag = "";


///////////////////////////////////////////////////////////////////////////////
// Scene input/output/comparison state
bool g_save_to_binary = false;
std::string g_binary_file_name;
std::ofstream g_binary_output;

bool g_simulate_comparison = false;
std::string g_comparison_file_name;
std::ifstream g_binary_input;


///////////////////////////////////////////////////////////////////////////////
// Simulation state
bool g_paused = true;
scalar g_dt = 0.0;
int g_num_steps = 0;
int g_current_step = 0;
bool g_simulation_ran_to_completion = false;



///////////////////////////////////////////////////////////////////////////////
// Simulation functions

void miscOutputCallback();
void sceneScriptingCallback();
void dumpPNG(const std::string &filename);

void stepSystem()
{
  assert( !(g_save_to_binary&&g_simulate_comparison) );

  // Determine if the simulation is complete
  if( g_current_step >= g_num_steps )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Simulation complete at time " << g_current_step*g_dt << ". Exiting." << std::endl;
    g_simulation_ran_to_completion = true;
    exit(0);
  }  

  // If comparing simulations, copy state of comparison scene to simulated scene
  if( g_simulate_comparison ) {
    g_executable_simulation->copyComparisonSceneToScene();
  }

  // Step the system forward in time
  g_executable_simulation->stepSystem(g_dt);
  g_current_step++;

  // If the user wants to save output to a binary
  if( g_save_to_binary ) 
    {
      g_executable_simulation->serializeScene(g_binary_output);
    }
  

  // Update the state of the renderers
  if( g_rendering_enabled ) g_executable_simulation->updateOpenGLRendererState();
  g_executable_simulation->updateSVGRendererState();

  // If comparing simulations, load comparison scene's equivalent step
  if( g_simulate_comparison ) g_executable_simulation->loadComparisonScene(g_binary_input);

  // If the user wants to generate a SVG movie
  if( g_svg_enabled )
  {
    // Generate a filename
    std::stringstream name;
    name << std::setfill('0');
    name << g_movie_dir << "/frame" << std::setw(20) << g_current_step << ".svg";    
    // Actually write the svg out
    g_executable_simulation->renderSceneSVG(name.str());    
  }  
  
  // If comparing simulations, compute the accumulated residual
  if( g_simulate_comparison ) g_executable_simulation->updateSceneComparison();
  
  // If the user wants to generate a PNG movie
  #ifdef PNGOUT
    std::stringstream oss;
    oss << "pngs/frame" << std::setw(5) << std::setfill('0') << g_current_step << ".png";
    dumpPNG(oss.str());
  #endif

  // Execute the user-customized output callback
  miscOutputCallback();

  // Execute the user-customized scripting callback
  sceneScriptingCallback();

}

void syncScene()
{
    if( g_save_to_binary ) 
    {
        g_executable_simulation->serializeScene(g_binary_output);
    }

    if( g_simulate_comparison )
    {
        g_executable_simulation->loadComparisonScene(g_binary_input);
        g_executable_simulation->copyComparisonSceneToScene();
    }
}

void headlessSimLoop()
{
  scalar nextpercent = 0.02;
  std::cout << outputmod::startpink << "Progress: " << outputmod::endpink;
  for( int i = 0; i < 50; ++i ) std::cout << "-";
  std::cout << std::endl;
  std::cout << "          ";
  while( true )
  {
    scalar percent_done = ((double)g_current_step)/((double)g_num_steps);
    if( percent_done >= nextpercent )
    {
      nextpercent += 0.02;
      std::cout << "." << std::flush;
    }
    stepSystem();
  }
}



///////////////////////////////////////////////////////////////////////////////
// Rendering and UI functions

void dumpPNG(const std::string &filename)
{
  #ifdef PNGOUT
    YImage image;
    image.resize(g_display_controller.getWindowWidth(), g_display_controller.getWindowHeight());

    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glReadBuffer(GL_BACK);

    glFinish();
    glReadPixels(0, 0, g_display_controller.getWindowWidth(), g_display_controller.getWindowHeight(), GL_RGBA, GL_UNSIGNED_BYTE, image.data());
    image.flip();

    image.save(filename.c_str());
  #endif
}

void reshape( int w, int h ) 
{
  g_display_controller.reshape(w,h);

  assert( renderingutils::checkGLErrors() );
}

// TODO: Move these functions to scene renderer?
void setOrthographicProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	gluOrtho2D(0, g_display_controller.getWindowWidth(), 0, g_display_controller.getWindowHeight());
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

  assert( renderingutils::checkGLErrors() );
}

void renderBitmapString( float x, float y, float z, void *font, std::string s ) 
{
	glRasterPos3f(x, y, z);
	for( std::string::iterator i = s.begin(); i != s.end(); ++i )
	{
		char c = *i;
		glutBitmapCharacter(font, c);
	}

  assert( renderingutils::checkGLErrors() );
}

void drawHUD()
{
  setOrthographicProjection();
  glColor3f(1.0-g_bgcolor.r,1.0-g_bgcolor.g,1.0-g_bgcolor.b);
  renderBitmapString( 4, g_display_controller.getWindowHeight()-20, 0.0, GLUT_BITMAP_HELVETICA_18, stringutils::convertToString(g_current_step*g_dt) ); 
  glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

  assert( renderingutils::checkGLErrors() );
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

  assert( g_executable_simulation != NULL );
  g_executable_simulation->renderSceneOpenGL();

  drawHUD();

  glutSwapBuffers();

  assert( renderingutils::checkGLErrors() );
}

void centerCamera()
{
  renderingutils::Viewport view;

  g_executable_simulation->computeCameraCenter(view);

  scalar ratio = ((scalar)g_display_controller.getWindowHeight())/((scalar)g_display_controller.getWindowWidth());

  view.size = 1.2*std::max(ratio*view.rx,view.ry);

  g_display_controller.setCenterX(view.cx);
  g_display_controller.setCenterY(view.cy);
  g_display_controller.setScaleFactor(view.size);
}

void keyboard( unsigned char key, int x, int y )
{
  g_display_controller.keyboard(key,x,y);

  if( key == 27 || key == 'q' )
  {
    exit(0);
  }
  else if( key == 's' || key =='S' )
  {
    stepSystem();
    glutPostRedisplay();
  }
  else if( key == ' ' )
  {
  //  int x = fork();
  //  if(x==0) {
  //    system("play thermo.dat");
  //    exit(1);
  //  }
    g_paused = !g_paused;
  }
  else if( key == 'c' || key == 'C' )
  {
    centerCamera();    
    g_display_controller.reshape(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
    glutPostRedisplay();
  }
  else if( key == 'i' || key == 'I' )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saving screenshot as 'output.svg'." << std::endl;
    g_executable_simulation->renderSceneSVG("output.svg");
  }

  assert( renderingutils::checkGLErrors() );
}

// Proccess 'special' keys
void special( int key, int x, int y )
{
  g_display_controller.special(key,x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void mouse( int button, int state, int x, int y )
{
  g_display_controller.mouse(button,state,x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void motion( int x, int y ) 
{
  g_display_controller.motion(x,y);
  
  assert( renderingutils::checkGLErrors() );
}

void idle()
{
  //std::cout << "g_last_time: " << g_last_time << std::endl;
  // Trigger the next timestep
  double current_time = timingutils::seconds();
  //std::cout << "current_time: " << current_time << std::endl;
  //std::cout << "g_sec_per_frame: " << g_sec_per_frame << std::endl;
  if( !g_paused && current_time-g_last_time >= g_sec_per_frame ) 
  {
    g_last_time = current_time;
    stepSystem();
    glutPostRedisplay();
  }
  
  assert( renderingutils::checkGLErrors() );
}

void initializeOpenGLandGLUT( int argc, char** argv )
{
  // Initialize GLUT
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
  glutInitWindowSize(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
  glutCreateWindow("Forty One Sixty Seven Sim");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutIdleFunc(idle);
  
  // Initialize OpenGL
	reshape(g_display_controller.getWindowWidth(),g_display_controller.getWindowHeight());
  glClearColor(g_bgcolor.r, g_bgcolor.g, g_bgcolor.b, 1.0);
  
  assert( renderingutils::checkGLErrors() );
}


///////////////////////////////////////////////////////////////////////////////
// Parser functions

void loadScene( const std::string& file_name )
{
  // Maximum time in the simulation to run for. This has nothing to do with run time, cpu time, etc. This is time in the 'virtual world'.
  scalar max_time;
  // Maximum frequency, in wall clock time, to execute the simulation for. This serves as a cap for simulations that run too fast to see a solution.
  scalar steps_per_sec_cap = 100.0;
  // Contains the center and 'scale factor' of the view
  renderingutils::Viewport view;

  // Load the simulation and pieces of rendring and UI state
  assert( g_executable_simulation == NULL );
  TwoDSceneXMLParser xml_scene_parser;
  xml_scene_parser.loadExecutableSimulation( file_name, g_simulate_comparison, g_rendering_enabled, g_display_controller, &g_executable_simulation,
                                             view, g_dt, max_time, steps_per_sec_cap, g_bgcolor, g_description, g_scene_tag );
  assert( g_executable_simulation != NULL );

  g_scene = xml_scene_parser.getScene();

  // If the user did not request a custom viewport, try to compute a reasonable default.
  if( view.size <= 0.0 )
  {
    centerCamera();
  }
  // Otherwise set the viewport per the user's request.
  else
  {
    g_display_controller.setCenterX(view.cx);
    g_display_controller.setCenterY(view.cy);
    g_display_controller.setScaleFactor(view.size);
  }

  // To cap the framerate, compute the minimum time a single timestep should take
  g_sec_per_frame = 1.0/steps_per_sec_cap;
  // Integer number of timesteps to take
  g_num_steps = ceil(max_time/g_dt);
  // We begin at the 0th timestep
  g_current_step = 0;  
}

void parseCommandLine( int argc, char** argv )
{
  try 
  {
    TCLAP::CmdLine cmd("Forty One Sixty Seven Sim");
    
    // XML scene file to load
    TCLAP::ValueArg<std::string> scene("s", "scene", "Simulation to run; an xml scene file", true, "", "string", cmd);
    
    // Begin the scene paused or running
    TCLAP::ValueArg<bool> paused("p", "paused", "Begin the simulation paused if 1, running if 0", false, true, "boolean", cmd);
    
    // Run the simulation with rendering enabled or disabled
    TCLAP::ValueArg<bool> display("d", "display", "Run the simulation with display enabled if 1, without if 0", false, true, "boolean", cmd);
    
    // These cannot be set at the same time
    // File to save output to
    TCLAP::ValueArg<std::string> output("o", "outputfile", "Binary file to save simulation state to", false, "", "string", cmd);
    // File to load for comparisons
    TCLAP::ValueArg<std::string> input("i", "inputfile", "Binary file to load simulation state from for comparison", false, "", "string", cmd);
    
    // Save svgs to a movie directory
    TCLAP::ValueArg<std::string> movie("m", "moviedir", "Directory to output svg screenshot to", false, "", "string", cmd);
    
    cmd.parse(argc, argv);
    
    if( output.isSet() && input.isSet() ) throw TCLAP::ArgException( "arguments i and o specified simultaneously", "arguments i and o", "invalid argument combination" );
    
    assert( scene.isSet() );
    g_xml_scene_file = scene.getValue();
    g_paused = paused.getValue();
    g_rendering_enabled = display.getValue();
    
    if( output.isSet() )
    {
      g_save_to_binary = true;
      g_binary_file_name = output.getValue();
    }
    
    if( input.isSet() )
    {
      g_simulate_comparison = true;
      g_comparison_file_name = input.getValue();
    }

    if( movie.isSet() )
    {
      g_svg_enabled = true;
      g_movie_dir = movie.getValue();
    }
  } 
  catch (TCLAP::ArgException& e) 
  {
    std::cerr << "error: " << e.what() << std::endl;
    exit(1);
  }
}



///////////////////////////////////////////////////////////////////////////////
// Various support functions

void miscOutputFinalization();

void cleanupAtExit()
{
  if( g_binary_output.is_open() )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Saved simulation to file '" << g_binary_file_name << "'." << std::endl;
    g_binary_output.close();
  }

  if( g_binary_input.is_open() )
  {
    std::cout << outputmod::startpink << "FOSSSim message: " << outputmod::endpink << "Colsing benchmarked simulation file '" << g_comparison_file_name << "'." << std::endl;
  }

  if( g_executable_simulation != NULL && g_simulate_comparison ) g_executable_simulation->printErrorInformation( g_simulation_ran_to_completion );

  miscOutputFinalization();

  if( g_executable_simulation != NULL )
  {
    delete g_executable_simulation;
    g_executable_simulation = NULL;
  }  
}

std::ostream& fosssim_header( std::ostream& stream )
{
  stream << outputmod::startgreen << 
  "------------------------------------------    " << std::endl <<
  "  _____ ___  ____ ____ ____  _                " << std::endl <<
  " |  ___/ _ \\/ ___/ ___/ ___|(_)_ __ ___      " << std::endl <<
  " | |_ | | | \\___ \\___ \\___ \\| | '_ ` _ \\ " << std::endl <<
  " |  _|| |_| |___) |__) |__) | | | | | | |     " << std::endl << 
  " |_|   \\___/|____/____/____/|_|_| |_| |_|    " << std::endl <<
  "------------------------------------------    " 
  << outputmod::endgreen << std::endl;
  
  return stream;
}

std::ofstream g_debugoutput;

void miscOutputInitialization()
{
  //g_debugoutput.open("debugoutput.txt");
  //g_debugoutput << "# Time   PotentialEnergy   KineticEnergy   TotalEnergy" << std::endl;
  //g_debugoutput << g_current_step*g_dt << "\t" << g_scene.computePotentialEnergy() << "\t" << g_scene.computeKineticEnergy() << "\t" << g_scene.computeTotalEnergy() << std::endl;
}

void miscOutputCallback()
{
  //g_debugoutput << g_current_step*g_dt << "\t" << g_scene.computePotentialEnergy() << "\t" << g_scene.computeKineticEnergy() << "\t" << g_scene.computeTotalEnergy() << std::endl;
}

void miscOutputFinalization()
{
  //g_debugoutput.close();
}


double scalingFn(double x) {
  return (5.0)*x/(50.0);
}

// Called at the end of each timestep. Intended for adding effects to creative scenes.
void sceneScriptingCallback()
{
  //The fourier transform shit goes here
  int i = 0;
  double x;
  char *tempBuf;
  double tx, amplitude;
  int tempVal = 0;

  for(i = 0; i<SIZE; i++) {
    tempBuf = fgets(fileBuf, 100, fp);
    if(tempBuf) {
      sscanf(fileBuf, "%*[ ]%lf%*[ ]%lf", &tx, &amplitude);
//      printf("Double: count: %d %lf, %lf\n", i, tx, amplitude);
      in[i][0] = amplitude;
      in[i][1] = 0;
    } else {
      fileFinished=true;
      break;
    }
  }
  if(!fileFinished) {
    p = fftw_plan_dft_1d(i, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p);
    tempVal = i;
  
      for(i = 0; i < tempVal; i++) {
            x = sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
        //    printf("Fourier: count: %d\t%f\n", i, x);
    }
    fftw_destroy_plan(p);
  }


  double buckets[10];
  int temp = (SIZE/2)/10;
  for(int a = 0; a < 10; a++) {
    double avg = 0;
    double t = (temp*a+temp<(SIZE/2))?temp*a+temp:(SIZE/2);
    for(int b = temp*a; b<t; b++) {
      x = sqrt(out[b][0] * out[b][0] + out[b][1] * out[b][1]);
      avg+=x;
    }
    avg/=temp;
    buckets[a] = avg;
  //  std::cout << "Printing da buckets " << buckets[a] << std::endl;
  }
  if(g_scene_tag == "Visualizer") {
     // Get the particle tags
    const std::vector<std::string>& tags = (*g_scene).getParticleTags();
    // Get the particle positions
    VectorXs& x = (*g_scene).getX();
    // Get the particle velocities
    VectorXs& v = (*g_scene).getV();

    for(int i = 0; i < tags.size(); i++) {
    	//v[2*i+1] = 1;
      std::string tempTag = tags[i];      //Parsing the tag:
      std::size_t found = tempTag.find("mobile");
      if(found!=std::string::npos) {
        int len = tempTag.length();
        int num = tempTag[len - 1] - '0';
        if(buckets[num]<=1) {
          x[2*i] = 100;
          x[2*i+1] = 100;
        } else {
          x[2*i] = -2 + 0.5*num;
          x[2*i+1] = -3;
        }
      }
      found = tempTag.find("spring");
      if(found!=std::string::npos) {
        int len = tempTag.length();
        int num = tempTag[len - 1] - '0';
        x[2*i+1]=scalingFn(buckets[num]);
      }
    /*
      std::string tempTag = tags[i];
      std::size_t found = tempTag.find("bucket");
      if(found!=std::string::npos) {
        int len = tempTag.length();
        int num = tempTag[len-1]-'0';
        if(v[2*i]>=buckets[num] || v[2*i+1]>=buckets[num]) {
          v[2*i] -= buckets[num]*(rand()%10);
          v[2*i+1] -= buckets[num]*(rand()%10);
        } else {
          v[2*i] += buckets[num]*(rand()%10);
          v[2*i+1] += buckets[num]*(rand()%10);
        }
      }
    */
    }
  }   
  memset(in, 0, SIZE*sizeof(fftw_complex) );
  memset(out, 0, SIZE*sizeof(fftw_complex) );
}
int main( int argc, char** argv )
{
  // Parse command line arguments
  parseCommandLine( argc, argv );
 
  assert( !(g_save_to_binary && g_simulate_comparison) );
  
  // Function to cleanup at progarm exit
  atexit(cleanupAtExit);


  // Load the user-specified scene
  loadScene(g_xml_scene_file);

  //Adding the file loading shit
  fp = fopen(fileName, "r");
  if(!fp) {
    std::cout << "Music file not found\n";
    return 0;
  }
  fscanf(fp, "; Sample Rate %d", &sampleRate);
  std::cout << "Sample rate of music: " << sampleRate << std::endl;

  SIZE = sampleRate * g_dt;
  SIZE+=1;

  //SIZE = (SIZE>MAXSAMPLERATE)?MAXSAMPLERATE:SIZE;

  std::cout << "Sample size of fft: " << SIZE << std::endl;

  fgets(fileBuf, 100, fp);
  fgets(fileBuf, 100, fp);

  in = (fftw_complex *)fftw_malloc(SIZE*sizeof(fftw_complex));
  out = (fftw_complex *)fftw_malloc(SIZE*sizeof(fftw_complex));

  // If requested, open the binary output file
  if( g_save_to_binary )
  {
    // Attempt to open the binary
    g_binary_output.open(g_binary_file_name.c_str());
    if( g_binary_output.fail() ) 
    {
      std::cerr << outputmod::startred << "ERROR IN INITIALIZATION: "  << outputmod::endred << "Failed to open binary output file: " << " `" << g_binary_file_name << "`   Exiting." << std::endl;
      exit(1);
    }
    // Save the initial conditions
    g_executable_simulation->serializeScene(g_binary_output);
  }
  // If requested, open the input file for the scene to benchmark
  else if( g_simulate_comparison )
  {
    // Attempt to open the binary
    g_binary_input.open(g_comparison_file_name.c_str());
    if( g_binary_input.fail() ) 
    {
      std::cerr << outputmod::startred << "ERROR IN INITIALIZATION: "  << outputmod::endred << "Failed to open binary input file: " << " `" << argv[3] << "`   Exiting." << std::endl;
      exit(1);
    }
    assert( g_executable_simulation != NULL );
    g_executable_simulation->loadComparisonScene(g_binary_input);
  }  

  // Initialization for OpenGL and GLUT
  if( g_rendering_enabled ) initializeOpenGLandGLUT(argc,argv);

  // Print a header
  std::cout << fosssim_header << std::endl;

  // Print some status info about this FOSSSim build
  #ifdef FOSSSIM_VERSION
    std::cout << outputmod::startblue << "FOSSSim Version: "  << outputmod::endblue << FOSSSIM_VERSION << std::endl;
  #endif
  #ifdef CMAKE_BUILD_TYPE
    std::cout << outputmod::startblue << "Build type: " << outputmod::endblue << CMAKE_BUILD_TYPE << std::endl;
  #endif
  #ifdef EIGEN_VECTORIZE
    std::cout << outputmod::startblue << "Vectorization: " << outputmod::endblue << "Enabled" << std::endl;
  #else
    std::cout << outputmod::startblue << "Vectorization: " << outputmod::endblue << "Disabled" << std::endl;
  #endif
  
  std::cout << outputmod::startblue << "Scene: " << outputmod::endblue << g_xml_scene_file << std::endl;
  std::cout << outputmod::startblue << "Integrator: " << outputmod::endblue << g_executable_simulation->getSolverName() << std::endl;
  std::cout << outputmod::startblue << "Collision Handling: " << outputmod::endblue << g_executable_simulation->getCollisionHandlerName() << std::endl;
  std::cout << outputmod::startblue << "Description: " << outputmod::endblue << g_description << std::endl;

  if( g_save_to_binary ) std::cout << outputmod::startpink << "FOSSSim message: "  << outputmod::endpink << "Saving simulation to: " << g_binary_file_name << std::endl;
  if( g_simulate_comparison ) std::cout << outputmod::startpink << "FOSSSim message: "  << outputmod::endpink << "Benchmarking simulation in: " << g_comparison_file_name << std::endl;

  miscOutputInitialization();

  if( g_rendering_enabled ) glutMainLoop();
  else headlessSimLoop();

  fclose(fp);

  return 0;
}
