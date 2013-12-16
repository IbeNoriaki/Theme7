#include "TwoDSceneRenderer.h"
#include "TwoDimensionalDisplayController.h"

TwoDSceneRenderer::TwoDSceneRenderer( const TwoDScene& scene, const TwoDimensionalDisplayController &dc, 
                                      const std::vector<renderingutils::Color>& particle_colors, const std::vector<renderingutils::Color>& edge_colors, 
                                      const std::vector<renderingutils::Color> &halfplane_colors, const std::vector<renderingutils::ParticlePath>& particle_paths )
: m_dc(dc)
// Particle system rendering state
, m_particle_colors(particle_colors)
, m_edge_colors(edge_colors)
, m_halfplane_colors(halfplane_colors)
, m_particle_paths(particle_paths)
// Precomputed rendering state
, m_circle_points()
, m_semi_circle_points()
{
  initializeCircleRenderer(16);
  initializeSemiCircleRenderer(16);
}

TwoDSceneRenderer::TwoDSceneRenderer( const TwoDimensionalDisplayController& dc )
: m_dc(dc)
{
  initializeCircleRenderer(16);
  initializeSemiCircleRenderer(16);
}

void TwoDSceneRenderer::initializeCircleRenderer( int num_points )
{
  m_circle_points.resize(num_points);
  double dtheta = 2.0*PI/((double)num_points);
  for( int i = 0; i < num_points; ++i )
  {
    m_circle_points[i].first =  cos(((double)i)*dtheta);
    m_circle_points[i].second = sin(((double)i)*dtheta);
  }
}

void TwoDSceneRenderer::initializeSemiCircleRenderer( int num_points )
{
  double dtheta = PI/((double)(num_points-1));
  m_semi_circle_points.resize(num_points);
  for( int i = 0; i < num_points; ++i )
  {
    m_semi_circle_points[i].first =  -sin(((double)i)*dtheta);
    m_semi_circle_points[i].second = cos(((double)i)*dtheta);
  }  
}

void TwoDSceneRenderer::renderRigdBody( const RigidBody& rb ) const
{
  glColor3d(0.0,0.0,0.0);
  //glBegin(GL_LINE_LOOP);
  for( int i = 0; i < rb.getNumVertices(); ++i )
  {
    Vector2s vrt0 = rb.getWorldSpaceVertex(i);
    Vector2s vrt1 = rb.getWorldSpaceVertex((i+1)%rb.getNumVertices());
    renderSweptEdge( vrt0, vrt1, rb.getRadius() );
  }
}

void TwoDSceneRenderer::renderRigdBodySimulation( const RigidBodyScene& scene )
{
  const std::vector<RigidBody>& rbs = scene.getRigidBodies();
  // Render each rigid body
  for( std::vector<RigidBody>::size_type i = 0; i < rbs.size(); ++i ) renderRigdBody( rbs[i] );  
}

void TwoDSceneRenderer::updateParticleSimulationState( const TwoDScene& scene )
{
  const VectorXs& x = scene.getX();
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
    m_particle_paths[i].addToPath(x.segment<2>(2*m_particle_paths[i].getParticleIdx()));
}


void TwoDSceneRenderer::renderSolidCircle( const Eigen::Vector2d& center, double radius ) const
{  
	glBegin(GL_TRIANGLE_FAN);
    glVertex2d(center.x(),center.y());
    
    for( std::vector<std::pair<double,double> >::size_type i = 0; i < m_circle_points.size(); ++i )
      glVertex2d(radius*m_circle_points[i].first+center.x(), radius*m_circle_points[i].second+center.y());

    glVertex2d(radius*m_circle_points.front().first+center.x(), radius*m_circle_points.front().second+center.y());  
	glEnd();  
}

void TwoDSceneRenderer::renderCircle( const Eigen::Vector2d& center, double radius ) const
{
  glBegin(GL_LINE_LOOP);
    for( std::vector<Eigen::Vector2d>::size_type i = 0; i < m_circle_points.size(); ++i )
      glVertex2d(radius*m_circle_points[i].first+center.x(), radius*m_circle_points[i].second+center.y());
  glEnd();
}

void TwoDSceneRenderer::renderImpulse( const TwoDScene &scene, const CollisionInfo &impulse, bool buggy) const
{
  assert(impulse.m_idx1 < scene.getNumParticles());
  buggy ? glColor3d(1,0,0) : glColor3d(0,1,0);

  glBegin(GL_LINES);
  double x = scene.getX()[2*impulse.m_idx1];
  double y = scene.getX()[2*impulse.m_idx1+1];
  glVertex2d(x,y);
  glVertex2d(x + impulse.m_n[0], y+impulse.m_n[1]);
  glEnd();
}

void TwoDSceneRenderer::renderSweptEdge( const Eigen::Vector2d& x0, const Eigen::Vector2d& x1, double radius ) const
{
  Eigen::Vector2d e = x1-x0;
  double length = e.norm();
  double theta = 360.0*atan2(e.y(),e.x())/(2.0*PI);

  glPushMatrix();
  
  glTranslated(x0.x(), x0.y(),0.0);
  glRotated(theta, 0.0, 0.0, 1.0);
  
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(0.0,0.0);
  for( std::vector<Eigen::Vector2d>::size_type i = 0; i < m_semi_circle_points.size(); ++i )
    glVertex2d(radius*m_semi_circle_points[i].first, radius*m_semi_circle_points[i].second);
  for( std::vector<Eigen::Vector2d>::size_type i = 0; i < m_semi_circle_points.size(); ++i )
    glVertex2d(-radius*m_semi_circle_points[i].first+length, -radius*m_semi_circle_points[i].second);
  glVertex2d(radius*m_semi_circle_points.front().first, radius*m_semi_circle_points.front().second);
	glEnd();  
  
  glPopMatrix();
}

void TwoDSceneRenderer::renderHalfplane( const VectorXs &x, const VectorXs &n) const
{
  glPushMatrix();
  double theta = -360.0*atan2(n[0], n[1])/(2.0*PI);

  glTranslated(x[0], x[1], 0);
  glRotated(theta, 0, 0, 1.0);

  double cx = m_dc.getCenterX();
  double cy = m_dc.getCenterY();

  double sqdist = (cx-x[0])*(cx-x[0]) + (cy-x[1])*(cy-x[1]);

  double w = m_dc.getWorldWidth();
  double h = m_dc.getWorldHeight();

  glBegin(GL_TRIANGLES);
  glVertex4d(0,0,0,1.0);
  glVertex4d(1.0, 0,0,0);
  glVertex4d(0,-1.0,0,0);
  glVertex4d(0,0,0,1.0);
  glVertex4d(-1.0,0,0,0);
  glVertex4d(0,-1.0,0,0);
  glEnd();

  glPopMatrix();
}

void TwoDSceneRenderer::renderParticleSimulation( const TwoDScene& scene ) const
{
  const VectorXs& x = scene.getX();
  assert( x.size()%2 == 0 );
  assert( 2*scene.getNumParticles() == x.size() );

  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
  {
    const std::list<Vector2s>& ppath = m_particle_paths[i].getPath();
    const renderingutils::Color& pathcolor = m_particle_paths[i].getColor();
    glColor3d(pathcolor.r,pathcolor.g,pathcolor.b);
    glBegin(GL_LINE_STRIP);
    for( std::list<Vector2s>::const_iterator itr = ppath.begin(); itr != ppath.end(); ++itr )
    {
      glVertex2d(itr->x(),itr->y());
    }
    glEnd();
  }
  
  // Render edges
  const std::vector<std::pair<int,int> >& edges = scene.getEdges();
  const std::vector<scalar>& edgeradii = scene.getEdgeRadii();
  assert( edgeradii.size() == edges.size() );
  for( std::vector<std::pair<int,int> >::size_type i = 0; i < edges.size(); ++i )
  {
    assert( edges[i].first >= 0 );  assert( edges[i].first < scene.getNumParticles() );
    assert( edges[i].second >= 0 ); assert( edges[i].second < scene.getNumParticles() );
    glColor3d(m_edge_colors[i].r,m_edge_colors[i].g,m_edge_colors[i].b);
    renderSweptEdge( x.segment<2>(2*edges[i].first), x.segment<2>(2*edges[i].second), edgeradii[i] );
  }

  // Render halfplanes
  const std::vector<std::pair<VectorXs, VectorXs> > &halfplanes = scene.getHalfplanes();
  for(int i=0; i< (int)halfplanes.size(); i++)
    {
      glColor3d(m_halfplane_colors[i].r,m_halfplane_colors[i].g,m_halfplane_colors[i].b);
      renderHalfplane(halfplanes[i].first, halfplanes[i].second);
    }

  // Render particles
  const std::vector<scalar>& radii = scene.getRadii();
  assert( (int) radii.size() == scene.getNumParticles() );
  for( int i = 0; i < scene.getNumParticles(); ++i ) 
  {
    glColor3d(m_particle_colors[i].r,m_particle_colors[i].g,m_particle_colors[i].b);
    renderSolidCircle( x.segment<2>(2*i), radii[i] );
  }  
}

void TwoDSceneRenderer::circleMajorParticleSimulationResiduals( const TwoDScene& oracle_scene, const TwoDScene& testing_scene, const std::vector<CollisionInfo> *impulses, const std::vector<CollisionInfo> *otherimpulses, scalar eps ) const
{
  assert(   oracle_scene.getNumParticles() == testing_scene.getNumParticles() );
  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getX().size() );
  assert( 2*oracle_scene.getNumParticles() == testing_scene.getX().size() );
  assert( 2*oracle_scene.getNumParticles() == oracle_scene.getV().size() );
  assert( 2*oracle_scene.getNumParticles() == testing_scene.getV().size() );

  const VectorXs& oracle_x = oracle_scene.getX();
  const VectorXs& testing_x = testing_scene.getX();

  const VectorXs& oracle_v = oracle_scene.getV();
  const VectorXs& testing_v = testing_scene.getV();

  glColor3d(1.0,0.0,0.0);
  for( int i = 0; i < oracle_scene.getNumParticles(); ++i )
  {
    scalar x_resid = (oracle_x.segment<2>(2*i)-testing_x.segment<2>(2*i)).norm();
    scalar v_resid = (oracle_v.segment<2>(2*i)-testing_v.segment<2>(2*i)).norm();
    if( x_resid > eps || v_resid > eps )
      renderCircle( oracle_x.segment<2>(2*i), 2.0*oracle_scene.getRadius(i) );
  }


  if(impulses)
    {

      int i=0, j=0;
      
      // loop over the real impulses
      while(i < (int)impulses->size())
	{
	  int curvert = (*impulses)[i].m_idx1;
	  CollisionInfo::collisiontype curtype = (*impulses)[i].m_type;
	  int curidx2 = (*impulses)[i].m_idx2;
	  
	  // all student impulses less than this correct impulse are buggy
	  while(j < (int)otherimpulses->size()
		&& (*otherimpulses)[j].m_idx1 < curvert
		&& (*otherimpulses)[j].m_type < curtype
		&& (*otherimpulses)[j].m_idx2 < curidx2)
	    {
	      renderImpulse( testing_scene, (*otherimpulses)[j], true);
	      j++;
	    }

	  // check for missed collision
	  if( ! (j < (int)otherimpulses->size()
		 && (*otherimpulses)[j].m_idx1 == curvert
		 && (*otherimpulses)[j].m_type == curtype
		 && (*otherimpulses)[j].m_idx2 == curidx2))
	    {
	      renderImpulse( testing_scene, (*impulses)[i], false);
	    }
	  else
	    {
	      // check for buggy normal
	      if( ((*otherimpulses)[j].m_n - (*impulses)[i].m_n).norm() > eps)
		{
		  renderImpulse( testing_scene, (*impulses)[i], false);
		  renderImpulse( testing_scene, (*otherimpulses)[j], true);
		}
	      j++;
	    }
	  
	  i++;
	}
      // Any remaining student impulses are buggy
      while(j < (int)otherimpulses->size())
	{
	  renderImpulse( testing_scene, (*otherimpulses)[j], true);
	  j++;
	}
    }
  
}

std::vector<renderingutils::Color>& TwoDSceneRenderer::getParticleColors()
{
  return m_particle_colors;
}

const std::vector<renderingutils::Color>& TwoDSceneRenderer::getParticleColors() const
{
  return m_particle_colors;
}

