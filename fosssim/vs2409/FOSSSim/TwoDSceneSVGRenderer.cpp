#include "TwoDSceneSVGRenderer.h"
#include "TwoDimensionalDisplayController.h"

TwoDSceneSVGRenderer::TwoDSceneSVGRenderer( const TwoDScene& scene, const TwoDimensionalDisplayController &dc, const std::vector<renderingutils::Color>& particle_colors, const std::vector<renderingutils::Color>& edge_colors, const std::vector<renderingutils::Color> &halfplane_colors, const std::vector<renderingutils::ParticlePath>& particle_paths, int imagewidth, int imageheight, const renderingutils::Color& bgcolor )
: m_scene(scene)
, m_dc(dc)
, m_w(imagewidth)
, m_h(imageheight)
, m_bgcolor(bgcolor)
, m_particle_colors(particle_colors)
, m_edge_colors(edge_colors)
, m_halfplane_colors(halfplane_colors)
, m_particle_paths(particle_paths)
, m_circle_points()
, m_semi_circle_points()
{
  assert( (int) m_particle_colors.size() == m_scene.getNumParticles() );
  assert( (int) m_edge_colors.size() == m_scene.getNumEdges() );
}

void TwoDSceneSVGRenderer::updateState()
{
  const VectorXs& x = m_scene.getX();
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
    m_particle_paths[i].addToPath(x.segment<2>(2*m_particle_paths[i].getParticleIdx()));
}

std::string TwoDSceneSVGRenderer::intToHexString( int integer ) const
{
  std::stringstream newstring;
  newstring << std::hex << std::setfill('0') << std::setw(2) << integer;
  return newstring.str();
}

void TwoDSceneSVGRenderer::computeSimToImageMap(scalar& scale, scalar& xmin, scalar& ymin, scalar& xshift, scalar& yshift ) const
{
  scalar xwidth = m_dc.getWorldWidth();
  scalar ywidth = m_dc.getWorldHeight();

  xmin = m_dc.getCenterX()-xwidth/2.0;
  ymin = m_dc.getCenterY()-ywidth/2.0;

  scale = m_w/(2.0*m_dc.getScaleFactor());
  scalar remainderx = ((scalar)m_w) - scale*xwidth;
  scalar remaindery = ((scalar)m_h) - scale*ywidth;

  xshift = 0.5*remainderx;
  yshift = 0.5*remaindery;
}

void TwoDSceneSVGRenderer::renderSolidCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const
{  
  file << "<circle fill=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\"" << " stroke=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\"" << " stroke-width=\"0\" cx=\"";
  file << center.x();
  file << "\" cy=\"";
  file << center.y();
  file << "\" r=\"";
  file << r;
  file << "\"/>" << std::endl;    
}

void TwoDSceneSVGRenderer::renderCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const
{
  file << "<circle cx=\"" << center.x() << "\" cy=\"" << center.y() << "\" r=\"" << r << "\" stroke=\"#";
  file << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "\" stroke-width=\"1\" fill=\"none\"/>" << std::endl;
}

void TwoDSceneSVGRenderer::renderSweptEdge( std::fstream& file, const Vector2s& x0, const Vector2s& x1, const scalar& r, const renderingutils::Color& color ) const
{
  scalar theta = atan2(x1.y()-x0.y(),x1.x()-x0.x());

  scalar rx = -r*sin(theta);
  scalar ry =  r*cos(theta);

  scalar p0x = x0.x() + rx;
  scalar p0y = x0.y() + ry;
  scalar p1x = x0.x() - rx;
  scalar p1y = x0.y() - ry;
  scalar p2x = x1.x() - rx;
  scalar p2y = x1.y() - ry;
  scalar p3x = x1.x() + rx;
  scalar p3y = x1.y() + ry;

  file << "<polygon points=\"" << p0x << "," << p0y << " " << p1x << "," << p1y << " " << p2x << "," << p2y << " " << p3x << "," << p3y;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;

  renderSolidCircle( file, x0, r, color );
  renderSolidCircle( file, x1, r, color );
}

void TwoDSceneSVGRenderer::renderImpulse( std::fstream &file, const TwoDScene &scene, const CollisionInfo &impulse, bool buggy) const
{
  scalar scale, xmin, ymin, xshift, yshift;
  computeSimToImageMap(scale, xmin, ymin, xshift, yshift );
  
  std::string color = (buggy ? "#FF0000" : "#00FF00");
  assert(impulse.m_idx1 < scene.getNumParticles());
  double x = scene.getX()[2*impulse.m_idx1];
  double y = scene.getX()[2*impulse.m_idx1+1];

  double x2 = x + impulse.m_n[0];
  double y2 = y + impulse.m_n[1];

  x = scale*(x-xmin)+xshift;
  y = m_h-(scale*(y-ymin)+yshift);

  x2 = scale*(x2-xmin)+xshift;
  y2 = m_h-(scale*(y2-ymin)+yshift);

  file << "<line x1=\"" << x << "\" y1=\"" << y << "\" x2=\"" << x2 << "\" y2=\"" << y2 << "\" style=\"stroke:" << color << ";stroke-width:2\"/>" << std::endl;
}

void TwoDSceneSVGRenderer::renderHalfplane( std::fstream &file, const VectorXs &x, const VectorXs &n, const renderingutils::Color &color) const
{
  double p0x = x[0] - 1000*n[1];
  double p0y = x[1] + 1000*n[0];
  double cx = x[0] - 1000*n[0];
  double cy = x[1] - 1000*n[1];
  double p1x = x[0] + 1000*n[1];
  double p1y = x[1] - 1000*n[0];
  
  file << "<polygon points=\"" << p0x << "," << p0y << " " << cx << "," << cy << " " << p1x << "," << p1y <<  "\" style=\"fill:#" << intToHexString(floor(255.0*color.r+0.5)) << intToHexString(floor(255.0*color.g+0.5)) << intToHexString(floor(255.0*color.b+0.5)) << "; stroke:#000000;stroke-width:0\"/>" << std::endl;
}

void TwoDSceneSVGRenderer::renderShared( std::fstream& file, const VectorXs& x, const std::vector<std::pair<int,int> >& edges, const std::vector<scalar>& radii, const std::vector<scalar>& edgeradii, const scalar& scale, const scalar& xmin, const scalar& ymin, const scalar& xshift, const scalar& yshift  ) const
{
  int numparticles = x.size()/2;
  
  for( std::vector<renderingutils::ParticlePath>::size_type i = 0; i < m_particle_paths.size(); ++i )
  {
    const std::list<Vector2s>& ppath = m_particle_paths[i].getPath();
    const renderingutils::Color& pathcolor = m_particle_paths[i].getColor();
    
    file << "<polyline points=\"";
    
    for( std::list<Vector2s>::const_iterator itr = ppath.begin(); itr != ppath.end(); ++itr )
    {
      Vector2s point;
      point << scale*(itr->x()-xmin) + xshift, ((scalar)m_h) - scale*(itr->y()-ymin) - yshift;
      file << point.x() << "," << point.y() << " ";
    }
    
    file << "\" style=\"fill:none;stroke:#";
    file << intToHexString(floor(255.0*pathcolor.r+0.5)) << intToHexString(floor(255.0*pathcolor.g+0.5)) << intToHexString(floor(255.0*pathcolor.b+0.5));
    file << ";stroke-width:1\"/>" << std::endl;
  }  
  
  // Render edges
  assert( edgeradii.size() == edges.size() );
  for( std::vector<std::pair<int,int> >::size_type i = 0; i < edges.size(); ++i )
  {
    assert( edges[i].first >= 0 );  assert( edges[i].first < m_scene.getNumParticles() );
    assert( edges[i].second >= 0 ); assert( edges[i].second < m_scene.getNumParticles() );
    
    int i0 = edges[i].first;
    int i1 = edges[i].second;
    
    Vector2s p0;
    p0 << scale*(x(2*i0)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i0+1)-ymin) - yshift;
    
    Vector2s p1;
    p1 << scale*(x(2*i1)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i1+1)-ymin) - yshift;
    
    renderSweptEdge( file, p0, p1, scale*edgeradii[i], m_edge_colors[i] );
  }

  // Render halfplanes
  for(int i=0; i < m_scene.getNumHalfplanes(); i++)
    {
      Vector2s p0;
      p0 << scale * (m_scene.getHalfplane(i).first[0]-xmin) + xshift, ((scalar)m_h) - scale*(m_scene.getHalfplane(i).first[1]-ymin) - yshift;
      Vector2s n0;
      n0 << m_scene.getHalfplane(i).second[0], -m_scene.getHalfplane(i).second[1];
      n0.normalize();
      renderHalfplane(file, p0, n0, m_halfplane_colors[i]);
    }
  
  // Render particles
  for( int i = 0; i < numparticles; ++i )
  {
    Vector2s center;
    center << scale*(x(2*i)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i+1)-ymin) - yshift;
    renderSolidCircle( file, center, scale*radii[i], m_particle_colors[i] );
  }  
}


void TwoDSceneSVGRenderer::renderScene( const std::string& filename ) const
{
  const VectorXs& x = m_scene.getX();
  assert( x.size()%2 == 0 );
  assert( 2*m_scene.getNumParticles() == x.size() );
  int numparticles = x.size()/2;
  const std::vector<scalar>& radii = m_scene.getRadii();
  assert( numparticles == (int) radii.size() );
  
  std::fstream file(filename.c_str(), std::fstream::out);
  if(!file)
  {
    std::cerr << "Failure writing SVG file!" << std::endl;
    exit(1);
  }
  
  scalar scale, xmin, ymin, xshift, yshift;
  computeSimToImageMap( scale, xmin, ymin, xshift, yshift );
  
  file << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <!-- Generator: Adobe Illustrator 13.0.0, SVG Export Plug-In . SVG Version: 6.00 Build 14948)  --> <svg version=\"1.2\" baseProfile=\"tiny\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" width=\"";
  file << m_w;
  file << "px\" height=\"";
  file << m_h;
  file << "px\" viewBox=\"0 0 ";
  file << m_w << " " << m_h;
  file << "\" xml:space=\"preserve\">" << std::endl;

  // Simulate a background color by rendering a large colored quad
  file << "<polygon points=\"" << 0 << "," << 0 << " " << m_w << "," << 0 << " " << m_w << "," << m_h << " " << 0 << "," << m_h;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*m_bgcolor.r+0.5)) << intToHexString(floor(255.0*m_bgcolor.g+0.5)) << intToHexString(floor(255.0*m_bgcolor.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;  
  
  
  const std::vector<std::pair<int,int> >& edges = m_scene.getEdges();
  const std::vector<scalar>& edgeradii = m_scene.getEdgeRadii();
  renderShared( file, x, edges, radii, edgeradii, scale, xmin, ymin, xshift, yshift );

  file << "</svg>" << std::endl;
  
  file.close();
}

void TwoDSceneSVGRenderer::renderComparisonScene( const std::string& filename, const TwoDScene& otherscene, const std::vector<CollisionInfo> *impulses, const std::vector<CollisionInfo> *otherimpulses, const scalar &eps) const
{
  const VectorXs& x = m_scene.getX();
  const VectorXs& v = m_scene.getV();
  assert( x.size()%2 == 0 );
  assert( 2*m_scene.getNumParticles() == x.size() );
  int numparticles = x.size()/2;
  const std::vector<scalar>& radii = m_scene.getRadii();
  assert( numparticles == (int) radii.size() );
  
  std::fstream file(filename.c_str(), std::fstream::out);
  if(!file)
  {
    std::cerr << "Failure writing SVG file!" << std::endl;
    exit(1);
  }
  
  scalar scale, xmin, ymin, xshift, yshift;
  computeSimToImageMap(scale, xmin, ymin, xshift, yshift );
  
  file << "<?xml version=\"1.0\" encoding=\"utf-8\"?> <!-- Generator: Adobe Illustrator 13.0.0, SVG Export Plug-In . SVG Version: 6.00 Build 14948)  --> <svg version=\"1.2\" baseProfile=\"tiny\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" width=\"";
  file << m_w;
  file << "px\" height=\"";
  file << m_h;
  file << "px\" viewBox=\"0 0 ";
  file << m_w << " " << m_h;
  file << "\" xml:space=\"preserve\">" << std::endl;
  
  // Simulate a background color by rendering a large colored quad
  file << "<polygon points=\"" << 0 << "," << 0 << " " << m_w << "," << 0 << " " << m_w << "," << m_h << " " << 0 << "," << m_h;
  file << "\" style=\"fill:#" << intToHexString(floor(255.0*m_bgcolor.r+0.5)) << intToHexString(floor(255.0*m_bgcolor.g+0.5)) << intToHexString(floor(255.0*m_bgcolor.b+0.5));
  file << "; stroke:#000000;stroke-width:0\"/>" << std::endl;
  
  const std::vector<std::pair<int,int> >& edges = m_scene.getEdges();
  const std::vector<scalar>& edgeradii = m_scene.getEdgeRadii();
  renderShared( file, x, edges, radii, edgeradii, scale, xmin, ymin, xshift, yshift );
  
  
  
  const VectorXs& otherx = otherscene.getX();  
  const VectorXs& otherv = otherscene.getV();

  for( int i = 0; i < numparticles; ++i )
  {
    scalar x_resid = (otherx.segment<2>(2*i)-x.segment<2>(2*i)).norm();
    scalar v_resid = (otherv.segment<2>(2*i)-v.segment<2>(2*i)).norm();
    if( x_resid > eps || v_resid > eps )
    {      
      Vector2s center;
      center << scale*(x(2*i)-xmin) + xshift, ((scalar)m_h) - scale*(x(2*i+1)-ymin) - yshift;
      renderCircle( file, center, 1.5*scale*radii[i], renderingutils::Color(1.0,0.0,0.0) );      
    }
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
	      renderImpulse( file, otherscene, (*otherimpulses)[j], true);
	      j++;
	    }

	  // check for missed collision
	  if( ! (j < (int)otherimpulses->size()
		 && (*otherimpulses)[j].m_idx1 == curvert
		 && (*otherimpulses)[j].m_type == curtype
		 && (*otherimpulses)[j].m_idx2 == curidx2))
	    {
	      renderImpulse( file, otherscene, (*impulses)[i], false);
	    }
	  else
	    {
	      // check for buggy normal
	      if( ((*otherimpulses)[j].m_n - (*impulses)[i].m_n).norm() > eps)
		{
		  renderImpulse( file, otherscene, (*impulses)[i], false);
		  renderImpulse( file, otherscene, (*otherimpulses)[j], true);
		}
	      j++;
	    }
	  
	  i++;
	}
      // Any remaining student impulses are buggy
      while(j < (int)otherimpulses->size())
	{
	  renderImpulse( file, otherscene, (*otherimpulses)[j], true);
	  j++;
	}
    }
  
  
  file << "</svg>" << std::endl;
  
  file.close();  
  
}
