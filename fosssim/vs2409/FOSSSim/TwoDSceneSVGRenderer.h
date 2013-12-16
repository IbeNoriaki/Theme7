#ifndef __TWO_D_SCENE_SVG_RENDERER_H__
#define __TWO_D_SCENE_SVG_RENDERER_H__

#include <iostream>
#include <fstream>
#include <iomanip>

#include "TwoDScene.h"
#include "MathUtilities.h"
#include "RenderingUtilities.h"
#include "CollisionHandler.h"

class TwoDimensionalDisplayController;

class TwoDSceneSVGRenderer
{
public:
  
  TwoDSceneSVGRenderer( const TwoDScene& scene, const TwoDimensionalDisplayController &dc, const std::vector<renderingutils::Color>& particle_colors, const std::vector<renderingutils::Color>& edge_colors, const std::vector<renderingutils::Color> &halfplane_colors, const std::vector<renderingutils::ParticlePath>& particle_paths, int imagewidth, int imageheight, const renderingutils::Color& bgcolor );

  void renderScene( const std::string& filename ) const;
  void renderComparisonScene( const std::string& filename, const TwoDScene& otherscene, const std::vector<CollisionInfo> *impulses, const std::vector<CollisionInfo> *otherimpulses, const scalar &eps = 1.0e-9) const;

  //void circleMajorResiduals( const TwoDScene& oracle_scene, const TwoDScene& testing_scene, scalar eps = 1.0e-9 ) const;

  void updateState();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
private:

  std::string intToHexString( int integer ) const;
  //  void computeBoundingBox( const VectorXs& x, const std::vector<scalar>& radii, scalar& xmin, scalar& xmax, scalar& ymin, scalar& ymax ) const;
  void computeSimToImageMap( scalar& scale, scalar& xmin, scalar& ymin, scalar& xshift, scalar& yshift ) const;
  
  void renderSolidCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const;
  void renderCircle( std::fstream& file, const Vector2s& center, const scalar& r, const renderingutils::Color& color ) const;
  void renderImpulse( std::fstream &files, const TwoDScene &scene, const CollisionInfo &impulse, bool buggy) const;
  void renderSweptEdge( std::fstream& file, const Vector2s& x0, const Vector2s& x1, const scalar& r, const renderingutils::Color& color ) const;
  void renderHalfplane( std::fstream &file, const VectorXs &x, const VectorXs &n, const renderingutils::Color& color) const;

  void renderShared( std::fstream& file, const VectorXs& x, const std::vector<std::pair<int,int> >& edges, const std::vector<scalar>& radii, const std::vector<scalar>& edgeradii, const scalar& scale, const scalar& xmin, const scalar& ymin, const scalar& xshift, const scalar& yshift  ) const;
  
  const TwoDScene& m_scene;
  const TwoDimensionalDisplayController &m_dc;

  int m_w;
  int m_h;
  
  renderingutils::Color m_bgcolor;

  std::vector<renderingutils::Color> m_particle_colors;
  std::vector<renderingutils::Color> m_edge_colors;
  std::vector<renderingutils::Color> m_halfplane_colors;
  std::vector<renderingutils::ParticlePath> m_particle_paths;

  std::vector<std::pair<double,double> > m_circle_points;
  std::vector<std::pair<double,double> > m_semi_circle_points;
};

#endif
