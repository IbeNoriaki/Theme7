#include "SimpleCollisionHandler.h"
#include <iostream>
#include <set>

// Loops over all edges, particles, and half-planes in the simulation, and
// applies collision detection and handling to each pair.
//
// Note: No collision detection is done between edges that are connected
// by a particle (since these are always colliding.)
//
// Does not need to be changed by students.

void SimpleCollisionHandler::handleCollisions(TwoDScene &scene, const VectorXs &oldpos, VectorXs &oldvel, scalar dt)
{
    Vector2s n;
    
    for (int i = 0; i < scene.getNumParticles(); i++)
    {
        for (int j = i + 1; j < scene.getNumParticles(); j++)
        {
            if (detectParticleParticle(scene, i, j, n))
            {
                addParticleParticleImpulse(i, j, n, 0);
                respondParticleParticle(scene, i, j, n);
            }
        }
        
        for (int e = 0; e < scene.getNumEdges(); e++)
        {
            if (scene.getEdges()[e].first != i && scene.getEdges()[e].second != i)
                if (detectParticleEdge(scene, i, e, n))
                {
                    addParticleEdgeImpulse(i, e, n, 0);
                    respondParticleEdge(scene, i, e, n);
                }
        }
        
        for (int f = 0; f < scene.getNumHalfplanes(); f++)
        {
            if (detectParticleHalfplane(scene, i, f, n))
            {
                addParticleHalfplaneImpulse(i, f, n, 0);
                respondParticleHalfplane(scene, i, f, n);
            }
        }
    }
}

std::string SimpleCollisionHandler::getName() const
{
    return "Simple Collision Handling";
}




// BEGIN STUDENT CODE //


// Detects whether two particles are overlapping (including the radii of each)
// and approaching.
// If the two particles overlap and are approaching, returns true and sets 
// the vector n to be the vector between the first and second particle.
// Inputs:
//   scene: The scene data structure. The positions and radii of the particles
//          can be obtained from here.
//   idx1:  The index of the first particle. (Ie, the degrees of freedom
//          corresponding to this particle are entries 2*idx1 and 2*idx1+1 in
//          scene.getX().
//   idx2:  The index of the second particle.
// Outputs:
//   n: The vector between the two particles.
//   Returns true if the two particles overlap and are approaching.
bool SimpleCollisionHandler::detectParticleParticle(TwoDScene &scene, int idx1, int idx2, Vector2s &n)
{
//    VectorXs x1 = scene.getX().segment<2>(2*idx1);
//    VectorXs x2 = scene.getX().segment<2>(2*idx2);
//    
//    // your implementation here
//    
//    return false;
    VectorXs x1 = scene.getX().segment<2>(2*idx1);
    VectorXs x2 = scene.getX().segment<2>(2*idx2);
    n = x2-x1;
    if(n.norm() < scene.getRadius(idx1) + scene.getRadius(idx2))
    {
        double relvel = (scene.getV().segment<2>(2*idx1)-scene.getV().segment<2>(2*idx2)).dot(n);
        if(relvel > 0)
            return true;
    }
    return false;
}

// Detects whether a particle and an edge are overlapping (including the radii 
// of both) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the 
// vector n to be the shortest vector between the particle and the edge.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge. (Ie, the indices of particle with index e are
//          scene.getEdges()[e].first and scene.getEdges()[e].second.)
// Outputs:
//   n: The shortest vector between the particle and the edge.
//   Returns true if the two objects overlap and are approaching.
bool SimpleCollisionHandler::detectParticleEdge(TwoDScene &scene, int vidx, int eidx, Vector2s &n)
{
//    VectorXs x1 = scene.getX().segment<2>(2*vidx);
//    VectorXs x2 = scene.getX().segment<2>(2*scene.getEdges()[eidx].first);
//    VectorXs x3 = scene.getX().segment<2>(2*scene.getEdges()[eidx].second);
//    
//    // your implementation here
//    
//    return false;
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs x2 = scene.getX().segment<2>(2*scene.getEdges()[eidx].first);
    VectorXs x3 = scene.getX().segment<2>(2*scene.getEdges()[eidx].second);
    double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
    alpha = std::min(1.0, std::max(0.0, alpha));
    
    VectorXs closest = x2 + alpha*(x3-x2);
    n = closest-x1;
    
    if(n.norm() < scene.getRadius(vidx)+scene.getEdgeRadii()[eidx]) 
    {
        VectorXs v1 = scene.getV().segment<2>(2*vidx);
        VectorXs v2 = scene.getV().segment<2>(2*scene.getEdges()[eidx].first);
        VectorXs v3 = scene.getV().segment<2>(2*scene.getEdges()[eidx].second);
        double relvel = (v1 - v2 - alpha*(v3-v2)).dot(n);
        if(relvel > 0)
        {
            return true;
        }
    }
    return false;
}

// Detects whether a particle and a half-plane are overlapping (including the 
// radius of the particle) and are approaching.
// If the two objects overlap and are approaching, returns true and sets the 
// vector n to be the shortest vector between the particle and the half-plane.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the halfplane. The vectors (px, py) and (nx, ny) can
//          be retrieved by calling scene.getHalfplane(pidx).
// Outputs:
//   n: The shortest vector between the particle and the half-plane.
//   Returns true if the two objects overlap and are approaching.
bool SimpleCollisionHandler::detectParticleHalfplane(TwoDScene &scene, int vidx, int pidx, Vector2s &n)
{
//    VectorXs x1 = scene.getX().segment<2>(2*vidx);
//    VectorXs px = scene.getHalfplane(pidx).first;
//    VectorXs pn = scene.getHalfplane(pidx).second;
//    
//    // your implementation here
//    
//    return false;
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs px = scene.getHalfplane(pidx).first;
    VectorXs pn = scene.getHalfplane(pidx).second;
    pn.normalize();
    n = (px-x1).dot(pn)*pn;
    if(n.norm() < scene.getRadius(vidx))
    {
        double relvel = scene.getV().segment<2>(2*vidx).dot(n);
        if(relvel > 0)
            return true;
    }
    return false;
}


// Responds to a collision detected between two particles by applying an impulse
// to the velocities of each one.
// You can get the COR of the simulation by calling getCOR().
// Inputs:
//   scene: The scene data structure.
//   idx1:  The index of the first particle.
//   idx2:  The index of the second particle.
//   n:     The vector between the first and second particle.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleParticle(TwoDScene &scene, int idx1, int idx2, const Vector2s &n)
{
//    const VectorXs &M = scene.getM();
//    VectorXs &v = scene.getV();
//    
//    // your implementation here
//    
    const VectorXs &M = scene.getM();
    VectorXs &v = scene.getV();
    
    VectorXs nhat = n;
    nhat.normalize();
    
    double cfactor = (1.0 + getCOR())/2.0;
    double m1 = scene.isFixed(idx1) ? std::numeric_limits<double>::infinity() : M[2*idx1];
    double m2 = scene.isFixed(idx2) ? std::numeric_limits<double>::infinity() : M[2*idx2];
    
    double numerator = 2*cfactor * (v.segment<2>(2*idx2) - v.segment<2>(2*idx1) ).dot(nhat);
    double denom1 = 1+m1/m2;
    double denom2 = m2/m1 + 1;
    
    if(!scene.isFixed(idx1))
        v.segment<2>(2*idx1) += numerator/denom1 * nhat;
    if(!scene.isFixed(idx2))
        v.segment<2>(2*idx2) -= numerator/denom2 * nhat;
}

// Responds to a collision detected between a particle and an edge by applying
// an impulse to the velocities of each one.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   eidx:  The index of the edge.
//   n:     The shortest vector between the particle and the edge.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleEdge(TwoDScene &scene, int vidx, int eidx, const Vector2s &n)
{
//    const VectorXs &M = scene.getM();
//    
//    int eidx1 = scene.getEdges()[eidx].first;
//    int eidx2 = scene.getEdges()[eidx].second;
//    
//    VectorXs x1 = scene.getX().segment<2>(2*vidx);
//    VectorXs x2 = scene.getX().segment<2>(2*eidx1);
//    VectorXs x3 = scene.getX().segment<2>(2*eidx2);
//    
//    VectorXs v1 = scene.getV().segment<2>(2*vidx);
//    VectorXs v2 = scene.getV().segment<2>(2*eidx1);
//    VectorXs v3 = scene.getV().segment<2>(2*eidx2);
//    
//    // your implementation here
//    
    const VectorXs &M = scene.getM();
    
    int eidx1 = scene.getEdges()[eidx].first;
    int eidx2 = scene.getEdges()[eidx].second;
    
    VectorXs x1 = scene.getX().segment<2>(2*vidx);
    VectorXs x2 = scene.getX().segment<2>(2*eidx1);
    VectorXs x3 = scene.getX().segment<2>(2*eidx2);
    
    VectorXs v1 = scene.getV().segment<2>(2*vidx);
    VectorXs v2 = scene.getV().segment<2>(2*eidx1);
    VectorXs v3 = scene.getV().segment<2>(2*eidx2);
    
    VectorXs nhat = n;
    nhat.normalize();
    
    double alpha = (x1-x2).dot(x3-x2)/(x3-x2).dot(x3-x2);
    alpha = std::min(1.0, std::max(0.0, alpha) );
    VectorXs vedge = v2 + alpha*(v3-v2);
    double cfactor = (1.0 + getCOR())/2.0;
    
    double m1 = scene.isFixed(vidx) ? std::numeric_limits<double>::infinity() : M[2*vidx];
    double m2 = scene.isFixed(eidx1) ? std::numeric_limits<double>::infinity() : M[2*eidx1];
    double m3 = scene.isFixed(eidx2) ? std::numeric_limits<double>::infinity() : M[2*eidx2];
    
    double numerator = 2*cfactor*(vedge-v1).dot(nhat);
    double denom1 = 1.0 + (1-alpha)*(1-alpha)*m1/m2 + alpha*alpha*m1/m3;
    double denom2 = m2/m1 + (1-alpha)*(1-alpha) + alpha*alpha*m2/m3;
    double denom3 = m3/m1 + (1-alpha)*(1-alpha)*m3/m2 + alpha*alpha;
    
    if(!scene.isFixed(vidx))
        scene.getV().segment<2>(2*vidx) += numerator/denom1 * nhat;
    if(!scene.isFixed(eidx1))
        scene.getV().segment<2>(2*eidx1) -= (1.0-alpha)*numerator/denom2 * nhat;
    if(!scene.isFixed(eidx2))
        scene.getV().segment<2>(2*eidx2) -= alpha * numerator/denom3 * nhat;
}


// Responds to a collision detected between a particle and a half-plane by 
// applying an impulse to the velocity of the particle.
// Inputs:
//   scene: The scene data structure.
//   vidx:  The index of the particle.
//   pidx:  The index of the half-plane.
//   n:     The shortest vector between the particle and the half-plane.
// Outputs:
//   None.
void SimpleCollisionHandler::respondParticleHalfplane(TwoDScene &scene, int vidx, int pidx, const Vector2s &n)
{
//    VectorXs nhat = n;
//    
//    // your implementation here
//    
    VectorXs nhat = n;
    nhat.normalize();
    double cfactor = (1.0+getCOR())/2.0;
    scene.getV().segment<2>(2*vidx) -= 2*cfactor*scene.getV().segment<2>(2*vidx).dot(nhat)*nhat;
}
