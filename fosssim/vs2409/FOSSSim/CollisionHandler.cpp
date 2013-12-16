#include "CollisionHandler.h"
#include "StringUtilities.h"
#include <iostream>

void CollisionHandler::addParticleParticleImpulse(int idx1, int idx2, const Vector2s &n, double time)
{
  m_impulses.push_back(CollisionInfo(CollisionInfo::PP, idx1, idx2, n, time));
}

void CollisionHandler::addParticleEdgeImpulse(int vidx, int eidx, const Vector2s &n, double time)
{
  m_impulses.push_back(CollisionInfo(CollisionInfo::PE, vidx, eidx, n, time));
}

void CollisionHandler::addParticleHalfplaneImpulse(int vidx, int pidx, const Vector2s &n, double time)
{
  m_impulses.push_back(CollisionInfo(CollisionInfo::PH, vidx, pidx, n, time));
}

void CollisionHandler::serializeImpulses(std::ofstream &ofs)
{
  int numimpulses = (int)m_impulses.size();
  ofs.write((char *)&numimpulses, sizeof(int));
  for(int i=0; i<numimpulses; i++)
    {
      CollisionInfo &ii = m_impulses[i];
      ofs.write((char *)&ii.m_type, sizeof(CollisionInfo::collisiontype));
      ofs.write((char *)&ii.m_idx1, sizeof(int));
      ofs.write((char *)&ii.m_idx2, sizeof(int));
      ofs.write((char *)ii.m_n.data(), ii.m_n.size()*sizeof(scalar));
      ofs.write((char *)&ii.m_time, sizeof(double));
    }
}

void CollisionHandler::loadImpulses(std::vector<CollisionInfo> &impulses, std::ifstream &ifs)
{
  impulses.clear();
  int numimpulses=0;
  ifs.read((char *)&numimpulses, sizeof(int));
  for(int i=0; i<numimpulses; i++)
    {
      CollisionInfo::collisiontype type;
      ifs.read((char *)&type, sizeof(CollisionInfo::collisiontype));
      int idx1, idx2;
      ifs.read((char *)&idx1, sizeof(int));
      ifs.read((char *)&idx2, sizeof(int));
      VectorXs n(2);
      ifs.read((char *)n.data(), n.size()*sizeof(scalar));
      double time;
      ifs.read((char *)&time, sizeof(double));
      impulses.push_back(CollisionInfo(type, idx1, idx2, n, time));
    }

  if( ifs.fail() )
    {
      std::cout << outputmod::startred << "Error while trying to deserialize time step impulses. Exiting." << std::endl;
      exit(1);
    }
}
