#include "HybridCollisionComparison.h"

HybridCollisionComparison::HybridCollisionComparison()
{
    reset();
}

void HybridCollisionComparison::reset()
{
    PIq = VectorXs(1);
    PIq.setZero();
    PIqdot = VectorXs(1);
    PIqdot.setZero();
    IZiters.clear();
    fail = false;
}

void HybridCollisionComparison::serialize(std::ofstream &ofs)
{
    int dim = PIq.size();
    assert(PIqdot.size() == dim);
    ofs.write( (char *)&dim, sizeof(int));
    ofs.write( (char *)PIq.data(), dim*sizeof(scalar));
    ofs.write( (char *)PIqdot.data(), dim*sizeof(scalar));
    int numentries = IZiters.size();
    ofs.write( (char *)&numentries, sizeof(int));
    for(int i=0 ;i<numentries; i++)
    {
        int numzones = IZiters[i].Z.size();
        ofs.write( (char *)&numzones, sizeof(int));
        for(int j=0; j<numzones; j++)
        {
            ImpactZone &iz = IZiters[i].Z[j];
            char hp = (iz.m_halfplane ? 1 : 0);
            ofs.write( (char *)&hp, 1);
            int numverts = iz.m_verts.size();
            ofs.write( (char *)&numverts, sizeof(int));
            for(std::set<int>::iterator it = iz.m_verts.begin(); it != iz.m_verts.end(); ++it)
                ofs.write( (char *)&*it, sizeof(int));
        }
        int dim = IZiters[i].q.size();
        assert(IZiters[i].qdot.size() == dim);
        ofs.write( (char *)&dim, sizeof(int));
        ofs.write( (char *)IZiters[i].q.data(), dim*sizeof(scalar));
        ofs.write( (char *)IZiters[i].qdot.data(), dim*sizeof(scalar));
    }
}


void HybridCollisionComparison::load(std::ifstream &ifs)
{
    int dim;
    ifs.read( (char *)&dim, sizeof(int));
    
    VectorXs q(dim);
    VectorXs qdot(dim);
    ifs.read( (char *)q.data(), dim*sizeof(scalar));
    ifs.read( (char *)qdot.data(), dim*sizeof(scalar));
    PIq = q;
    PIqdot = qdot;
    int entries;
    ifs.read( (char *)&entries, sizeof(int));
    for(int i=0; i<entries; i++)
    {
        ImpactZoneInfo izi;
        int numzones;
        ifs.read((char *)&numzones, sizeof(int));
        for(int j=0; j<numzones; j++)
        {
            char hp;
            ifs.read(&hp, 1);
            std::set<int> verts;
            int numverts;
            ifs.read( (char *)&numverts, sizeof(int));
            for(int k=0; k<numverts; k++)
            {
                int val;
                ifs.read((char *)&val, sizeof(int));
                verts.insert(val);
            }
            ImpactZone iz(verts, hp>0);
            izi.Z.push_back(iz);
        }
        int dim;
        ifs.read((char *)&dim, sizeof(int));
        VectorXs q(dim);
        VectorXs qdot(dim);
        ifs.read( (char *)q.data(), dim*sizeof(scalar));
        ifs.read( (char *)qdot.data(), dim*sizeof(scalar));
        izi.q = q;
        izi.qdot = qdot;
        IZiters.push_back(izi);
    }
}

void HybridCollisionComparison::serializePostImpulses(const VectorXs &q, const VectorXs &qdot)
{
    PIq = q;
    PIqdot = qdot;
}

void HybridCollisionComparison::serializeImpactZones(const ImpactZones &Z, const VectorXs &q, const VectorXs &qdot, int iter)
{
    assert(iter == (int)IZiters.size());
    ImpactZoneInfo izi;
    izi.Z = Z;
    izi.q = q;
    izi.qdot = qdot;
    IZiters.push_back(izi);
}

const double eps = 1e-8;

bool HybridCollisionComparison::comparePI(VectorXs &q, VectorXs &qdot)
{
    if( (PIq - q).norm() > eps)
    {
        std::cerr << "Position after iterated impulses wrong: residual " << (PIq-q).norm() << std::endl;
        q = PIq;
        qdot = PIqdot;
        fail = true;
        return false;
    }
    if( (PIqdot - qdot).norm() > eps)
    {
        std::cerr << "Velocity after iterated impulses wrong: residual " << (PIqdot-qdot).norm() << std::endl;
        q = PIq;
        qdot = PIqdot;
        fail = true;
        return false;
    }
    
    q = PIq;
    qdot = PIqdot;
    return true;
}

void printIZ(ImpactZones &z)
{
    std::cerr << "{";
    for(int i=0; i<(int)z.size(); i++)
    {
        if(i != 0)
            std::cerr << ", ";
        std::cerr << "{";
        for(std::set<int>::iterator it = z[i].m_verts.begin(); it != z[i].m_verts.end(); ++it)
            std::cerr << *it << ", ";
        std::cerr << (z[i].m_halfplane ? "h-plane}" : "no-h-plane}");
    }
    std::cerr << "}";
}

void printBothZones(ImpactZones &z1, ImpactZones &z2, int iter)
{
    std::cerr << "Impact zones do not match for geometric response iteration " << iter << std::endl;
    std::cerr << "Student impact zones:" << std::endl;
    printIZ(z1);
    std::cerr << std::endl << "Expected impact zones:" << std::endl;
    printIZ(z2);
    std::cerr << std::endl << std::endl;
}

bool HybridCollisionComparison::compareIZ(const ImpactZones &iz, const VectorXs &q, const VectorXs &qdot, int iter)
{
    if(iter >= (int)IZiters.size())
    {
        std::cerr << "Number of geometric response iterations does not match! Aborting further iterations." << std::endl;
        fail = true;
        return false;
    }
    ImpactZoneInfo izi = IZiters[iter];
    
    ImpactZones Z1 = izi.Z;
    ImpactZones Z2 = iz;
    
    std::sort(Z1.begin(), Z1.end());
    std::sort(Z2.begin(), Z2.end());
    
    if(!(Z1 == Z2))
    {
        printBothZones(Z1, Z2, iter);
        fail = true;
        return false;
    }
    
    if( (izi.q - q).norm() > eps)
    {
        std::cerr << "Position after geometric impulse iteration " << iter << " wrong: residual " << (izi.q-q).norm() << std::endl;
        fail = true;
        return false;
    }
    if( (izi.qdot - qdot).norm() > eps)
    {
        std::cerr << "Velocity after geometric impulse iteration " << iter << " wrong: residual " << (izi.qdot-qdot).norm() << std::endl;
        fail = true;
        return false;
    }
    return true;
}

bool HybridCollisionComparison::doneWithImpactZones(int iter)
{
    if(iter < (int)IZiters.size()-1)
    {
        std::cerr << "Not enough iterations of geometric collision response!" << std::endl;
        fail = true;
        return false;
    }
    return true;
}

