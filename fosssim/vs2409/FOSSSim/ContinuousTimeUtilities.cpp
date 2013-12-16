#include "ContinuousTimeUtilities.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <limits>

std::vector<Polynomial> PolynomialIntervalSolver::s_polynomials;

// Class representing a polynomial.
//
// Does not need to be changed by students.

// Creates a polynomial c0*t^n + c1*t^(n-1) + ... + cn from a vector of
// coefficients (c0, c1, ..., cn).
Polynomial::Polynomial(const std::vector<double> &coeffs) 
{
    // remove leading zeros
    m_coeffs.clear();
    int i = 0;
    for ( ; i < (int)coeffs.size() && fabs(coeffs[i]) < 1e-12; i++)
        ;
    for ( ; i < (int)coeffs.size(); i++)
        m_coeffs.push_back(coeffs[i]);
}

// Evaluates the polymomial at t.d
double Polynomial::evaluate(double t) const
{
    double result = 0;
    for(int i=0; i<(int)m_coeffs.size(); i++)
    {
        result *= t;
        result += m_coeffs[i];
    }
    return result;
}

// Interval classes used internally by the polynomial interval solver.
//
// Does not need to be changed by students.

bool overlap(const Interval &a, const Interval &b)
{
    return b.m_s <= a.m_e && a.m_s <= b.m_e;
}

Interval Iunion(const Interval &a, const Interval &b)
{
    assert(overlap(a,b));
    return Interval(std::min(a.m_s, b.m_s), std::max(a.m_e, b.m_e));
}

Interval Iintersect(const Interval &a, const Interval &b)
{
    assert(overlap(a,b));
    return Interval(std::max(a.m_s, b.m_s), std::min(a.m_e, b.m_e));
}

Intervals::Intervals(const std::vector<Interval> &intervals) : m_intervals()
{
    // Push on all intervals
    for(std::vector<Interval>::const_iterator it = intervals.begin(); it != intervals.end(); ++it)
    {
        m_intervals.push_back(*it);
    }
    m_intervals.sort();
    consolidateIntervals();
}

void Intervals::consolidateIntervals()
{
    std::list<Interval> consolidated;
    std::list<Interval>::iterator it = m_intervals.begin();
    while(it != m_intervals.end())
    {
        Interval inter = *it;
        it++;
        
        while(it != m_intervals.end() && overlap(inter, *it))
        {
            inter = Iunion(inter, *it);
            it++;
        }
        consolidated.push_back(inter);
    }
    m_intervals = consolidated;
}

double Intervals::findNextSatTime(double t)
{
    std::list<Interval>::iterator it = m_intervals.begin();
    while(it != m_intervals.end() && it->m_e <= t)
        ++it;
    
    if(it == m_intervals.end())
        return std::numeric_limits<double>::infinity();
    
    if(it->m_s <= t)
        return t;
    
    return it->m_s;
}

Intervals intersect(const Intervals &i1, const Intervals &i2)
{
    std::vector<Interval> seedIntervals;
    for(std::list<Interval>::const_iterator it = i1.m_intervals.begin(); it != i1.m_intervals.end(); ++it)
    {
        for(std::list<Interval>::const_iterator it2 = i2.m_intervals.begin(); it2 != i2.m_intervals.end(); ++it2)
        {
            if(overlap(*it, *it2))
                seedIntervals.push_back(Iintersect(*it, *it2));
        }
    }
    return Intervals(seedIntervals);
}

std::ostream &operator<<(std::ostream &os, const Intervals &inter)
{
    os << "[";
    bool first = true;
    for(std::list<Interval>::const_iterator it = inter.m_intervals.begin();
        it != inter.m_intervals.end(); ++it)
    {
        if(first)
            first = false;
        else
            os << ", ";
        os << "(" << it->m_s << ", " << it->m_e << ")";
    }
    os << "]";
    return os;
}

RootFinder PolynomialIntervalSolver::rf;

// Finds the intervals on which a given polynomial is > 0.
//
// Does not need to be changed by students.

Intervals PolynomialIntervalSolver::findPolyIntervals(const Polynomial &poly)
{
    const double eps = 1e-8;
    
    int leadcoeff=0;
    std::vector<Interval> empty;
    std::vector<Interval> all;
    all. push_back(Interval(-std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity()));
    
    const std::vector<double> &coeffs = poly.getCoeffs();
    int deg = coeffs.size()-1;
    
    for(int i=0; i<(int)coeffs.size(); i++)
        assert(!isnan(coeffs[i]));
    
    // get rid of leading 0s
//    for(leadcoeff=0; leadcoeff < (int)coeffs.size() && fabs(coeffs[leadcoeff]) < eps; leadcoeff++)
//    {
//        deg--;
//    }
    
    // check for the zero polynomial
    if(deg < 0)
    {
        return Intervals(empty);
    }
    
    // check for constant polynomial
    if(deg == 0)
    {
        double val = poly.evaluate(0);
        if(val > 0)
        {
            return Intervals(all);
        }
        return Intervals(empty);
    }
    
    // nonconstant polynomial... rpoly time!!!
    assert(deg <= 6);
    double zeror[6];
    double zeroi[6];
    int numroots = rf.rpoly(&coeffs[leadcoeff], deg, zeror, zeroi);
    
    std::vector<double> roots;
    for(int i=0; i<numroots; i++)
        if( fabs(zeroi[i]) < eps )
            roots.push_back(zeror[i]);
    
    // no roots: check at 0
    if(roots.size() == 0)
    {
        double val = poly.evaluate(0);
        if(val > 0)
            return Intervals(all);
        return Intervals(empty);
    }
    
    std::sort(roots.begin(), roots.end());
    
    std::vector<Interval> intervals;
    
    for(int i=0; i<(int)roots.size(); i++)
    {
        if(i == 0)
        {
            //check poly on (-inf, r)
            
            double t = roots[i]-1;
            double val = poly.evaluate(t);
            if(val > 0)
            {
                intervals.push_back(Interval(-std::numeric_limits<double>::infinity(),
                                             roots[i]));
            }
        }
        if(i == (int)roots.size()-1)
        {
            //check poly on (r, inf)
            double t = roots[i]+1;
            double val = poly.evaluate(t);
            if(val > 0)
            {
                intervals.push_back(Interval(roots[i],
                                             std::numeric_limits<double>::infinity()));
            }
        }
        
        if(i < (int)roots.size()-1)
        {
            // check poly on (r, r+1)
            double t = 0.5*(roots[i]+roots[i+1]);
            double val = poly.evaluate(t);
            if(val > 0)
            {
                intervals.push_back(Interval(roots[i],
                                             roots[i+1]));
            }
        }
    }
    return Intervals(intervals);
}

// Given some number of polynomials (where each polynomial c0*t^n +
// + c1*t^(n-1) + ... + cn is represented by the std::vector containing 
// (c0, c1, ..., cn), find the first time t >= 0 for which:
//   1. Each polynomial is >= 0 at t;
//   2. Each polynomial is > 0 at t + any sufficiently small number epsilon.
// If no such time exists, returns infinity instead.
//
// Does not need to be changed by students.
double PolynomialIntervalSolver::findFirstIntersectionTime(const std::vector<Polynomial> &polys)
{
    s_polynomials.insert(s_polynomials.end(), polys.begin(), polys.end());
    
    if(polys.size() == 0)
        return std::numeric_limits<double>::infinity();
    
    std::vector<Polynomial>::const_iterator it = polys.begin();
    
    Intervals inter = findPolyIntervals(*it);
    for(++it; it != polys.end(); ++it)
    {
        Intervals nextint = findPolyIntervals(*it);
        inter = intersect(inter, nextint);
    }
    return inter.findNextSatTime(0.0);
}

void PolynomialIntervalSolver::writePolynomials(std::ostream & os)
{
    int n = s_polynomials.size();
    os.write((char *)&n, sizeof (int));
    for (int i = 0; i < n; i++)
    {
        int m = s_polynomials[i].getCoeffs().size();
        os.write((char *)&m, sizeof (int));
        for (int j = 0; j < m; j++)
        {
            double c = s_polynomials[i].getCoeffs()[j];
            os.write((char *)&c, sizeof (double));
        }
    }
}

void PolynomialIntervalSolver::readPolynomials(std::vector<Polynomial> & polynomials, std::istream & is)
{
    polynomials.clear();
    int n;
    is.read((char *)&n, sizeof (int));
    for (int i = 0; i < n; i++)
    {
        std::vector<double> coefs;
        int m;
        is.read((char *)&m, sizeof (int));
        for (int j = 0; j < m; j++)
        {
            double c;
            is.read((char *)&c, sizeof (double));
            coefs.push_back(c);
        }
        Polynomial p(coefs);
        polynomials.push_back(p);
    }
}
