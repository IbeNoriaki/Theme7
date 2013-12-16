#include "LinearizedImplicitEuler.h"

// TODO: Save space by creating dx, dv, rhs, A only once.

LinearizedImplicitEuler::LinearizedImplicitEuler()
: SceneStepper()
{}

LinearizedImplicitEuler::~LinearizedImplicitEuler()
{}

bool LinearizedImplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
  VectorXs& x = scene.getX();
  VectorXs& v = scene.getV();
  const VectorXs& m = scene.getM();
  assert(x.size() == v.size());
  assert(x.size() == m.size());
  int ndof = x.size();
  assert( ndof%2 == 0 );
  int nprts = scene.getNumParticles();
  assert( 2*nprts == ndof );

  // We specify the current iterate as a change from last timestep's solution
  // Linearizing about the previous timestep's solution implies dx = dt*v0
  VectorXs dx = dt*v;
  // Linearizing about the previous timestep's solution implies dv = 0
  VectorXs dv = VectorXs::Zero(ndof);

  // RHS of linear system is force. rhs == f
  VectorXs rhs = VectorXs::Zero(ndof);
  scene.accumulateGradU(rhs,dx,dv);
  rhs *= -1.0;
  // rhs == h*f
  rhs *= dt;
  // For scripted DoFs, zero the elements of fixed degrees of freedom
  zeroFixedDoFs( scene, rhs );

  // Matrix to use while building/solving system
  MatrixXs A = MatrixXs::Zero(ndof,ndof);
  // lhs == -df/dx
  scene.accumulateddUdxdx(A,dx,dv);
  // lhs == -h*df/dx
  A *= dt;
  // lhs == -df/dv -h*df/dx
  scene.accumulateddUdxdv(A,dx,dv);
  // lhs == -h*df/dv -h^2*df/dx
  A *= dt;
  // lhs == M -h*df/dv -h^2*df/dx
  A.diagonal() += m;
  // For scripted DoFs, zero out the rows/cols of fixed degrees of freedom
  setFixedRowsAndColsToIdentity(scene,A);

  // Change in velocity returned by linear solve
  VectorXs dqdot = A.fullPivLu().solve(rhs);

  // Check the residual from the linear solve
  #ifdef DEBUG
    VectorXs linslvrsd = A*dqdot-rhs;
    scalar lininfnorm = linslvrsd.array().abs().maxCoeff();
    if( lininfnorm > 1.0e-9 )
    {
      std::cout << outputmod::startred << "WARNING IN IMPLICITEULER: " << outputmod::endred;
      std::cout << "Encountered large residual after linear solve: " << lininfnorm << std::endl;
    }
  #endif

  // Check the behavior of fixed DoFs
  #ifdef DEBUG
    for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) )
    {
      assert( dqdot(2*i) == 0.0 );
      assert( dqdot(2*i+1) == 0.0 );
    }
  #endif

  v += dqdot;
  x += dt*v;

  return true;
}

std::string LinearizedImplicitEuler::getName() const
{
  return "Linearized Implicit Euler";
}

void LinearizedImplicitEuler::zeroFixedDoFs( const TwoDScene& scene, VectorXs& vec )
{
  int nprts = scene.getNumParticles();
  for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) ) vec.segment<2>(2*i).setZero();
}

void LinearizedImplicitEuler::setFixedRowsAndColsToIdentity( const TwoDScene& scene, MatrixXs& mat )
{
  int nprts = scene.getNumParticles();
  for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) )
  {
    mat.row(2*i).setZero();
    mat.row(2*i+1).setZero();
    mat.col(2*i).setZero();
    mat.col(2*i+1).setZero();
    // Set diagonal of fixed degrees of freedom to 1
    mat(2*i,2*i) = 1.0;
    mat(2*i+1,2*i+1) = 1.0;
  }  
}
