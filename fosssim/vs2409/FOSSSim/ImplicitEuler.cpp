#include "ImplicitEuler.h"

// TODO: Turn this into a more generic Newton Solver
// TODO: Option to pass initial iterate
// TODO: Don't recreate dx, dv, rhs, A, etc... just make member vaiables, check if size is correct
// TODO: Have option to select different convergence criteria

ImplicitEuler::ImplicitEuler()
: SceneStepper()
{}

ImplicitEuler::~ImplicitEuler()
{}

bool ImplicitEuler::stepScene( TwoDScene& scene, scalar dt )
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

  // Most recent iterate. As an initial iterate, assume no forces act on system.
  // We specify the current iterate as a change from last timestep's solution.
  VectorXs dx = dt*v;
  // For scripted DoFs, change in position is just linear velocity. This is redundant with the above initial iterate, but others would require this.
  //for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) ) dx.segment<2>(2*i) = dt*v.segment<2>(2*i);
  // Corresponding change in velocity
  //VectorXs dv = dx/dt - v;
  VectorXs dv = VectorXs::Zero(ndof);
  
  // Matrix to use while building/solving system
  MatrixXs A(ndof,ndof);

  // RHS of linear system
  VectorXs rhs(ndof);

  // Change in step returned by linear solve
  VectorXs ddv(ndof);

  // Use change in step size as the convergence criterion
  scalar stplen = std::numeric_limits<scalar>::infinity();

  int numitrs = 0;
  while( stplen >= 1.0e-9 )
  {
    // rhs == h*f
    rhs.setZero();
    scene.accumulateGradU(rhs,dx,dv);
    rhs *= -dt;
    // rhs == -Mdv_i + h*f
    rhs += -(m.array()*dv.array()).matrix();

    // For scripted DoFs, zero the elements of fixed degrees of freedom
    zeroFixedDoFs( scene, rhs );

    // lhs == -df/dx
    A.setZero();
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

    ddv = A.fullPivLu().solve(rhs);

    // Check the residual from the linear solve
    #ifdef DEBUG
      VectorXs linslvrsd = A*ddv-rhs;
      scalar lininfnorm = linslvrsd.array().abs().maxCoeff();
      if( lininfnorm > 1.0e-9 )
      {
        std::cout << outputmod::startred << "WARNING IN IMPLICITEULER: ";
        std::cout << outputmod::endred << "Encountered large residual after linear solve: " << lininfnorm << std::endl;
      }
    #endif

    // Check the behavior of fixed DoFs
    #ifdef DEBUG
      for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) )
      {
        assert( ddv(2*i) == 0.0 );
        assert( ddv(2*i+1) == 0.0 );
      }
    #endif

    // Update the current iterate (change relative to last timestep's solution)
    dv += ddv;
    // Change in position determined by change in velocity
    dx = dt*(v+dv);
    
    stplen = ddv.norm();
    ++numitrs;
  }

  // Check the residual from the nonlinear solve
  #ifdef DEBUG
    // This just checks that we computed dx from dv correctly...
    scalar xresid = (dx-dt*(v+dv)).array().abs().maxCoeff();
    if( xresid > 1.0e-8 )
    {
      std::cout << outputmod::startred << "WARNING IN IMPLICITEULER: " << outputmod::endred;
      std::cout << "Encountered large residual (x) after nonlinear solve: " << xresid << std::endl;
    }
    // This is the residual that should actually indicate if there is a problem
    rhs.setZero();
    scene.accumulateGradU(rhs,dx,dv);
    zeroFixedDoFs(scene,rhs);
    rhs *= dt;
    rhs += (m.array()*dv.array()).matrix();
    scalar vresid = rhs.array().abs().maxCoeff();
    if( vresid > 1.0e-8 )
    {
      std::cout << outputmod::startred << "WARNING IN IMPLICITEULER: " << outputmod::endred;
      std::cout << "Encountered large residual (v) after nonlinear solve: " << vresid << std::endl;
    }
  #endif

  x += dx;
  v += dv;  

  return true;
}

std::string ImplicitEuler::getName() const
{
  return "Implicit Euler";
}

void ImplicitEuler::zeroFixedDoFs( const TwoDScene& scene, VectorXs& vec )
{
  int nprts = scene.getNumParticles();
  for( int i = 0; i < nprts; ++i ) if( scene.isFixed(i) ) vec.segment<2>(2*i).setZero();
}

void ImplicitEuler::setFixedRowsAndColsToIdentity( const TwoDScene& scene, MatrixXs& mat )
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
