#include "kimm_hqp_controller/solver/solver_HQP_factory.hpp"
#include "kimm_hqp_controller/solver/solver_HQP_eiquadprog.hpp"
#include "kimm_hqp_controller/solver/solver_HQP_eiquadprog_fast.hpp"
#include "kimm_hqp_controller/solver/solver_HQP_qpoases.hpp"
namespace kimmhqp
{
  namespace solver
  {
    SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType, const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG)
        return new SolverHQuadProg(name);
      
      if(solverType==SOLVER_HQP_EIQUADPROG_FAST)
        return new SolverHQuadProgFast(name);

      if(solverType==SOLVER_HQP_QPOASES)
        return new SolverHQPoases(name);    
      
      assert(false && "Specified solver type not recognized");
      return NULL;
    }
    
  }
}