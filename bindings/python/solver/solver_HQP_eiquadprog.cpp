#include "kimm_hqp_controller/bindings/python/solver/expose_solver.hpp"
#include "kimm_hqp_controller/bindings/python/solver/solver_HQP_eiquadprog.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeSolverHQuadProg()
    {
      SolverHQuadProgPythonVisitor<kimmhqp::solver::SolverHQuadProg>::expose("SolverHQuadProg");
      SolverHQuadProgPythonVisitor<kimmhqp::solver::SolverHQuadProgFast>::expose("SolverHQuadProgFast");
    }
  }
}
