
#ifndef __kimm_hqp_controller_python_expose_solvers_hpp__
#define __kimm_hqp_controller_python_expose_solvers_hpp__


#include "kimm_hqp_controller/bindings/python/solver/solver_HQP_eiquadprog.hpp"
#include "kimm_hqp_controller/bindings/python/solver/HQPData.hpp"
#include "kimm_hqp_controller/bindings/python/solver/HQPOutput.hpp"
namespace kimmhqp
{
  namespace python
  {
    void exposeSolverHQuadProg();
    void exposeConstraintLevel();
    void exposeHQPData();
    void exposeHQPOutput();
    inline void exposeSolver()
    {
      exposeSolverHQuadProg();
      exposeConstraintLevel();
      exposeHQPData();
      exposeHQPOutput();
    }
    
  } // namespace python
} // namespace kimm_hqp_controller
#endif // ifndef __kimm_hqp_controller_python_expose_solvers_hpp__
