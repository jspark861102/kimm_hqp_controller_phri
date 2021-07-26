#ifndef __kimm_hqp_controller_python_expose_formulations_hpp__
#define __kimm_hqp_controller_python_expose_formulations_hpp__

#include "kimm_hqp_controller/bindings/python/formulation/formulation.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeInverseDynamicsFormulationAccForce();

    inline void exposeFormulation()
    {
      exposeInverseDynamicsFormulationAccForce();
    }
    
  } // namespace python
} // namespace kimm_hqp_controller
#endif // ifndef __kimm_hqp_controller_python_expose_formulations_hpp__
