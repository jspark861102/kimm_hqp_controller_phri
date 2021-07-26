
#include "kimm_hqp/bindings/python/formulation/expose_formulation.hpp"
#include "kimm_hqp/bindings/python/formulation/formulation.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeInverseDynamicsFormulationAccForce()
    {
      InvDynPythonVisitor<kimmhqp::InverseDynamicsFormulationAccForce>::expose("InverseDynamicsFormulationAccForce");
    }
  }
}
