#include "kimm_hqp/bindings/python/solver/expose_solver.hpp"
#include "kimm_hqp/bindings/python/solver/HQPOutput.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeHQPOutput()
    {
      HQPOutputPythonVisitor<solver::HQPOutput>::expose("HQPOutput");
    }
  }
}
