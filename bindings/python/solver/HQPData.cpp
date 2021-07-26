
#include "kimm_hqp/bindings/python/solver/expose_solver.hpp"
#include "kimm_hqp/bindings/python/solver/HQPData.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeConstraintLevel()
    {
      ConstPythonVisitor<ConstraintLevels>::expose("ConstraintLevel");
    }
    void exposeHQPData()
    {
      HQPPythonVisitor<HQPDatas>::expose("HQPData");
    }
  }
}
