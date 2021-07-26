#include "kimm_hqp_controller/bindings/python/constraint/constraint_inequality.hpp"
#include "kimm_hqp_controller/bindings/python/constraint/expose_constraint.hpp"


namespace kimmhqp
{
  namespace python
  {
    void exposeConstraintInequality()
    {
      ConstraintIneqPythonVisitor<kimmhqp::math::ConstraintInequality>::expose("ConstraintInequality");
    }
  }
}
