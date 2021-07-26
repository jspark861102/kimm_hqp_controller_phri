#include "kimm_hqp/bindings/python/constraint/constraint_equality.hpp"
#include "kimm_hqp/bindings/python/constraint/expose_constraint.hpp"


namespace kimmhqp
{
  namespace python
  {
    void exposeConstraintEquality()
    {
      ConstraintEqPythonVisitor<kimmhqp::math::ConstraintEquality>::expose("ConstraintEquality");
    }
  }
}
