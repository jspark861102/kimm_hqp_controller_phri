#ifndef __kimm_hqp_controller_python_expose_constraint_bound_hpp__
#define __kimm_hqp_controller_python_expose_constraint_bound_hpp__

#include "kimm_hqp_controller/bindings/python/constraint/constraint_bound.hpp"
#include "kimm_hqp_controller/bindings/python/constraint/constraint_equality.hpp"
#include "kimm_hqp_controller/bindings/python/constraint/constraint_inequality.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeConstraintBound();
    void exposeConstraintEquality();
    void exposeConstraintInequality();
   
    inline void exposeConstraints()
    {
      exposeConstraintBound();
      exposeConstraintEquality();
      exposeConstraintInequality();
    }
    
  } // namespace python
} // namespace kimm_hqp_controller
#endif // ifndef __kimm_hqp_controller_python_expose_constraint_bound_hpp__
