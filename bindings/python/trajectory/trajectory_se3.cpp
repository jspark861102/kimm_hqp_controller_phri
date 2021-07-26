
#include "kimm_hqp/bindings/python/trajectory/trajectory_se3.hpp"
#include "kimm_hqp/bindings/python/trajectory/expose_trajectory.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTrajectorySE3Constant()
    {
      TrajectorySE3ConstantPythonVisitor<kimmhqp::trajectory::TrajectorySE3Constant>::expose("TrajectorySE3Constant");
    }
    void exposeTrajectorySE3Cubic()
    {
      TrajectorySE3CubicPythonVisitor<kimmhqp::trajectory::TrajectorySE3Cubic>::expose("TrajectorySE3Cubic");
    }
  }
}
