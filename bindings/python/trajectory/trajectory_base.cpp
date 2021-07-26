
#include "kimm_hqp_controller/bindings/python/trajectory/trajectory_base.hpp"
#include "kimm_hqp_controller/bindings/python/trajectory/expose_trajectory.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTrajectorySample()
    {
      TrajectorySamplePythonVisitor<kimmhqp::trajectory::TrajectorySample>::expose("TrajectorySample");
    }
  }
}
