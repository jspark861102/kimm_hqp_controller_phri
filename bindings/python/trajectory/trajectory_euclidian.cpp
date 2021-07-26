
#include "kimm_hqp_controller/bindings/python/trajectory/trajectory_euclidian.hpp"
#include "kimm_hqp_controller/bindings/python/trajectory/expose_trajectory.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTrajectoryEuclidianConstant()
    {
      TrajectoryEuclidianConstantPythonVisitor<kimmhqp::trajectory::TrajectoryEuclidianConstant>::expose("TrajectoryEuclidianConstant");
    }
    void exposeTrajectoryEuclidianCubic()
    {
      TrajectoryEuclidianCubicPythonVisitor<kimmhqp::trajectory::TrajectoryEuclidianCubic>::expose("TrajectoryEuclidianCubic");
    }
  }
}
