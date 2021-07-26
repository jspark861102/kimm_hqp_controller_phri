#ifndef __kimm_hqp_controller_python_expose_trajectories_hpp__
#define __kimm_hqp_controller_python_expose_trajectories_hpp__

#include "kimm_hqp_controller/bindings/python/trajectory/trajectory_se3.hpp"
#include "kimm_hqp_controller/bindings/python/trajectory/trajectory_euclidian.hpp"
#include "kimm_hqp_controller/bindings/python/trajectory/trajectory_base.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTrajectorySE3Constant();
    void exposeTrajectoryEuclidianConstant();
    void exposeTrajectorySample();
    void exposeTrajectorySE3Cubic();
    void exposeTrajectoryEuclidianCubic();
    
    inline void exposeTrajectories()
    {
      exposeTrajectorySE3Constant();
      exposeTrajectoryEuclidianConstant();
      exposeTrajectorySE3Cubic();
      exposeTrajectoryEuclidianCubic();
      exposeTrajectorySample();
    }    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_trajectories_hpp__
