
#ifndef __kimm_hqp_controller_python_expose_tasks_hpp__
#define __kimm_hqp_controller_python_expose_tasks_hpp__

#include "kimm_hqp_controller/bindings/python/tasks/task_se3_equality.hpp"
#include "kimm_hqp_controller/bindings/python/tasks/task_joint_posture.hpp"
#include "kimm_hqp_controller/bindings/python/tasks/task_joint_bound.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTaskSE3Equality();
    void exposeTaskJointPosture();
    void exposeTaskJointBounds();

    inline void exposeTasks()
    {
      exposeTaskSE3Equality();
      exposeTaskJointPosture();
      exposeTaskJointBounds();
    }
    
  } // namespace python
} // namespace kimm_hqp_controller
#endif // ifndef __kimm_hqp_controller_python_expose_tasks_hpp__
