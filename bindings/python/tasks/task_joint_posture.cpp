#include "kimm_hqp_controller/bindings/python/tasks/task_joint_posture.hpp"
#include "kimm_hqp_controller/bindings/python/tasks/expose_tasks.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTaskJointPosture()
    {
      TaskJointPosturePythonVisitor<kimmhqp::tasks::TaskJointPosture>::expose("TaskJointPosture");
    }
  }
}
