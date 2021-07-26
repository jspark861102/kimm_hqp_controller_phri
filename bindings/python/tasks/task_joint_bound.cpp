#include "kimm_hqp_controller/bindings/python/tasks/task_joint_bound.hpp"
#include "kimm_hqp_controller/bindings/python/tasks/expose_tasks.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTaskJointBounds()
    {
      TaskJointBoundsPythonVisitor<kimmhqp::tasks::TaskTorqueBounds>::expose("TaskJointBounds");
    }
  }
}
