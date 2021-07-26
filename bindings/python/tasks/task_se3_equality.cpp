#include "kimm_hqp/bindings/python/tasks/task_se3_equality.hpp"
#include "kimm_hqp/bindings/python/tasks/expose_tasks.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeTaskSE3Equality()
    {
      TaskSE3EqualityPythonVisitor<kimmhqp::tasks::TaskSE3Equality>::expose("TaskSE3Equality");
    }
  }
}
