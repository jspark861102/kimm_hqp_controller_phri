#include "kimm_hqp/bindings/python/robot/expose_robot.hpp"
#include "kimm_hqp/bindings/python/robot/robot_wrapper.hpp"

namespace kimmhqp
{
  namespace python
  {
    void exposeRobotWrapper()
    {
      RobotPythonVisitor<kimmhqp::robot::RobotWrapper>::expose("RobotWrapper");
    }
  }
}
