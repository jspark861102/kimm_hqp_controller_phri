
#ifndef __kimm_hqp_controller_python_task_se3_hpp__
#define __kimm_hqp_controller_python_task_se3_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/tasks/task_se3_equality.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"
#include "kimm_hqp_controller/trajectory/trajectory_base.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/math/constraint_base.hpp"
namespace kimmhqp
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename TaskSE3>
    struct TaskSE3EqualityPythonVisitor
    : public boost::python::def_visitor< TaskSE3EqualityPythonVisitor<TaskSE3> >
    {

      template<class PyClass>


      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robot::RobotWrapper &, std::string> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename")), "Default Constructor"))
        .add_property("dim", &TaskSE3::dim, "return dimension size")
        .def("setReference", &TaskSE3EqualityPythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskSE3EqualityPythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("getAcceleration", &TaskSE3EqualityPythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskSE3EqualityPythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskSE3EqualityPythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskSE3EqualityPythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskSE3EqualityPythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskSE3EqualityPythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskSE3EqualityPythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskSE3EqualityPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskSE3EqualityPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskSE3EqualityPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskSE3EqualityPythonVisitor::setKd, bp::arg("Kd"))
        .def("useLocalFrame", &TaskSE3EqualityPythonVisitor::useLocalFrame, bp::arg("local_frame"))
        .add_property("mask", bp::make_function(&TaskSE3EqualityPythonVisitor::getMask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskSE3EqualityPythonVisitor::setMask, bp::arg("mask"))
        .def("compute", &TaskSE3EqualityPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskSE3EqualityPythonVisitor::getConstraint)
        .add_property("frame_id", &TaskSE3::frame_id, "frame id return")
        .add_property("name", &TaskSE3EqualityPythonVisitor::name)
        ;
      }
       static std::string name(TaskSE3 & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskSE3 & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskSE3 & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskSE3 & self, trajectory::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskSE3 & self){
        return self.getDesiredAcceleration();
      }
      static Eigen::VectorXd getAcceleration (TaskSE3 & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskSE3 & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskSE3 & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskSE3 & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskSE3 & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskSE3 & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskSE3 & self){
        return self.velocity_ref();
      }
      static const Eigen::VectorXd & Kp (TaskSE3 & self){
        return self.Kp();
      }
      static const Eigen::VectorXd & Kd (TaskSE3 & self){
        return self.Kd();
      }
      static void setKp (TaskSE3 & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskSE3 & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void useLocalFrame (TaskSE3 & self, const bool local_frame) {
        self.useLocalFrame(local_frame);
      }
      static void getMask (TaskSE3 & self) {
        self.getMask();
      }
      static void setMask (TaskSE3 & self, const::Eigen::VectorXd mask) {
        self.setMask(mask);
      }
      static Eigen::VectorXd frame_id (TaskSE3 & self){
        return self.frame_id();
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskSE3 info.";
        bp::class_<TaskSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskSE3EqualityPythonVisitor<TaskSE3>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_task_se3_hpp__
