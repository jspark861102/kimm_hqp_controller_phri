
#ifndef __kimm_hqp_controller_python_task_joint_hpp__
#define __kimm_hqp_controller_python_task_joint_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/tasks/task_joint_posture.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"
#include "kimm_hqp_controller/trajectory/trajectory_base.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/math/constraint_base.hpp"
namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskJoint>
    struct TaskJointPosturePythonVisitor
    : public boost::python::def_visitor< TaskJointPosturePythonVisitor<TaskJoint> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robot::RobotWrapper &> ((bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &TaskJoint::dim, "return dimension size")
        .def("setReference", &TaskJointPosturePythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskJointPosturePythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .add_property("mask", bp::make_function(&TaskJointPosturePythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskJointPosturePythonVisitor::setmask, bp::arg("mask"))
        .def("getAcceleration", &TaskJointPosturePythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskJointPosturePythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskJointPosturePythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskJointPosturePythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskJointPosturePythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskJointPosturePythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskJointPosturePythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskJointPosturePythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskJointPosturePythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskJointPosturePythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskJointPosturePythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskJointPosturePythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskJointPosturePythonVisitor::getConstraint)
        .add_property("name", &TaskJointPosturePythonVisitor::name)
        ;
      }
      static std::string name(TaskJoint & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskJoint & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskJoint & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskJoint & self, const trajectory::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskJoint & self){
        return self.getDesiredAcceleration();
      }
      static const Eigen::VectorXd & getmask(const TaskJoint & self){
        return self.getMask();
      }
      static void setmask (TaskJoint & self, const Eigen::VectorXd mask){
        return self.setMask(mask);
      }
      static Eigen::VectorXd getAcceleration (TaskJoint & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskJoint & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskJoint & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskJoint & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskJoint & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskJoint & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskJoint & self){
        return self.velocity_ref();
      }     
      static const Eigen::VectorXd & Kp (TaskJoint & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (TaskJoint & self){
        return self.Kd();
      }    
      static void setKp (TaskJoint & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskJoint & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskJoint info.";
        bp::class_<TaskJoint>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskJointPosturePythonVisitor<TaskJoint>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_task_joint_hpp__
