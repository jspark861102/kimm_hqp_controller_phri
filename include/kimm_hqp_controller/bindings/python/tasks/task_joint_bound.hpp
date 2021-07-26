#ifndef __kimm_hqp_controller_python_task_actuation_bounds_hpp__
#define __kimm_hqp_controller_python_task_actuation_bounds_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/tasks/task_joint_bound.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"
#include "kimm_hqp_controller/math/constraint_inequality.hpp"
#include "kimm_hqp_controller/math/constraint_base.hpp"

namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename Task>
    struct TaskJointBoundsPythonVisitor
    : public boost::python::def_visitor< TaskJointBoundsPythonVisitor<Task> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robot::RobotWrapper &> ((bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &Task::dim, "return dimension size")
        //.add_property("mask", bp::make_function(&TaskJointBoundsPythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
       // .def("setMask", &TaskJointBoundsPythonVisitor::setmask, bp::arg("mask"))
        .def("setBounds", &TaskJointBoundsPythonVisitor::setBounds, bp::args("lower", "upper"))
        .def("compute", &TaskJointBoundsPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskJointBoundsPythonVisitor::getConstraint)
        .add_property("getLowerBounds", bp::make_function(&TaskJointBoundsPythonVisitor::getLowerBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getUpperBounds", bp::make_function(&TaskJointBoundsPythonVisitor::getUpperBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("name", &TaskJointBoundsPythonVisitor::name)
        ;
      }
      static std::string name(Task & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintInequality compute(Task & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintInequality cons(self.getConstraint().name(), self.getConstraint().matrix(),
                                        self.getConstraint().lowerBound(), self.getConstraint().upperBound());
        return cons;
      }
      static math::ConstraintInequality getConstraint(const Task & self){
        math::ConstraintInequality cons(self.getConstraint().name(), self.getConstraint().matrix(),
                                        self.getConstraint().lowerBound(), self.getConstraint().upperBound());
        return cons;
      }
      // static const Eigen::VectorXd & getmask(const Task & self){
      //   return self.mask();
      // }
      // static void setmask (Task & self, const Eigen::VectorXd mask){
      //   return self.mask(mask);
      // }
      static const Eigen::VectorXd & getLowerBounds (const Task & self){
        return self.getTorqueLowerBounds();
      }  
      static const Eigen::VectorXd & getUpperBounds (const Task & self){
        return self.getTorqueUpperBounds();
      }    
      static void setBounds (Task & self, const Eigen::VectorXd lower, const Eigen::VectorXd upper){
        return self.setTorqueBounds(lower, upper);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "Task info.";
        bp::class_<Task>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskJointBoundsPythonVisitor<Task>());
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_task_actuation_bounds_hpp__