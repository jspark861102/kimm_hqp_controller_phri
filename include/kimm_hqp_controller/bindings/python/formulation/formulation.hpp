
#ifndef __kimm_hqp_controller_python_HQPOutput_hpp__
#define __kimm_hqp_controller_python_HQPOutput_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include <pinocchio/bindings/python/utils/deprecation.hpp>

#include "kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp"
#include "kimm_hqp_controller/bindings/python/solver/HQPData.hpp"

#include "kimm_hqp_controller/tasks/task_joint_posture.hpp"
#include "kimm_hqp_controller/tasks/task_se3_equality.hpp"
#include "kimm_hqp_controller/tasks/task_joint_bound.hpp"


namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct InvDynPythonVisitor
    : public boost::python::def_visitor< InvDynPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robot::RobotWrapper &, bool>((bp::args("name", "robot", "verbose"))))
        .def("data", &InvDynPythonVisitor::data)
        .add_property("nVar", &T::nVar)
        .add_property("nEq", &T::nEq)
        .add_property("nIn", &T::nIn)

        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_SE3, bp::args("task", "weight", "priorityLevel", "transition duration"))
        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_Joint, bp::args("task", "weight", "priorityLevel", "transition duration"))
        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_JointBounds, bp::args("task", "weight", "priorityLevel", "transition duration"))
       
        .def("updateTaskWeight", &InvDynPythonVisitor::updateTaskWeight, bp::args("task_name", "weight")) 
        .def("removeTask", &InvDynPythonVisitor::removeTask, bp::args("task_name", "duration"))
       
        .def("removeFromHqpData", &InvDynPythonVisitor::removeFromHqpData, bp::args("constraint_name"))
        .def("computeProblemData", &InvDynPythonVisitor::computeProblemData, bp::args("time", "q", "v"))
        
        .def("getActuatorForces", &InvDynPythonVisitor::getActuatorForces, bp::args("HQPOutput"))
        .def("getAccelerations", &InvDynPythonVisitor::getAccelerations, bp::args("HQPOutput"))
        ;
      }
      static pinocchio::Data data(T & self){
        pinocchio::Data data = self.data();
        return data;
      }
      static bool addMotionTask_SE3(T & self, tasks::TaskSE3Equality & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
      static bool addMotionTask_Joint(T & self, tasks::TaskJointPosture & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
      static bool addMotionTask_JointBounds(T & self, tasks::TaskTorqueBounds & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
     
      static bool updateTaskWeight(T& self, const std::string & task_name, double weight){
        return self.updateTaskWeight(task_name, weight);
      }     
      static bool removeTask(T& self, const std::string & task_name, double transition_duration){
        return self.removeTask(task_name, transition_duration);
      }

      static bool removeFromHqpData(T& self, const std::string & constraintName){
        return self.removeFromHqpData(constraintName);
      }
      static HQPDatas computeProblemData(T& self, double time, const Eigen::VectorXd & q, const Eigen::VectorXd & v){
        HQPDatas Hqp;
        Hqp.set(self.computeProblemData(time, q, v));
        return Hqp;
      } 
      static Eigen::VectorXd getActuatorForces (T & self, const solver::HQPOutput & sol){
        return self.getActuatorForces(sol);
      }
      static Eigen::VectorXd getAccelerations (T & self, const solver::HQPOutput & sol){
        return self.getAccelerations(sol);
      }
    
      static void expose(const std::string & class_name)
      {
        std::string doc = "InvDyn info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(InvDynPythonVisitor<T>());       
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_HQPOutput_hpp__
