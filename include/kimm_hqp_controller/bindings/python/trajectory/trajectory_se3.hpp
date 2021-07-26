

#ifndef __kimm_hqp_controller_python_traj_se3_hpp__
#define __kimm_hqp_controller_python_traj_se3_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/trajectory/trajectory_se3.hpp"
namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename TrajSE3>
    struct TrajectorySE3ConstantPythonVisitor
    : public boost::python::def_visitor< TrajectorySE3ConstantPythonVisitor<TrajSE3> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, pinocchio::SE3>((bp::arg("name"), bp::arg("reference")), "Default Constructor with name and ref_vec"))

        .add_property("size", &TrajSE3::size)
        .def("setReference", &TrajectorySE3ConstantPythonVisitor::setReference, bp::arg("M_ref"))
        .def("computeNext", &TrajectorySE3ConstantPythonVisitor::computeNext)
        .def("getLastSample", &TrajectorySE3ConstantPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectorySE3ConstantPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectorySE3ConstantPythonVisitor::getSample, bp::arg("time"))
        ;
      }
      static void setReference(TrajSE3 & self, const pinocchio::SE3 & ref){
          self.setReference(ref);
      }
      static trajectory::TrajectorySample computeNext(TrajSE3 & self){
          return self.computeNext();
      }
      static void getLastSample(const TrajSE3 & self, trajectory::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const TrajSE3 & self){
          return self.has_trajectory_ended();
      }
      static trajectory::TrajectorySample getSample(TrajSE3 & self, double time){
        return self.operator()(time);
      }
     

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory SE3 Constant info.";
        bp::class_<TrajSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectorySE3ConstantPythonVisitor<TrajSE3>());
      }
    };

    template<typename TrajSE3>
    struct TrajectorySE3CubicPythonVisitor
    : public boost::python::def_visitor< TrajectorySE3CubicPythonVisitor<TrajSE3> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, pinocchio::SE3, pinocchio::SE3, double, double>((bp::arg("name"), bp::arg("init_ref"), bp::arg("goal_ref"), bp::arg("duration"), bp::arg("start time")), "Default Constructor with name and ref_vec"))

        .add_property("size", &TrajSE3::size)
        .def("setReference", &TrajectorySE3CubicPythonVisitor::setReference, bp::arg("M_ref"))
        .def("computeNext", &TrajectorySE3CubicPythonVisitor::computeNext)
        .def("getLastSample", &TrajectorySE3CubicPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectorySE3CubicPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectorySE3CubicPythonVisitor::getSample, bp::arg("time"))

        .def("setInitSample", &TrajectorySE3CubicPythonVisitor::setInitSample, bp::arg("sample"))
        .def("setGoalSample", &TrajectorySE3CubicPythonVisitor::setGoalSample, bp::arg("sample"))
        .def("setDuration", &TrajectorySE3CubicPythonVisitor::setDuration, bp::arg("sample"))
        .def("setCurrentTime", &TrajectorySE3CubicPythonVisitor::setCurrentTime, bp::arg("sample"))
        .def("setStartTime", &TrajectorySE3CubicPythonVisitor::setStartTime, bp::arg("sample"))
        ;
      }
      static void setReference(TrajSE3 & self, const pinocchio::SE3 & ref){
          self.setReference(ref);
      }
      static trajectory::TrajectorySample computeNext(TrajSE3 & self){
          return self.computeNext();
      }
      static void getLastSample(const TrajSE3 & self, trajectory::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const TrajSE3 & self){
          return self.has_trajectory_ended();
      }
      static trajectory::TrajectorySample getSample(TrajSE3 & self, double time){
        return self.operator()(time);
      }
     
      static void setInitSample(TrajSE3 & self, const pinocchio::SE3 & ref){
        self.setInitSample(ref);
      }
      static void setGoalSample(TrajSE3 & self, const pinocchio::SE3 & ref){
        self.setGoalSample(ref);
      }
      static void setDuration(TrajSE3 & self, const double & ref){
        self.setDuration(ref);
      }
      static void setCurrentTime(TrajSE3 & self, const double & ref){
        self.setCurrentTime(ref);
      }
      static void setStartTime(TrajSE3 & self, const double & ref){
        self.setStartTime(ref);
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory SE3 cubic info.";
        bp::class_<TrajSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectorySE3CubicPythonVisitor<TrajSE3>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_se3_hpp__
