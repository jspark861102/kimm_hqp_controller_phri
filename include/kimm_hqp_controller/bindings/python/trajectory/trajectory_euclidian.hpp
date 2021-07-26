#ifndef __kimm_hqp_controller_python_traj_euclidian_hpp__
#define __kimm_hqp_controller_python_traj_euclidian_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/trajectory/trajectory_euclidian.hpp"
namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Traj>
    struct TrajectoryEuclidianConstantPythonVisitor
    : public boost::python::def_visitor< TrajectoryEuclidianConstantPythonVisitor<Traj> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, Eigen::VectorXd>((bp::arg("name"), bp::arg("reference")), "Default Constructor with name and ref_vec"))

        .add_property("size", &Traj::size)
        .def("setReference", &TrajectoryEuclidianConstantPythonVisitor::setReference, bp::arg("ref_vec"))
        .def("computeNext", &TrajectoryEuclidianConstantPythonVisitor::computeNext)
        .def("getLastSample", &TrajectoryEuclidianConstantPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectoryEuclidianConstantPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectoryEuclidianConstantPythonVisitor::getSample, bp::arg("time"))
        ;
      }
      static void setReference(Traj & self, const Eigen::VectorXd & ref){
          self.setReference(ref);
      }
      static trajectory::TrajectorySample computeNext(Traj & self){
          return self.computeNext();
      }
      static void getLastSample(const Traj & self, trajectory::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const Traj & self){
          return self.has_trajectory_ended();
      }
      static trajectory::TrajectorySample getSample(Traj & self, double time){
        return self.operator()(time);
      }
     

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory Euclidian Constant info.";
        bp::class_<Traj>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectoryEuclidianConstantPythonVisitor<Traj>());
      }
    };

    template<typename Traj>
    struct TrajectoryEuclidianCubicPythonVisitor
    : public boost::python::def_visitor< TrajectoryEuclidianCubicPythonVisitor<Traj> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, Eigen::VectorXd, Eigen::VectorXd, double, double>((bp::arg("name"), bp::arg("init_ref"), bp::arg("goal_ref"), bp::arg("duration"), bp::arg("start time")), "Default Constructor with name and ref_vec"))

        .add_property("size", &Traj::size)
        .def("setReference", &TrajectoryEuclidianCubicPythonVisitor::setReference, bp::arg("ref_vec"))
        .def("computeNext", &TrajectoryEuclidianCubicPythonVisitor::computeNext)
        .def("getLastSample", &TrajectoryEuclidianCubicPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectoryEuclidianCubicPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectoryEuclidianCubicPythonVisitor::getSample, bp::arg("time"))

        .def("setInitSample", &TrajectoryEuclidianCubicPythonVisitor::setInitSample, bp::arg("sample"))
        .def("setGoalSample", &TrajectoryEuclidianCubicPythonVisitor::setGoalSample, bp::arg("sample"))
        .def("setDuration", &TrajectoryEuclidianCubicPythonVisitor::setDuration, bp::arg("sample"))
        .def("setCurrentTime", &TrajectoryEuclidianCubicPythonVisitor::setCurrentTime, bp::arg("sample"))
        .def("setStartTime", &TrajectoryEuclidianCubicPythonVisitor::setStartTime, bp::arg("sample"))
        
        ;
      }
      static void setReference(Traj & self, const Eigen::VectorXd & ref){
          self.setReference(ref);
      }
      static trajectory::TrajectorySample computeNext(Traj & self){
          return self.computeNext();
      }
      static void getLastSample(const Traj & self, trajectory::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const Traj & self){
          return self.has_trajectory_ended();
      }
      static trajectory::TrajectorySample getSample(Traj & self, double time){
        return self.operator()(time);
      }

      static void setInitSample(Traj & self, const Eigen::VectorXd & ref){
        self.setInitSample(ref);
      }
      static void setGoalSample(Traj & self, const Eigen::VectorXd & ref){
        self.setGoalSample(ref);
      }
      static void setDuration(Traj & self, const double & ref){
        self.setDuration(ref);
      }
      static void setCurrentTime(Traj & self, const double & ref){
        self.setCurrentTime(ref);
      }
      static void setStartTime(Traj & self, const double & ref){
        self.setStartTime(ref);
      }




      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory Euclidian Cubic info.";
        bp::class_<Traj>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectoryEuclidianCubicPythonVisitor<Traj>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_euclidian_hpp__