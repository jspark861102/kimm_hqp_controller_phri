#ifndef __kimm_python_robot_wrapper_hpp__
#define __kimm_python_robot_wrapper_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"

namespace kimmhqp
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Robot>
    struct RobotPythonVisitor
    : public boost::python::def_visitor< RobotPythonVisitor<Robot> >
    {
      typedef std::vector<std::string> std_vec;
      typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;

      template<class PyClass >

      void visit(PyClass& cl) const
      {
        cl
       .def(bp::init<std::string, std_vec, bool, bool>((bp::arg("filename"), bp::arg("package_dir"), bp::arg("is_mobile"), bp::arg("verbose")), "Default constructor without RootJoint."))
        .add_property("nq", &Robot::nq)
        .add_property("nv", &Robot::nv)
        .add_property("na", &Robot::na)
        .add_property("model", &RobotPythonVisitor::model)

        //.def("model", &RobotPythonVisitor::model)
        .def("data", &RobotPythonVisitor::data)
        .def("ismobile", RobotPythonVisitor::ismobile)

        .def("computeAllTerms", &RobotPythonVisitor::computeAllTerms, bp::args("data", "q", "v"), "compute all dynamics")
        .def("mass", &RobotPythonVisitor::mass, bp::arg("data"))
        .def("nonLinearEffect", &RobotPythonVisitor::nonLinearEffects, bp::arg("data"))
        .def("position", &RobotPythonVisitor::position, bp::args("data", "index"))
        .def("velocity", &RobotPythonVisitor::velocity, bp::args("data", "index"))
        .def("acceleration", &RobotPythonVisitor::acceleration, bp::args("data", "index"))

        .def("framePosition", &RobotPythonVisitor::framePosition, bp::args("data", "index"))
        .def("frameVelocity", &RobotPythonVisitor::frameVelocity, bp::args("data", "index"))
        .def("frameAcceleration", &RobotPythonVisitor::frameAcceleration, bp::args("data", "index"))
        .def("frameClassicAcceleration", &RobotPythonVisitor::frameClassicAcceleration, bp::args("data", "index"))
        ;
      }

      static bool ismobile (const Robot & self){
        return self.ismobile();
      }
      static pinocchio::Model model (const Robot & self){
        return self.model();
      }
      static pinocchio::Data data(const Robot & self){
        pinocchio::Data data(self.model());
        return data;
      }
      
      static void computeAllTerms (Robot & self, pinocchio::Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v){
        return self.computeAllTerms(data, q, v);
      }
      static Eigen::MatrixXd mass (Robot & self, pinocchio::Data & data){
        return self.mass(data);
      }
      static Eigen::VectorXd nonLinearEffects(Robot & self, const pinocchio::Data & data){
        return self.nonLinearEffects(data);
      }
      static pinocchio::SE3 position(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.position(data, index);
      }
      static pinocchio::Motion velocity(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.velocity(data, index);
      }
      static pinocchio::Motion acceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.acceleration(data, index);
      }
      static pinocchio::SE3 framePosition(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.framePosition(data, index);
      }
      static pinocchio::Motion frameVelocity(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameVelocity(data, index);
      }
      static pinocchio::Motion frameAcceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameAcceleration(data, index);
      }
      static pinocchio::Motion frameClassicAcceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameClassicAcceleration(data, index);
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Robot Wrapper info.";
        bp::class_<Robot>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(RobotPythonVisitor<Robot>());
        ;
      }
    };
  }
}


#endif // ifndef __tsid_python_robot_wrapper_hpp__
