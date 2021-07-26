#include "kimm_hqp_controller/bindings/python/fwd.hpp"
#include <eigenpy/geometry.hpp>

#include "kimm_hqp_controller/bindings/python/robot/expose_robot.hpp"
#include "kimm_hqp_controller/bindings/python/constraint/expose_constraint.hpp"
// #include "kimm_hqp_controller/bindings/python/contacts/expose-contact.hpp"
#include "kimm_hqp_controller/bindings/python/trajectory/expose_trajectory.hpp"
#include "kimm_hqp_controller/bindings/python/tasks/expose_tasks.hpp"
#include "kimm_hqp_controller/bindings/python/solver/expose_solver.hpp"
#include "kimm_hqp_controller/bindings/python/formulation/expose_formulation.hpp"

namespace bp = boost::python;
using namespace kimmhqp::python;

BOOST_PYTHON_MODULE(libkimm_hqp_controller_pywrap)
{
  eigenpy::enableEigenPy();
  eigenpy::exposeAngleAxis();
  eigenpy::exposeQuaternion();

  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
  
  eigenpy::enableEigenPySpecific<Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d>();
  eigenpy::enableEigenPySpecific<Matrix6x>();
  eigenpy::enableEigenPySpecific<Matrix3x>();
  eigenpy::enableEigenPySpecific<Eigen::MatrixXd>();
  eigenpy::enableEigenPySpecific<Eigen::Vector3d>();
  
  exposeRobots();  
  exposeConstraints();
//   exposeContact();
  exposeTrajectories();
  exposeTasks();
  exposeSolver();
  exposeFormulation();

}