
#ifndef __kimm_hqp_controller_python_HQPOutput_hpp__
#define __kimm_hqp_controller_python_HQPOutput_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/solver/solver_HQP_output.hpp"

namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct HQPOutputPythonVisitor
    : public boost::python::def_visitor< HQPOutputPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Defulat Constructor"))
        .def(bp::init<int, int, int>((bp::args("nVars", "nEq", "nInCon"))))
        .add_property("x", &HQPOutputPythonVisitor::x)
        .add_property("status", &HQPOutputPythonVisitor::status)
        ;
      }
      static Eigen::VectorXd x (const T & self) {return self.x;}
      static int status (const T & self) {return self.status;}
      static void expose(const std::string & class_name)
      {
        std::string doc = "HQPOutput info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(HQPOutputPythonVisitor<T>());       
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_HQPOutput_hpp__
