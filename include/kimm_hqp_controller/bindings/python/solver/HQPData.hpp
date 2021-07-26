#ifndef __kimm_hqp_controller_python_test_hpp__
#define __kimm_hqp_controller_python_test_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"
#include "kimm_hqp_controller/bindings/python/util/container.hpp"

namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct ConstPythonVisitor
    : public boost::python::def_visitor< ConstPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("append", &T::append_eq, bp::arg("data"))
        .def("append", &T::append_ineq, bp::arg("data"))
        .def("append", &T::append_bound, bp::arg("data"))  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "ConstraintLevel info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ConstPythonVisitor<T>());       
      }
    };

    template<typename T>
    struct HQPPythonVisitor
    : public boost::python::def_visitor< HQPPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("resize", &T::resize, bp::arg("i"))
        .def("append", &T::append_helper, bp::arg("constraintLevel"))  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "HQPdata info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(HQPPythonVisitor<T>());       
      }
    };

  }
}


#endif // ifndef __kimm_hqp_controller_python_test_hpp__
