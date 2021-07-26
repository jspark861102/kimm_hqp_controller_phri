#ifndef __kimm_hqp_controller_controller_python_solver_quadprog_hpp__
#define __kimm_hqp_controller_controller_python_solver_quadprog_hpp__

#include "kimm_hqp_controller_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller_controller/solver/solver_HQP_eiquadprog.hpp"
#include "kimm_hqp_controller_controller/solver/solver_HQP_eiquadprog_fast.hpp"
#include "kimm_hqp_controller_controller/solver/solver_HQP_output.hpp"
#include "kimm_hqp_controller_controller/solver/fwd.hpp"
#include "kimm_hqp_controller_controller/bindings/python/util/container.hpp"

namespace kimmhqp
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Solver>
    struct SolverHQuadProgPythonVisitor
    : public boost::python::def_visitor< SolverHQuadProgPythonVisitor<Solver> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        
        .def("resize", &SolverHQuadProgPythonVisitor::resize, bp::args("n", "neq", "nin"))
        .add_property("ObjVal", &Solver::getObjectiveValue, "return obj value")
        .def("solve", &SolverHQuadProgPythonVisitor::solve, bp::args("HQPData"))
        .def("solve", &SolverHQuadProgPythonVisitor::solver_helper, bp::args("HQPData for Python"))

        ;
      }
       
      static void resize(Solver & self, unsigned int n, unsigned int neq, unsigned int nin){
          self.resize(n, neq, nin);
      }  
      static solver::HQPOutput solve(Solver & self, const solver::HQPData & problemData){
          solver::HQPOutput output;
          output = self.solve(problemData);
          return output;
      }
      static solver::HQPOutput solver_helper(Solver & self, HQPDatas & HQPDatas){
          solver::HQPOutput output;
          solver::HQPData data = HQPDatas.get();

          output = self.solve(data);
         
          return output;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Solver EiQuadProg info.";
        bp::class_<Solver>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(SolverHQuadProgPythonVisitor<Solver>());       
      }
    };
  }
}


#endif // ifndef __kimm_hqp_controller_controller_python_solver_quadprog_hpp__
