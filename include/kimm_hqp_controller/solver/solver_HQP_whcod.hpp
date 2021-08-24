#ifndef __solver_hqp_whcod_hpp__
#define __solver_hqp_whcod_hpp__

#include "kimm_hqp_controller/solver/solver_HQP_base.hpp"
#include "weighted_hqp/iHQP_solver.hpp"
#include "weighted_hqp/InitSet.hpp"

using namespace hcod;

namespace kimmhqp{
    namespace solver{
        class TSID_DLLAPI SolverHQPWhcod:
        public SolverHQPBase
        {
            public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            typedef math::Matrix Matrix;
            typedef math::Vector Vector;
            typedef math::RefVector RefVector;
            typedef math::ConstRefVector ConstRefVector;
            typedef math::ConstRefMatrix ConstRefMatrix;

            SolverHQPWhcod(const std::string & name);

            void resize(unsigned int n, unsigned int neq, unsigned int nin){};

            /** Solve the given Hierarchical Quadratic Program
             */
            const HQPOutput & solve(const HQPData & problemData);
            const HQPOutput & solve(const WHQPData & problemData);
            /** Get the objective value of the last solved problem. */
            double getObjectiveValue();

            protected:

            void sendMsg(const std::string & s);

            unsigned int m_neq;  /// number of equality constraints
            unsigned int m_nin;  /// number of inequality constraints
            unsigned int m_n;    /// number of variables

            protected:
            double m_objValue;
            std::vector<Eigen::MatrixXd> m_A;
            Eigen::MatrixXd m_A_sub, m_b_sub, m_W_sub;
            Eigen::VectorXi m_btype_sub;
            std::vector<Eigen::MatrixXd> m_b;
            std::vector<Eigen::VectorXi> m_btype;
            std::vector<Eigen::MatrixXd> m_W;

            Initset* m_initset;
            iHQP_solver* m_iHQP;
        };

    }
}

#endif