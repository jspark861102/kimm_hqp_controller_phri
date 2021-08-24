#include "kimm_hqp_controller/solver/solver_HQP_whcod.hpp"
#include "kimm_hqp_controller/math/util.hpp"
#include "kimm_hqp_controller/util/stop_watch.hpp"

using namespace kimmhqp::math;
using namespace kimmhqp::solver;
using namespace Eigen;
using namespace std;

SolverHQPWhcod::SolverHQPWhcod(const std::string & name):
  SolverHQPBase(name)
{
  m_initset = new Initset();
  m_iHQP = new iHQP_solver();
}

void SolverHQPWhcod::sendMsg(const std::string & s)
{
  std::cout<<"[SolverHQPWhcod."<<m_name<<"] "<<s<<std::endl;
}

const HQPOutput & SolverHQPWhcod::solve(const HQPData & problemData)
{
  assert("false");  
  return m_output;
}

const HQPOutput & SolverHQPWhcod::solve(const WHQPData & problemData)
{
  vector<unsigned int> neq_vec, nin_vec, n_task_vec;
  unsigned int level = 0, r_level = 0;
  level = problemData.size();

  for (int i=0; i<level; i++){
    const WConstraintLevel & cl = problemData[i];
    unsigned int neq=0, nin=0, ntask=0;
    if (cl.size()>0){
      const unsigned int n = cl[0].second->cols();
       for(WConstraintLevel::const_iterator it=cl.begin(); it!=cl.end(); it++)
        {
          auto constr = it->second;
          assert(n==constr->cols());
          if(constr->isEquality())
            neq += constr->rows();
          else
            nin += constr->rows();

          ntask++;
        }
        neq_vec.push_back(neq);
        nin_vec.push_back(nin);
        n_task_vec.push_back(ntask);
        r_level++;
    }
  }

  m_A.resize(r_level);
  m_b.resize(r_level);
  m_W.resize(r_level);
  m_btype.resize(r_level);

  unsigned int c_level = 0;
  for (int i=0; i<level; i++){
    const WConstraintLevel & cl = problemData[i];
    unsigned int idx = 0, task_idx = 0;

    if (cl.size()>0){
      const unsigned int n = cl[0].second->cols();
      m_A_sub.resize(neq_vec[c_level]+nin_vec[c_level], n);
      m_b_sub.resize(neq_vec[c_level]+nin_vec[c_level], 2);

      m_btype_sub.setOnes(neq_vec[c_level]+nin_vec[c_level]);

      m_W_sub.resize(n *r_level , n);

       
      for(WConstraintLevel::const_iterator it=cl.begin(); it!=cl.end(); it++)
      {
        
        auto constr = it->second;
        if(constr->isEquality()){
          m_A_sub.block(idx, 0, constr->rows(), n) = constr->matrix();
          m_b_sub.col(0).segment(idx, constr->rows()) = constr->vector();
          m_b_sub.col(1).segment(idx, constr->rows()) = constr->vector();
        }
        else if (constr->isInequality()){
          m_A_sub.block(idx, 0, constr->rows(), n) = constr->matrix();
          m_b_sub.col(0).segment(idx, constr->rows()) = constr->lowerBound();
          m_b_sub.col(1).segment(idx, constr->rows()) = constr->upperBound();
          m_btype_sub.segment(idx, constr->rows()) = Eigen::VectorXi::Ones(constr->rows())*2;
        }
        else{
          m_A_sub.block(idx, 0, constr->rows(), n) = Eigen::MatrixXd::Identity(constr->rows(), constr->rows());
          m_b_sub.col(0).segment(idx, constr->rows()) = constr->lowerBound();
          m_b_sub.col(1).segment(idx, constr->rows()) = constr->upperBound();
          m_btype_sub.segment(idx, constr->rows()) = Eigen::VectorXi::Ones(constr->rows())*2;
        }

      //  m_W_sub.block(task_idx * n, 0, n, n).setIdentity();
        
        idx += constr->rows();
        task_idx++;
      }
      m_A[c_level] = m_A_sub;
      m_b[c_level] = m_b_sub;
      m_btype[c_level]= m_btype_sub;
      m_W[c_level] = m_W_sub.setIdentity(n, n);

      c_level++;
    }
  }

  // for (int i=0; i<r_level; i++){
  //   cout << "m_A " << i << " " <<  m_A[i] << endl;
  //   cout << "m_b" << i <<  " " << m_b[i] << endl;
  //   cout << "m_btype" << i  << " " << m_btype[i].transpose() << endl;
  //   cout << "m_W" << i  << " " << m_W[i] << endl;
  // }

  m_initset->set_btype(m_btype);
  // cout << " m_initset" << endl;
  m_iHQP->initialized(m_A, m_b, m_btype, m_initset->getactiveset(), m_initset->getbounds(), m_W);  
  // cout << " hi" << endl;
  m_output.x = m_iHQP->solve();

  return m_output;
}

double SolverHQPWhcod::getObjectiveValue()
{
  return m_objValue;
}