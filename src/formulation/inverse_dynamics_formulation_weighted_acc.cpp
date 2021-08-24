#include "kimm_hqp_controller/formulation/inverse_dynamics_formulation_weighted_acc.hpp"

using namespace kimmhqp;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace contacts;
using namespace std;

typedef pinocchio::Data Data;

InverseDynamicsFormulationWeightedAccForce::InverseDynamicsFormulationWeightedAccForce(const std::string & name, RobotWrapper & robot, bool verbose):
    InverseDynamicsFormulationBase(name, robot, verbose),
    m_data(robot.model()),
    m_baseDynamics(new math::ConstraintEquality("base-dynamics", 6, robot.nv())),
    m_solutionDecoded(false)
    {
    m_mobile = robot.ismobile();

    m_t = 0.0;
    if (m_mobile)
      m_v = robot.nv() - 3;     
    else
      m_v = robot.nv();
    m_k = 0;

    m_eq = 0;
    m_in = 0;
    m_hqpData.resize(5);
    m_Jc.setZero(m_k, m_v);

    
    //m_hqpData[0].push_back(solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(1.0, m_baseDynamics));
    }


Data & InverseDynamicsFormulationWeightedAccForce::data()
{
  return m_data;
}

unsigned int InverseDynamicsFormulationWeightedAccForce::nVar() const
{
  return m_v+m_k;
}

unsigned int InverseDynamicsFormulationWeightedAccForce::nEq() const
{
  return m_eq;
}

unsigned int InverseDynamicsFormulationWeightedAccForce::nIn() const
{
  return m_in;
}


void InverseDynamicsFormulationWeightedAccForce::resizeHqpData()
{
  m_Jc.setZero(m_k, m_v);
  //m_baseDynamics->resize(m_u, m_v+m_k);
  for(WHQPData::iterator it=m_hqpData.begin(); it!=m_hqpData.end(); it++)
  {
    for(WConstraintLevel::iterator itt=it->begin(); itt!=it->end(); itt++)
    {
      itt->second->resize(itt->second->rows(), m_v+m_k);
    }
  }
}

template<class TaskLevelPointer>
void InverseDynamicsFormulationWeightedAccForce::addTask(TaskLevelPointer tl, Eigen::VectorXd weight, unsigned int priorityLevel)
{
  if(priorityLevel > m_hqpData.size())
    m_hqpData.resize(priorityLevel);
  const ConstraintBase & c = tl->task.getConstraint();
  if(c.isEquality())
  {
    tl->constraint = std::make_shared<ConstraintEquality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_eq += c.rows();
  }
  else //if(c.isInequality())
  {
    tl->constraint = std::make_shared<ConstraintInequality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_in += c.rows();
  }
  m_hqpData[priorityLevel].push_back(make_pair<Eigen::VectorXd, std::shared_ptr<ConstraintBase> >(weight, tl->constraint));
}

template<class TaskLevelPointer>
void InverseDynamicsFormulationWeightedAccForce::addTask(TaskLevelPointer tl, double weight, unsigned int priorityLevel)
{
  this->addTask(tl, Eigen::VectorXd::Ones(m_v) * weight, priorityLevel);
}

bool InverseDynamicsFormulationWeightedAccForce::addMotionTask(TaskMotion & taskmotion, Eigen::VectorXd weight, unsigned int priorityLevel, double transition_duration)
{
  assert(weight.size() == m_v); // check for mobile
  assert(transition_duration>=0.0);
  
  // This part is not used frequently so we can do some tests.
  if (transition_duration<0.0)
    std::cerr << "transition_duration should be positive" << std::endl;

  auto tl = std::make_shared<TaskLevel>(taskmotion, priorityLevel);
  m_taskMotions.push_back(tl);
  addTask(tl, weight, priorityLevel);

  return true;
}

bool InverseDynamicsFormulationWeightedAccForce::addMotionTask(TaskMotion & taskmotion, double weight, unsigned int priorityLevel, double transition_duration)
{
  return this->addMotionTask(taskmotion, Eigen::VectorXd::Ones(m_v) * weight, priorityLevel, transition_duration);
}

bool InverseDynamicsFormulationWeightedAccForce::updateTaskWeight(const std::string & task_name, Eigen::VectorXd weight)
{
  assert(weight.size() == m_v);
  WConstraintLevel::iterator it;
  for(unsigned int i=0; i<m_hqpData.size(); i++)
  {
    for(it=m_hqpData[i].begin(); it!=m_hqpData[i].end(); it++)
    {
      if(it->second->name() == task_name)
      {
        it->first = weight;
        return true;
      }
    }
  }
  return false;
}
bool InverseDynamicsFormulationWeightedAccForce::updateTaskWeight(const std::string & task_name, double weight)
{
  return this->updateTaskWeight(task_name, Eigen::VectorXd::Ones(m_v) * weight);
}

const WHQPData & InverseDynamicsFormulationWeightedAccForce::computeProblemData(double time, ConstRefVector q, ConstRefVector v)
{
  m_t = time;

  m_robot.computeAllTerms(m_data, q, v);

  // ggory15 
  for (auto& it : m_taskMotions)
  {
    const ConstraintBase & c = it->task.compute(time, q, v, m_data);
    
    if(c.isEquality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->vector() = c.vector();
    }
    else if(c.isInequality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
    else
    {
      it->constraint->matrix().leftCols(m_v) = Matrix::Identity(m_v, m_v);
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
  }
  // cout << solver::HQPDataToString(m_hqpData, true) << endl;
  m_solutionDecoded = false;
  
  return m_hqpData;
}

bool InverseDynamicsFormulationWeightedAccForce::decodeSolution(const HQPOutput & sol)
{
  if(m_solutionDecoded)
    return true;

  if (!m_mobile){
    const Matrix & M_a = m_robot.mass(m_data);
    const Vector & h_a = m_robot.nonLinearEffects(m_data);
    const Matrix & J_a = m_Jc.rightCols(m_v);
    
    m_dv = sol.x.head(m_v);
    m_f = sol.x.tail(m_k);
    
    m_tau = h_a;
    m_tau.noalias() += M_a * m_dv;
  // m_tau.noalias() -= J_a.transpose() * m_f;
    m_solutionDecoded = true;
    return true;
  }
  else{
    m_tau.setZero(m_v - 2);
    const Matrix & M_a = m_robot.mass(m_data).bottomRows(m_v- 2);
    const Vector & h_a = m_robot.nonLinearEffects(m_data).tail(m_v-2);

    // const Matrix & J_a = m_Jc.rightCols(m_v);
    
    m_dv = sol.x.head(m_v );
    m_f = sol.x.tail(m_k);
  
    m_tau = h_a;
    m_tau.noalias() += M_a * m_dv;
   
  // m_tau.noalias() -= J_a.transpose() * m_f;
    m_solutionDecoded = true;
    return true;
  }
}

const Vector & InverseDynamicsFormulationWeightedAccForce::getAccelerations(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_dv;
}

const Vector & InverseDynamicsFormulationWeightedAccForce::getActuatorForces(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_tau;
}

bool InverseDynamicsFormulationWeightedAccForce::removeTask(const std::string & taskName,
                                                    double )
{
#ifndef NDEBUG
  bool taskFound = removeFromHqpData(taskName);
  //assert(taskFound);
#else
  removeFromHqpData(taskName);
#endif
  
  for(auto it=m_taskMotions.begin(); it!=m_taskMotions.end(); it++)
  {
    if((*it)->task.name()==taskName)
    {
      if((*it)->priority==0)
      {
        if((*it)->constraint->isEquality())
          m_eq -= (*it)->constraint->rows();
        else if((*it)->constraint->isInequality())
          m_in -= (*it)->constraint->rows();
      }
      m_taskMotions.erase(it);
      return true;
    }
  }
 
  return false;
}
void InverseDynamicsFormulationWeightedAccForce::resetHqpData(){
  std::vector<std::shared_ptr<TaskLevel> > new_tasks;
  m_taskMotions = new_tasks;
  WHQPData new_Data;
  m_hqpData = new_Data;
  m_hqpData.resize(5);
  m_eq = 0;
  m_in = 0;      
}

bool InverseDynamicsFormulationWeightedAccForce::removeFromHqpData(const std::string & name)
{
  bool found = false;
  for(WHQPData::iterator it=m_hqpData.begin(); !found && it!=m_hqpData.end(); it++)
  {
    for(WConstraintLevel::iterator itt=it->begin(); !found && itt!=it->end(); itt++)
    {
      if(itt->second->name()==name)
      {
        it->erase(itt);
        return true;
      }
    }
  }
  return false;
}
