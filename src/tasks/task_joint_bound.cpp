#include "kimm_hqp_controller/tasks/task_joint_bound.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectory;
    using namespace pinocchio;

    TaskTorqueBounds::TaskTorqueBounds(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_constraint(name, robot.nv()),
      m_nv(robot.nv()),
      m_na(robot.na()),
      m_mobile(robot.ismobile())
    {
      if (!m_mobile){
        int offset = m_nv-m_na;
        for(int i=0; i<offset; i++)
        {
          m_constraint.upperBound()(i) = 1e10;
          m_constraint.lowerBound()(i) = -1e10;
        }
      }
      else{
        m_constraint.resize(m_nv-3, m_nv-3);
      }
    }

    int TaskTorqueBounds::dim() const
    { return m_nv; }

    void TaskTorqueBounds::setTorqueBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_torque_lb = lower;
      m_torque_ub = upper;
    }

    const Vector & TaskTorqueBounds::getTorqueLowerBounds() const{
      return m_torque_lb;
    }
    const Vector & TaskTorqueBounds::getTorqueUpperBounds() const{
      return m_torque_ub;
    }

    const ConstraintBase & TaskTorqueBounds::getConstraint() const
    {
      return m_constraint;
    }

    void TaskTorqueBounds::setMask(ConstRefVector mask)
    {
      m_mask = mask;
    }

    const ConstraintBase & TaskTorqueBounds::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data &)
    {
      int offset = m_nv-m_na;
      if (m_mobile)
        offset = 0;

      for(int i=0; i<m_na; i++)
      {
        // TODO: use mask here
        m_constraint.upperBound()(offset+i) = m_torque_ub(i); //std::min(m_ddq_max_due_to_vel(i), m_a_ub(i));
        m_constraint.lowerBound()(offset+i) = m_torque_lb(i); // std::max(m_ddq_min_due_to_vel(i), m_a_lb(i));
      }
            
      return m_constraint;
    }
    
  }
}
