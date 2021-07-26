#ifndef __kimm_hqp_controller_python_util_container_hpp__
#define __kimm_hqp_controller_python_util_container_hpp__

#include "kimm_hqp_controller/bindings/python/fwd.hpp"

#include "kimm_hqp_controller/solver/fwd.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/math/constraint_inequality.hpp"
#include "kimm_hqp_controller/math/constraint_bound.hpp"

using namespace std;
namespace kimmhqp
{
  namespace python
  {         
    typedef solver::ConstraintLevel ConstraintLevel;
    typedef solver::HQPData HQPData;

    class ConstraintLevels
    {
    public:
        ConstraintLevels(){}
        ~ConstraintLevels(){}

        inline void print (){
            stringstream ss;
            for(ConstraintLevel::const_iterator iit=m_std_const.begin(); iit!=m_std_const.end(); iit++)
            {
                auto c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
            }
            cout << ss.str() << endl;
        }
        inline ConstraintLevel& get (){
            return m_std_const;
        }
    
        inline void append_eq (double num, std::shared_ptr<math::ConstraintEquality> i){
           m_std_const.push_back(solver::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }
        inline void append_ineq (double num, std::shared_ptr<math::ConstraintInequality> i){
           m_std_const.push_back(solver::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }
        inline void append_bound (double num, std::shared_ptr<math::ConstraintBound> i){
           m_std_const.push_back(solver::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }
    private:
        ConstraintLevel m_std_const;
    };

    class HQPDatas
    {
    public:
        HQPDatas(){}
        ~HQPDatas(){}

        inline void resize (size_t i) {
           m_std_hqp.resize(i);
        }

        inline void print () const {
            stringstream ss;
            unsigned int priority = 0;
            for(HQPData::const_iterator it=m_std_hqp.begin(); it!=m_std_hqp.end(); it++)
            {
                ss<<"Level "<< priority<<endl;
                for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
                {
                auto c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
                }
                priority++;
            }
             cout << ss.str() << endl;
        }
        // inline void append (ConstraintLevel cons){
        //     m_std_hqp.push_back(cons);
        // }
        inline void append_helper (ConstraintLevels* cons){
            m_std_hqp.push_back(cons->get());
        }

        inline HQPData get (){
            return m_std_hqp;
        }
        inline bool set (HQPData data){
            m_std_hqp = data;
            return true;
        }
    
    private:
        HQPData m_std_hqp;
    };
  }
}


#endif // ifndef __kimm_hqp_controller_python_util_container_hpp__