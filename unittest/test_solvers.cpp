#include <iostream>
#include <cstdio>
#include <Eigen/Core>

#include <kimm_hqp_controller/robot/robot_wrapper.hpp>
#include <kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <kimm_hqp_controller/tasks/task_se3_equality.hpp>
#include <kimm_hqp_controller/tasks/task_joint_posture.hpp>
#include <kimm_hqp_controller/tasks/task_joint_bound.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>

#include <kimm_hqp_controller/solver/solver_HQP_factory.hxx>
#include <kimm_hqp_controller/solver/util.hpp>
#include <kimm_hqp_controller/math/util.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate


using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;
using namespace std;

std::shared_ptr<RobotWrapper> robot_;
std::shared_ptr<InverseDynamicsFormulationAccForce> tsid_;
std::shared_ptr<TaskJointPosture> postureTask_;
std::shared_ptr<TaskTorqueBounds> torqueBoundsTask_;
std::shared_ptr<TaskSE3Equality> eeTask_;

VectorXd q_, v_;
double time_;
int main(int argc, char **argv){
    string model_path, urdf_name;
    model_path = "/home/ggory15/kimm_catkin/src/kimm_robots_description/franka_panda_description/";
    urdf_name = "/franka_single/panda_arm_l.urdf";
    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false, false);

    const Model & model = robot_->model();
    int na = robot_->na(); // the number of joint

    q_.setZero(na);
    v_.setZero(na);
    time_ = 0;

    tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
    tsid_->computeProblemData(time_, q_, v_);
    pinocchio::Data & data = tsid_->data();

    postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
    postureTask_->Kp(5000.0*Vector::Ones(na));
    postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

    torqueBoundsTask_ = std::make_shared<TaskTorqueBounds>("task-torque-bounds", *robot_);
    Vector dq_max = 0.01*Vector::Ones(na);
    Vector dq_min = -dq_max;
    torqueBoundsTask_->setTorqueBounds(dq_min, dq_max);

    eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7");
    eeTask_->Kp(5000.0*Vector::Ones(6));
    eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());    

    VectorXd Maxvel = Vector::Ones(na);
    VectorXd Maxacc = Vector::Ones(na);
    auto trajPosture = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture", Maxvel, Maxacc);
    TrajectorySample samplePosture(na);

    Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
    Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
    auto trajEE = std::make_shared<TrajectorySE3Timeopt>("traj_ee", Maxvel_ee, Maxacc_ee);
    TrajectorySample sampleEE(12, 6);

    //Solver
    SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_WHCOD, "whcod");
    SE3 H_ee_ref;

    int max_it = 5000;
    double dt = 0.001;
    for (int i=0; i<max_it; i++){
        if (i == 0){
            tsid_->addMotionTask(*postureTask_, 1e-2, 2);
            tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                        
            // Set Desired Posture
            trajPosture->clearWaypoints();
            trajPosture->setStartTime(time_);
            trajPosture->addWaypoint(q_);
            Vector q_ref(7);
            q_ref.setOnes();
            trajPosture->addWaypoint(q_ref); //first add point
        }

        if (i == 1000){
            tsid_->addMotionTask(*eeTask_, 1, 1);
            trajEE->clearWaypoints();
            trajEE->setStartTime(time_);
            H_ee_ref = robot_->position(data, robot_->model().getJointId("panda_joint7"));
            trajEE->addWaypoint(H_ee_ref);
            H_ee_ref.translation()(0) += 0.10;
            trajEE->addWaypoint(H_ee_ref);
        }

        trajPosture->setCurrentTime(time_);            
        samplePosture = trajPosture->computeNext();       
        postureTask_->setReference(samplePosture);
        
        if (i>=1000){
            trajEE->setCurrentTime(time_);
            sampleEE = trajEE->computeNext();
            eeTask_->setReference(sampleEE);
        }

        const HQPData & HQPData = tsid_->computeProblemData(time_, q_, v_);
        Eigen::VectorXd dv = solver->solve(HQPData).x;
        v_ += dt*dv;
        q_ = pinocchio::integrate(model, q_, dt*v_);

        time_ += dt;
        if (i%500 == 0){
            cout << "time " << time_ << endl;
            cout << "joint " << q_.transpose() << endl;
        }
    }

    return 0;
}
