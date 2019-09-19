#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include "dyros_red_controller/wholebody_controller.h"
#include "dyros_red_msgs/TaskCommand.h"
#include "stdlib.h"
#include <fstream>

std::mutex mtx;
std::mutex mtx_rbdl;
std::mutex mtx_dc;
std::mutex mtx_terminal;
std::mutex mtx_ncurse;

RedController::RedController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm), red_(dc_global.red_)
{
    initialize();

    task_command = dc.nh.subscribe("/dyros_red/taskcommand", 100, &RedController::TaskCommandCallback, this);
}

void RedController::TaskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr &msg)
{
    tc.command_time = control_time_;
    tc.traj_time = msg->time;
    tc.ratio = msg->ratio;
    tc.angle = msg->angle;
    tc.height = msg->height;
    tc.mode = msg->mode;

    red_.link_[Right_Foot].Set_initpos();
    red_.link_[Left_Foot].Set_initpos();
    red_.link_[Pelvis].Set_initpos();
    red_.link_[COM_id].Set_initpos();

    task_switch = true;
}

void RedController::stateThread()
{
    s_.connect();
    s_.stateThread();
}

void RedController::dynamicsThreadHigh()
{
    std::cout << "DynamicsThreadHigh : READY ?" << std::endl;
    ros::Rate r(4000);

    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalcdone) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }

    std::cout << "DynamicsThreadHigh : START" << std::endl;
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    int cnt = 0;

    std::chrono::duration<double> time_now = std::chrono::high_resolution_clock::now() - start_time;

    bool set_q_init = true;
    while (!dc.shutdown && ros::ok())
    {
        time_now = std::chrono::high_resolution_clock::now() - start_time;

        if (dc.mode == "simulation")
        {
        }
        else if (dc.mode == "realrobot")
        {
        }
        if (dc.positionControl)
        {
            if (set_q_init)
            {
                q_desired_ = q_;
                set_q_init = false;
            }
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torque_desired(i) = Kps[i] * (q_desired_(i) - q_(i)) - Kvs[i] * (q_dot_(i));
            }
        }

        if (dc.emergencyoff)
        {
            torque_desired.setZero();
        }
        cnt++;
        mtx.lock();
        s_.sendCommand(torque_desired, sim_time);
        mtx.unlock();

        r.sleep();
    }
}

void RedController::dynamicsThreadLow()
{
    std::cout << "DynamicsThreadLow : READY ?" << std::endl;
    ros::Rate r(2000);
    int calc_count = 0;
    int ThreadCount = 0;
    int i = 1;

    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalcdone) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }

    Wholebody_controller wc_(dc, red_);

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::chrono::seconds sec1(1);

    std::chrono::duration<double> sec = std::chrono::high_resolution_clock::now() - start_time;
    bool display = false;
    std::chrono::high_resolution_clock::time_point start_time2 = std::chrono::high_resolution_clock::now();

    int contact_number = 2;
    int link_id[contact_number];
    int total_dof_ = MODEL_DOF;
    Eigen::MatrixXd Lambda_c, J_C_INV_T, N_C, I37, Slc_k, Slc_k_T, W, W_inv;
    Eigen::MatrixXd P_c;
    Eigen::VectorXd Grav_ref;
    Eigen::VectorXd G;
    Eigen::MatrixXd J_g;
    Eigen::MatrixXd aa;
    Eigen::VectorXd torque_grav, torque_task;
    Eigen::MatrixXd ppinv;
    Eigen::MatrixXd tg_temp;
    Eigen::MatrixXd A_matrix_inverse;
    Eigen::MatrixXd J_C;
    Eigen::MatrixXd J_C_temp;
    Eigen::MatrixXd Jcon[2];
    Eigen::VectorXd qtemp[2], conp[2];

    Eigen::MatrixXd J_task;
    Eigen::VectorXd f_star;

    int task_number;

    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    N_C.setZero(total_dof_ + 6, MODEL_DOF_VIRTUAL);
    bool first = true;

    Eigen::Vector3d task_desired;

    acceleration_estimated_before.setZero();
    q_dot_before_.setZero();

    VectorQd TorqueDesiredLocal, TorqueContact;
    TorqueDesiredLocal.setZero();
    TorqueContact.setZero();

    Vector12d fc_redis;
    double fc_ratio;
    fc_redis.setZero();

    Vector3d kp_, kd_, kpa_, kda_;
    for (int i = 0; i < 3; i++)
    {
        kp_(i) = 400;
        kd_(i) = 40;
        kpa_(i) = 200;
        kda_(i) = 20;
    }

    //kd_(1) = 120;

    std::cout << "DynamicsThreadLow : START" << std::endl;
    int dynthread_cnt = 0;

    //Control Loop Start
    while (!dc.shutdown && ros::ok())
    {
        static double est;
        std::chrono::high_resolution_clock::time_point dyn_loop_start = std::chrono::high_resolution_clock::now();
        dynthread_cnt++;
        if (control_time_ == 0)
        {
            first = true;
            task_switch = false;
        }
        if ((dyn_loop_start - start_time2) > sec1)
        {
            start_time2 = std::chrono::high_resolution_clock::now();
            if (dc.checkfreqency)
                std::cout << "dynamics thread : " << dynthread_cnt << " hz, time : " << est << std::endl;
            dynthread_cnt = 0;
            est = 0;
        }
        getState(); //link data override

        //Task link gain setting.
        red_.link_[COM_id].pos_p_gain = kp_;
        red_.link_[COM_id].pos_d_gain = kd_;
        red_.link_[COM_id].rot_p_gain = red_.link_[Pelvis].rot_p_gain = kpa_;
        red_.link_[COM_id].rot_d_gain = red_.link_[Pelvis].rot_d_gain = kda_;
        red_.link_[Pelvis].pos_p_gain = kp_;
        red_.link_[Pelvis].pos_d_gain = kd_;

        red_.link_[Right_Foot].pos_p_gain = red_.link_[Left_Foot].pos_p_gain = kp_;
        red_.link_[Right_Foot].pos_d_gain = red_.link_[Left_Foot].pos_d_gain = kd_;
        red_.link_[Right_Foot].rot_p_gain = red_.link_[Left_Foot].rot_p_gain = kpa_;
        red_.link_[Right_Foot].rot_d_gain = red_.link_[Left_Foot].rot_d_gain = kda_;

        red_.link_[COM_id].pos_p_gain = kp_;
        red_.link_[COM_id].pos_d_gain  = kd_;

        wc_.update();

        sec = std::chrono::high_resolution_clock::now() - start_time;
        if (sec.count() - control_time_ > 0.01)
        {
            // std::cout << "diff ::" << sec.count() - control_time_ << std::endl; //<<" dyn_low current time : " << control_time_ << "   chrono : " << sec.count() << std::endl;
        }
        ///////////////////////////////////////////////////////////////////////////////////////
        /////////////              Controller Code Here !                     /////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        torque_task.setZero(MODEL_DOF);
        TorqueContact.setZero();

        if (dc.gravityMode)
        {
            std::cout << "Task Turned Off,, gravity compensation only !" << std::endl;
            task_switch = false;
            dc.gravityMode = false;
        }

        if (task_switch)
        {
            if (tc.mode == 0) //Pelvis position control
            {
                wc_.set_contact(1, 1);

                torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[Pelvis].Jac;

                red_.link_[Pelvis].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[Pelvis].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(Pelvis);
                torque_task = wc_.task_control_torque(J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 1) //COM position control
            {
                wc_.set_contact(1, 1);

                torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(COM_id);
                torque_task = wc_.task_control_torque(J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 2) //COM to Left foot, then switch double support to single support
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(1, 1);
                }
                else
                {
                    wc_.set_contact(1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = red_.link_[Left_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + red_.link_[Left_Foot].xpos(2);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                f_star = wc_.getfstar6d(COM_id);

                torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 3) //COM to Left foot, then switch double support to single support while holding com rotation.
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(1, 1);
                }
                else
                {
                    wc_.set_contact(1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = red_.link_[Left_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + red_.link_[Left_Foot].xpos(2);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                f_star = wc_.getfstar6d(COM_id);

                torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 4) //left foot controller
            {
                wc_.set_contact(1, 0);
                torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;
                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Right_Foot].Jac;

                red_.link_[COM_id].x_desired = red_.link_[COM_id].x_init;
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[Right_Foot].x_desired(0) = tc.ratio;
                red_.link_[Right_Foot].x_desired(1) = red_.link_[Right_Foot].x_init(1);
                red_.link_[Right_Foot].x_desired(2) = tc.height;

                red_.link_[Right_Foot].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[Right_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 6) = wc_.getfstar6d(COM_id);
                f_star.segment(6, 6) = wc_.getfstar6d(Right_Foot);

                torque_task = wc_.task_control_torque(J_task, f_star);
                //red_.link_[Right_Foot].x_desired = tc.
            }
            else if (tc.mode == 3)
            {
                static int loop_temp;
                static int loop_;
                static bool cgen_init = true;
                static bool loop_cnged = false;
                static bool walking_init = true;

                //QP_switch = true;
                red_.ee_[0].contact = true;
                red_.ee_[1].contact = true;

                //right_foot_contact_ = true;
                //left_foot_contact_ = true;

                double time_segment = 1.0;
                double step_length = 0.0;

                double task_time = control_time_ - tc.command_time;
                Vector2d cp_current = red_.com_.CP;
                double w_ = sqrt(9.81 / red_.com_.pos(2));
                loop_temp = loop_;
                loop_ = (int)(task_time / time_segment);

                double loop_time = task_time - (double)loop_ * time_segment;

                double b_ = exp(w_ * (time_segment - loop_time));

                Vector2d desired_cp;
                Vector2d right_cp, left_cp, cp_mod;
                //cp_mod << -0.02, -0.00747;
                //right_cp << -0.04, -0.095;
                //left_cp << -0.04, 0.095;
                cp_mod << -0.00, -0.00747;
                right_cp << 0.03, -0.1;
                left_cp << 0.03, 0.1;

                if (loop_ - loop_temp)
                {
                    loop_cnged = true;
                    cgen_init = true;
                }
                double st_temp;

                if (loop_ % 2)
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;

                    desired_cp = right_cp;
                    desired_cp(0) = right_cp(0) + ((double)loop_) * st_temp;
                }
                else
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;
                    desired_cp = left_cp;
                    desired_cp(0) = left_cp(0) + ((double)loop_) * st_temp;
                }

                Vector2d zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * red_.com_.CP;

                if (cgen_init)
                {
                    // std::cout << "loop : " << loop_ << " loop time : " << loop_time << std::endl;
                    //cx_init = model_.com_.pos.segment(0, 2);
                    //cv_init = model_.com_.vel.segment(0, 2);
                    //std::cout << "desired cp   x : " << desired_cp(0) << "  y : " << desired_cp(1) << std::endl;
                    //std::cout << "zmp gen   x : " << zmp(0) << "  y : " << zmp(1) << std::endl;
                    //std::cout << "c CP" << std::endl;
                    //std::cout << model_.com_.CP << std::endl;

                    //zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;
                    //cgen_init = false;
                }

                if (zmp(1) > 0.11)
                {
                    zmp(1) = 0.11;
                }
                if (zmp(1) < -0.11)
                {
                    zmp(1) = -0.11;
                }
                if (loop_ > 0)
                {
                    if (red_.ee_[0].contact)
                    {
                        if (zmp(1) > red_.link_[Left_Foot].xpos(1) + 0.05)
                        {
                            zmp(1) = red_.link_[Left_Foot].xpos(1) + 0.05;
                        }
                        if (zmp(1) < red_.link_[Left_Foot].xpos(1) - 0.08)
                        {
                            zmp(1) = red_.link_[Left_Foot].xpos(1) - 0.08;
                        }
                    }
                    else if (red_.ee_[1].contact)
                    {
                        if (zmp(1) > red_.link_[Right_Foot].xpos(1) + 0.08)
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos(1) + 0.08;
                        }
                        if (zmp(1) < red_.link_[Right_Foot].xpos(1) - 0.05)
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos(1) - 0.05;
                        }
                    }
                }

                // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
                // std::cout << "estimated_cp" << std::endl;
                // std::cout << est_cp_ << std::endl;
                wc_.set_zmp_control(zmp, 1.0);
                //MatrixXd damping_matrix;
                //damping_matrix.setIdentity(total_dof_, total_dof_);

                // //torque_dc_.segment(0, 12) = q_dot_.segment(0, 12) * 2.0;
                // std::cout << "zmp_des" << std::endl;
                // std::cout << zmp << std::endl;
                //std::cout << "zmp_cur" << std::endl;
                //std::cout << model_.com_.ZMP << std::endl;
                //std::cout << "Sensor ZMP" << std::endl;
                //std::cout << body_zmp_ << std::endl;

                //single support test at tc_
                // single support , 0.1 ~ 0.9s foot up -> down just for 4cm?
                // at loop_ = 0( first phase)
                // at loop_ = 1, zmp at right foot.
                // at each loop, 0~0.1 double support 0.1~ 0.9 singlesupport 0.9~1.0 double support

                //
                double lr_st, lr_mt, lr_et;
                lr_st = time_segment / 10.0;
                lr_mt = time_segment / 10.0 * 5.0;
                lr_et = time_segment / 10.0 * 9.0;

                if ((double)loop_ > 0.1)
                {
                    if (loop_ % 2)
                    {
                        if ((loop_time < lr_et))
                        {

                            red_.ee_[1].contact = false;
                            red_.ee_[0].contact = true;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                    else
                    {
                        if ((loop_time < lr_et))
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = false;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                }
                //(model_.link_[model_.COM_id].x_init - zmp)*cosh(loop_time/time_segment)+time_segment * model_.link_[model_.COM_id]
                task_desired.setZero();
                task_desired(2) = tc.height;

                red_.link_[COM_id]
                    .Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, task_desired);

                // model_.link_[model_.COM_id].x_traj.segment(0, 2) = (cx_init - zmp) * cosh(loop_time * w_) + cv_init * sinh(loop_time * w_) / w_ + zmp;

                // model_.link_[model_.COM_id].v_traj.segment(0, 2) = (cx_init - zmp) * w_ * sinh(loop_time * w_) + cv_init * cosh(loop_time * w_);

                // std::cout << " xtraj : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].x_traj.segment(0, 2) << std::endl;
                //std::cout << "vtrah : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].v_traj.segment(0, 2) << std::endl;

                if (red_.ee_[1].contact && red_.ee_[0].contact)
                {
                    walking_init = true;
                    task_number = 6;
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);
                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;

                    wc_.set_contact(1, 1);
                    torque_grav = wc_.gravity_compensation_torque();

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 3) = wc_.getfstar_tra(COM_id, kp_, kd_);
                    f_star.segment(3, 3) = wc_.getfstar_rot(Pelvis, kpa_, kda_);
                }
                else if (red_.ee_[0].contact)
                {
                    if (walking_init)
                    {
                        red_.link_[Right_Foot].x_init = red_.link_[Right_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;

                    wc_.set_contact(1, 0);
                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = red_.link_[Right_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque();

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    Vector3d lf_desired;
                    lf_desired = red_.link_[Right_Foot].x_init;
                    lf_desired(1) = -0.1024;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.05, red_.link_[Right_Foot].x_init(0), 0, 0, red_.link_[Left_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Right_Foot].x_traj(0) = quintic(0);
                    red_.link_[Right_Foot].v_traj(0) = quintic(1);

                    red_.link_[Right_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 3) = wc_.getfstar_tra(COM_id, kp_, kd_);
                    f_star.segment(3, 3) = wc_.getfstar_rot(Pelvis, kpa_, kda_);
                    f_star.segment(6, 3) = wc_.getfstar_tra(Right_Foot, kp_, kd_);

                    f_star.segment(9, 3) = wc_.getfstar_rot(Right_Foot, kpa_, kda_);
                    f_star(9) = 0;
                }
                else if (red_.ee_[1].contact) // rightfoot contact
                {

                    Vector3d lf_desired;
                    if (walking_init)
                    {
                        red_.link_[Left_Foot].x_init = red_.link_[Left_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;
                    wc_.set_contact(0, 1);

                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = red_.link_[Left_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque();

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);
                    //model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

                    lf_desired = red_.link_[Left_Foot].x_init;

                    lf_desired(1) = 0.1024;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.05, red_.link_[Left_Foot].x_init(0), 0, 0, red_.link_[Right_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Left_Foot].x_traj(0) = quintic(0);
                    red_.link_[Left_Foot].v_traj(0) = quintic(1);

                    red_.link_[Left_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 3) = wc_.getfstar_tra(COM_id, kp_, kd_);
                    f_star.segment(3, 3) = wc_.getfstar_rot(Pelvis, kpa_, kda_);
                    f_star.segment(6, 3) = wc_.getfstar_tra(Left_Foot, kp_, kd_);

                    f_star.segment(9, 3) = wc_.getfstar_rot(Left_Foot, kpa_, kda_);
                    f_star(9) = 0;
                }

                torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 4)
            {
            }
        }
        else
        {
            wc_.set_contact(1, 1);
            torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
        }

        TorqueDesiredLocal = torque_grav + torque_task;

        static int cr_mode;

        if (dc.torqueredis)
        {
            dc.torqueredis = false;
            cr_mode++;
            if (cr_mode > 2)
                cr_mode = 0;

            if (cr_mode == 0)
                std::cout << "contact torque redistribution by yslee " << std::endl;
            else if (cr_mode == 1)
                std::cout << "contact torque redistribution by qp " << std::endl;
            else if (cr_mode == 2)
                std::cout << "contact torque redistribution disabled " << std::endl;
        }

        if (cr_mode == 0)
        {
            TorqueContact = wc_.contact_force_redistribution_torque(red_.yaw_radian, TorqueDesiredLocal, fc_redis, fc_ratio);
        }
        else if (cr_mode == 1)
        {
            TorqueContact = wc_.contact_torque_calc_from_QP(TorqueDesiredLocal);
        }

        //acceleration_estimated = (A_matrix_inverse * N_C * Slc_k_T * (TorqueDesiredLocal - torque_grav)).segment(6, MODEL_DOF);
        //acceleration_observed = q_dot_ - q_dot_before_;
        //q_dot_before_ = q_dot_;
        //std::cout << "acceleration_observed : " << std::endl;
        //std::cout << acceleration_observed << std::endl;
        //acceleration_differance = acceleration_observed - acceleration_estimated_before;
        //acceleration_estimated_before = acceleration_estimated;

        ///////////////////////////////////////////////////////////////////////////////////////
        //////////////////              Controller Code End             ///////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        mtx.lock();
        torque_desired = TorqueDesiredLocal + TorqueContact;
        //dc.accel_dif = acceleration_differance;
        //dc.accel_obsrvd = acceleration_observed;
        mtx.unlock();

        contact_force = wc_.get_contact_force(torque_desired);

        //VectorXd tau_coriolis;
        //RigidBodyDynamics::NonlinearEffects(model_,red_.q_virtual_,red_.q_dot_virtual_,tau_coriolis)
        wc_.GetZMPpos();
        if (dc.shutdown)
            break;
        first = false;

        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - dyn_loop_start;

        est += elapsed_time.count();

        //std::this_thread::sleep_until(dyn_loop_start + dc.dym_timestep);
    }
}

void RedController::tuiThread()
{
    while (!dc.shutdown && ros::ok())
    {
        mtx_terminal.lock();
        if (dc.ncurse_mode)
            mvprintw(4, 30, "I'm alive : %f", ros::Time::now().toSec());
        mtx_terminal.unlock();
        for (int i = 0; i < 40; i++)
        {
            if (dc.Tq_[i].update && dc.ncurse_mode)
            {
                mtx_terminal.lock();
                mvprintw(dc.Tq_[i].y, dc.Tq_[i].x, dc.Tq_[i].text);
                dc.Tq_[i].update = false;
                if (dc.Tq_[i].clr_line)
                {
                    move(dc.Tq_[i].y, 0);
                    clrtoeol();
                }
                mtx_terminal.unlock();
            }
            else if (dc.Tq_[i].update && (!dc.ncurse_mode))
            {
                std::cout << dc.Tq_[i].text << std::endl;
                dc.Tq_[i].update = false;
            }
        }
        if (dc.ncurse_mode)
        {
            if (getch() == 'q')
            {
                dc.shutdown = true;
            }
        }
        else if (!ros::ok())
        {
            dc.shutdown = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RedController::getState()
{
    int count = 0;
    if (dc.testmode == false)
    {
        while ((time == dc.time) && ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            count++;
        }
    }
    mtx_dc.lock();

    time = dc.time;
    control_time_ = dc.time;
    sim_time = dc.sim_time;
    dym_hz = dc.dym_hz;
    stm_hz = dc.stm_hz;
    q_ = dc.q_;
    q_virtual_ = dc.q_virtual_;
    q_dot_ = dc.q_dot_;
    q_dot_virtual_ = dc.q_dot_virtual_;
    q_ddot_virtual_ = dc.q_ddot_virtual_;
    torque_ = dc.torque_;

    red_.q_ = dc.q_;
    red_.q_virtual_ = dc.q_virtual_;
    red_.q_dot_ = dc.q_dot_;
    red_.q_dot_virtual_ = dc.q_dot_virtual_;
    red_.q_ddot_virtual_ = dc.q_ddot_virtual_;

    static bool first_run = true;
    if (first_run)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {

            red_.link_[i] = dc.link_[i];
        }
        first_run = false;
    }
    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        red_.link_[i].xpos = dc.link_[i].xpos;
        red_.link_[i].xipos = dc.link_[i].xipos;
        red_.link_[i].Rotm = dc.link_[i].Rotm;
        red_.link_[i].Jac = dc.link_[i].Jac;
        red_.link_[i].Jac_COM = dc.link_[i].Jac_COM;
        red_.link_[i].Jac_COM_p = dc.link_[i].Jac_COM_p;
        red_.link_[i].Jac_COM_r = dc.link_[i].Jac_COM_r;
        red_.link_[i].COM_position = dc.link_[i].COM_position;
        red_.link_[i].xpos_contact = dc.link_[i].xpos_contact;
        red_.link_[i].v = dc.link_[i].v;
        red_.link_[i].w = dc.link_[i].w;
    }
    red_.yaw_radian = dc.yaw_radian;
    red_.A_ = dc.A_;
    red_.com_ = dc.com_;
    mtx_dc.unlock();
}

void RedController::initialize()
{
    torque_desired.setZero();
}

void RedController::ContinuityChecker(double data)
{
}

void RedController::ZMPmonitor()
{
}