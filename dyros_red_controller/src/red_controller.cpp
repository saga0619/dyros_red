#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include "dyros_red_controller/wholebody_controller.h"
#include <fstream>

std::mutex mtx;
std::mutex mtx_rbdl;
std::mutex mtx_dc;
std::mutex mtx_terminal;
std::mutex mtx_ncurse;

RedController::RedController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm)
{
    initialize();
}

void RedController::stateThread()
{
    s_.connect();
    s_.stateThread();
}

void RedController::dynamicsThreadHigh()
{
    std::cout << "DynamicsThreadHigh : READY ?" << std::endl;
    ros::Rate r(2000);

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
    Eigen::VectorXd torque_grav;
    Eigen::MatrixXd ppinv;
    Eigen::MatrixXd tg_temp;
    Eigen::MatrixXd A_matrix_inverse;
    Eigen::MatrixXd J_C;
    Eigen::MatrixXd J_C_temp;
    Eigen::MatrixXd Jcon[2];
    Eigen::VectorXd qtemp[2], conp[2];

    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    N_C.setZero(total_dof_ + 6, total_dof_ + 6);
    bool first = true;

    acceleration_estimated_before.setZero();
    q_dot_before_.setZero();

    VectorQd TorqueDesiredLocal, TorqueContact;
    TorqueDesiredLocal.setZero();
    TorqueContact.setZero();

    Vector12d fc_redis;
    double fc_ratio;
    fc_redis.setZero();

    std::cout << "DynamicsThreadLow : START" << std::endl;
    while (!dc.shutdown && ros::ok())
    {
        std::chrono::high_resolution_clock::time_point dyn_loop_start = std::chrono::high_resolution_clock::now();

        getState();
        wc_.update();
        sec = std::chrono::high_resolution_clock::now() - start_time;
        if (sec.count() - control_time_ > 0.01)
        {
            // std::cout << "diff ::" << sec.count() - control_time_ << std::endl; //<<" dyn_low current time : " << control_time_ << "   chrono : " << sec.count() << std::endl;
        }
        ///////////////////////////////////////////////////////////////////////////////////////
        /////////////              Controller Code Here !                     /////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        wc_.contact_set_multi(1, 1, 0, 0);

        TorqueDesiredLocal = wc_.gravity_compensation_torque(dc.fixedgravity);
        TorqueContact = wc_.contact_force_redistribution_torque(red_.yaw_radian, TorqueDesiredLocal, fc_redis, fc_ratio);
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

        if (dc.shutdown)
            break;
        first = false;

        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - dyn_loop_start;
        //std::cout << "elapsed time : " << elapsed_time.count() << std::endl;
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