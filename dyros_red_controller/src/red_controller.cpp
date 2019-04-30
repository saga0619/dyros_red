#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include "dyros_red_controller/redsvd.h"
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
    ros::Rate r(2000);

    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalc) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }


    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    int cnt = 0;

    while (!dc.shutdown && ros::ok())
    {
        mtx.lock();
        torque_desired.setZero();
        if(cnt == 4000)
        {
            printf("t(5) = 10 activate !\n");
        }
        if(cnt == 6000)
        {
            printf("torque zero\n");
        }
        if(cnt>4000 && cnt<5000)
        {
            torque_desired(0)=40;
        }
        s_.sendCommand(torque_desired,0.0);
        mtx.unlock();

        cnt++;
        if(cnt%200==0)
        {
            printf("%f \n",dc.q_(0));
        }
        
        r.sleep();
    }
}

void RedController::dynamicsThreadLow()
{
    ros::Rate r(2000);
    int calc_count = 0;
    int ThreadCount = 0;
    int i = 1;

    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalc) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::chrono::seconds sec1(1);

    std::chrono::duration<double> sec = std::chrono::high_resolution_clock::now() - start_time;
    bool display = false;
    std::chrono::high_resolution_clock::time_point start_time2 = std::chrono::high_resolution_clock::now();

    int contact_number = 2;
    int link_id[contact_number];
    int total_dof_ = MODEL_DOF;
    Eigen::MatrixXd Lambda_c, J_C_INV_T, N_C, I37, Slc_k, Slc_k_T, W, W_inv;
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

/*
    while (!dc.shutdown && ros::ok())
    {
        getState();

        A_matrix_inverse = A_.inverse();
        link_id[0] = Right_Foot;
        link_id[1] = Left_Foot;
        for (int i = 0; i < contact_number; i++)
        {
            link_[link_id[i]].Set_Contact(q_virtual_, link_[link_id[i]].contact_point);

            qtemp[i] = q_virtual_;
            conp[i] = link_[link_id[i]].contact_point;

            J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = link_[link_id[i]].Jac_Contact;

            Jcon[i] = link_[link_id[i]].Jac_Contact;
        }

        J_C_temp = J_C;

        Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();

        J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;
        I37.setIdentity(total_dof_ + 6, total_dof_ + 6);

        N_C = I37 - J_C.transpose() * J_C_INV_T;
        Slc_k.setZero(total_dof_, total_dof_ + 6);
        Slc_k.block(0, 6, total_dof_, total_dof_).setIdentity();
        Slc_k_T = Slc_k.transpose();
        //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
        W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix

        //W_inv = DyrosMath::pinv_SVD(W);
        G.setZero(total_dof_ + 6);

        Grav_ref.setZero(3);
        Grav_ref(2) = -9.81;

        for (int i = 0; i < total_dof_ + 1; i++)
        {
            G -= link_[i].Jac_COM_p.transpose() * link_[i].Mass * Grav_ref;
        }
        J_g.setZero(total_dof_, total_dof_ + 6);
        J_g.block(0, 6, total_dof_, total_dof_).setIdentity();

        aa = J_g * A_matrix_inverse * N_C * J_g.transpose();

        double epsilon = std::numeric_limits<double>::epsilon();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //RedSVD::RedSVD<Eigen::MatrixXd> svd(aa);

        double tolerance = epsilon * std::max(aa.cols(), aa.rows()) * svd.singularValues().array().abs()(0);

        ppinv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

        tg_temp = ppinv * J_g * A_matrix_inverse * N_C;
        torque_grav = tg_temp * G;

        mtx.lock();
        torque_desired = torque_grav;
        s_.sendCommand(torque_desired, sim_time);
        mtx.unlock();
        if (dc.shutdown)
            break;

        first = false;
    }*/
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
    mtx_dc.lock();
    time = dc.time;
    sim_time = dc.sim_time;

    dym_hz = dc.dym_hz;
    stm_hz = dc.stm_hz;

    q_ = dc.q_;
    q_virtual_ = dc.q_virtual_;
    q_dot_ = dc.q_dot_;
    q_dot_virtual_ = dc.q_dot_virtual_;
    q_ddot_virtual_ = dc.q_ddot_virtual_;
    torque_ = dc.torque_;

    static bool first_run = true;
    if (first_run)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            link_[i] = dc.link_[i];
        }
    }

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        link_[i].xpos = dc.link_[i].xpos;
        link_[i].xipos = dc.link_[i].xipos;
        link_[i].Rotm = dc.link_[i].Rotm;
        link_[i].Jac = dc.link_[i].Jac;
        link_[i].Jac_COM = dc.link_[i].Jac_COM;
        link_[i].Jac_COM_p = dc.link_[i].Jac_COM_p;
        link_[i].Jac_COM_r = dc.link_[i].Jac_COM_r;
        link_[i].COM_position = dc.link_[i].COM_position;
        link_[i].xpos_contact = dc.link_[i].xpos_contact;
    }
    yaw_radian = dc.yaw_radian;
    A_ = dc.A_;
    com_ = dc.com_;
    mtx_dc.unlock();
}

void RedController::initialize()
{
    torque_desired.setZero();
}