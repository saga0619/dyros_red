#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include "dyros_red_controller/redsvd.h"
#include <fstream>

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
    while (!dc.shutdown && ros::ok())
    {
        mtx.lock();
        s_.sendCommand(torque_desired);
        mtx.unlock();
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

    while (!dc.shutdown && ros::ok())
    {

        mtx.lock();
        getState();
        mtx.unlock();

        A_matrix_inverse = A_.inverse();
        link_id[0] = Right_Foot;
        link_id[1] = Left_Foot;
        for (int i = 0; i < contact_number; i++)
        {
            link_[link_id[i]].Set_Contact(q_virtual_, link_[link_id[i]].contact_point);
            if (!first)
            {
                if (qtemp[i] != q_virtual_)
                {
                    rprint(dc, 4, 30 * i, "qtmp %d : %f", i, ros::Time::now().toSec());
                }
            }
            if (!first)
            {
                if (conp[i] != link_[link_id[i]].contact_point)
                {
                    rprint(dc, 5, 30 * i, "conp %d : %f", i, ros::Time::now().toSec());
                }
            }
            qtemp[i] = q_virtual_;
            conp[i] = link_[link_id[i]].contact_point;

            J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = link_[link_id[i]].Jac_Contact;

            if (false)
            {
                if (Jcon[i] != link_[link_id[i]].Jac_Contact)
                {
                    rprint(dc, 6 + i, 0, "jcon %d : %f", i, ros::Time::now().toSec());
                    std::cout << "J_con " << i << std::endl
                              << Jcon[i] << std::endl
                              << "J con before " << std::endl
                              << link_[link_id[i]].Jac_Contact << std::endl
                              << "Diff" << std::endl
                              << Jcon[i] - link_[link_id[i]].Jac_Contact << std::endl;
                }
            }
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
        if (!first)
        {
            if (torque_desired != torque_grav)
            {
                rprint(dc, 9, 0, "torque_grav : %f", ros::Time::now().toSec());
            }
        }
        mtx.lock();
        torque_desired = torque_grav;
        mtx.unlock();
        if (dc.shutdown)
            break;

        first = false;
    }

    while (false)
    {

        if (display)
            start_time2 = std::chrono::high_resolution_clock::now();
        mtx.lock();
        getState();
        mtx.unlock();
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.02ms
            mvprintw(18, 0, "1 : %f ms", start * 1000);
        }
        A_matrix_inverse = A_.inverse();
        link_id[0] = Right_Foot;
        link_id[1] = Left_Foot;
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.048ms
            mvprintw(19, 0, "2 : %f ms", start * 1000);
        }
        J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
        for (int i = 0; i < contact_number; i++)
        {
            link_[link_id[i]].Set_Contact(q_virtual_, link_[link_id[i]].contact_point);
            J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = link_[link_id[i]].Jac_Contact;
        }
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.05 ms
            mvprintw(20, 0, "3 : %f ms", start * 1000);
        }

        Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();
        J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;
        N_C.setZero(total_dof_ + 6, total_dof_ + 6);
        I37.setIdentity(total_dof_ + 6, total_dof_ + 6);
        N_C = I37 - J_C.transpose() * J_C_INV_T;
        Slc_k.setZero(total_dof_, total_dof_ + 6);
        Slc_k.block(0, 6, total_dof_, total_dof_).setIdentity();
        Slc_k_T = Slc_k.transpose();
        //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
        W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix

        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.11ms
            mvprintw(21, 0, "4 : %f ms", start * 1000);
        }
        //W_inv = DyrosMath::pinv_SVD(W);
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.4ms
            mvprintw(21, 20, "svd : %f ms", start * 1000);
        }
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

        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.6ms
            mvprintw(22, 0, "5 : %f ms", start * 1000);
        }
        double epsilon = std::numeric_limits<double>::epsilon();

        double svd_time;
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.6ms
            svd_time = start.count();
            //mvprintw(22, 20, "5 : %f ms", start * 1000);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //RedSVD::RedSVD<Eigen::MatrixXd> svd(aa);
        if (display)
        {
            std::chrono::duration<double> start = std::chrono::high_resolution_clock::now() - start_time2; //0.6ms
            mvprintw(22, 20, "5 svd : %f ms", (start.count() - svd_time) * 1000);
        }
        double tolerance = epsilon * std::max(aa.cols(), aa.rows()) * svd.singularValues().array().abs()(0);

        ppinv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

        tg_temp = ppinv * J_g * A_matrix_inverse * N_C;
        torque_grav = tg_temp * G;

        mtx.lock();
        torque_desired = torque_grav;
        mtx.unlock();

        calc_count++;
        sec = std::chrono::high_resolution_clock::now() - start_time;

        if (display)
            display = false;
        if (sec.count() > sec1.count())
        {
            mvprintw(1, 0, "count : %d  ", calc_count);
            calc_count = 0;
            start_time = std::chrono::high_resolution_clock::now();
            display = false;
        }
    }
}

void RedController::tuiThread()
{
    while (!dc.shutdown && ros::ok())
    {
        for (int i = 0; i < 40; i++)
        {
            if (dc.Tq_[i].update && dc.ncurse_mode)
            {
                mtx.lock();
                mvprintw(dc.Tq_[i].y, dc.Tq_[i].x, dc.Tq_[i].text);
                dc.Tq_[i].update = false;
                if (dc.Tq_[i].clr_line)
                {
                    move(dc.Tq_[i].y, 0);
                    clrtoeol();
                }
                mtx.unlock();
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

    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        link_[i] = dc.link_[i];
    }
    yaw_radian = dc.yaw_radian;
    A_ = dc.A_;
    com_ = dc.com_;
}

void RedController::initialize()
{
    torque_desired.setZero();
}