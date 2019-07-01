#include "dyros_red_controller/wholebody_controller.h"

// #include "cvxgen/solver.h"

// Vars vars;
// Params params;
// Workspace work;(DataContainer &dc, KinematicsData &kd_);
// Settings settings;
Wholebody_controller::Wholebody_controller(DataContainer &dc_global, KinematicsData &kd_) : dc(dc_global), rk_(kd_)
{
    Grav_ref.setZero(3);
    Grav_ref(2) = -9.81;
}

void Wholebody_controller::update()
{
    current_q_ = rk_.q_virtual_;
    A_matrix = rk_.A_;
    A_matrix_inverse = rk_.A_.inverse();
    task_force_control = false;
    task_force_control_feedback = false;
    zmp_control = false;
}
/*
void Wholebody_controller::contact_set(int contact_number, int link_id[])
{
    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    for (int i = 0; i < contact_number; i++)
    {
        rk_.link_[link_id[i]].Set_Contact(current_q_, rk_.link_[link_id[i]].contact_point);
        J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = rk_.link_[link_id[i]].Jac_Contact;
    }
    Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();
    J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;
    N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    N_C = I37 - J_C.transpose() * J_C_INV_T;
    Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Slc_k_T = Slc_k.transpose();
    //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
    W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix
    W_inv = DyrosMath::pinv_SVD(W);
    contact_force_predict.setZero();
}*/

void Wholebody_controller::set_contact(bool right_foot, bool left_foot, bool right_hand, bool left_hand)
{
    contact_index = 0;
    if (right_foot)
    {
        contact_part[contact_index] = Right_Foot;
        contact_index++;
    }
    if (left_foot)
    {
        contact_part[contact_index] = Left_Foot;
        contact_index++;
    }
    /* if (right_hand)
    {
        contact_part[contact_index] = Right_Hand;
        contact_index++;
    }
    if (left_hand)
    {
        contact_part[contact_index] = Left_Hand;
        contact_index++;
    }*/
    //contact_set(contact_index, contact_part);

    J_C.setZero(contact_index * 6, MODEL_DOF_VIRTUAL);
    for (int i = 0; i < contact_index; i++)
    {
        rk_.link_[contact_part[i]].Set_Contact(current_q_, rk_.link_[contact_part[i]].contact_point);
        J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = rk_.link_[contact_part[i]].Jac_Contact;
    }
    Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();
    J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;
    N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    N_C = I37 - J_C.transpose() * J_C_INV_T;
    Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Slc_k_T = Slc_k.transpose();
    //W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
    W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix
    W_inv = DyrosMath::pinv_SVD(W);
    contact_force_predict.setZero();
}

Matrix2d matpower(Matrix2d mat, int i)
{
    Matrix2d m;
    m.setIdentity();
    if (i == 0)
    {
        return m;
    }
    else
    {
        for (int j = 0; j < i; j++)
        {
            m = m * mat;
        }
    }
    return m;
}

Vector2d Wholebody_controller::getcpref(double task_time, double future_time)
{
    double time_segment = 1.0;
    double step_length = 0.1;
    double w_ = sqrt(9.81 / 0.081);
    double b_ = exp(w_ * (time_segment));

    Vector2d CP_ref[9];
    CP_ref[0] << -0.04, 0;
    CP_ref[1] << -0.04, 0.096256;
    CP_ref[2] << 0.06, -0.096256;
    CP_ref[3] << 0.16, 0.096256;
    CP_ref[4] << 0.26, -0.096256;
    CP_ref[5] << 0.36, 0.096256;
    CP_ref[6] << 0.46, -0.096256;
    CP_ref[7] << 0.46, 0.0;
    CP_ref[8] << 0.46, 0.0;

    Vector2d ZMP_ref[8];
    for (int i = 0; i < 8; i++)
    {
        ZMP_ref[i] = 1 / (1 - b_) * CP_ref[i + 1] - b_ / (1 - b_) * CP_ref[i];
    }

    //(int)time
    double left_time = 1 - task_time + (int)task_time;

    double t_see;

    t_see = task_time + future_time;
    Vector2d CP_t;
    CP_t = exp(w_ * (t_see - (int)t_see)) * CP_ref[(int)t_see] + (1.0 - exp(w_ * (t_see - (int)t_see))) * ZMP_ref[(int)t_see];

    if ((task_time - (int)task_time) + future_time < 1)
    {
        double b = exp(w_ * left_time);
        Vector2d zmp_ = 1 / (1 - b) * CP_ref[(int)task_time + 1] - b / (1 - b) * rk_.com_.CP;

        CP_t = exp(w_ * future_time) * rk_.com_.CP + (1.0 - exp(w_ * future_time)) * zmp_;
    }

    return CP_t;
}

// Vector2d Wholebody_controller::getcptraj(double time, Vector2d zmp) //task_time
// {
//   double time_segment = 1.0;
//   double step_length = 0.1;

//   int n_sample = 30;
//   double t_sample = 0.005; //milliseconds

//   double task_time = control_time_ - tc_.taskcommand_.command_time;

//   double w_ = sqrt(9.81 / 0.81);

//   Matrix2d A, B;
//   A << exp(w_ * t_sample), 0, 0, exp(w_ * t_sample);
//   B << 1 - exp(w_ * t_sample), 0, 0, 1 - exp(w_ * t_sample);

//   MatrixXd F_xi, F_p, F_p_temp;
//   F_xi.setZero(n_sample * 2, 2);
//   F_p.setZero(n_sample * 2, n_sample * 2);
//   F_p_temp.setZero(n_sample * 2, 2);

//   for (int i = 0; i < n_sample; i++)
//   {
//     F_xi.block(i * 2, 0, 2, 2) = matpower(A, i + 1);
//     for (int j = i; j < n_sample; j++)
//     {
//       F_p.block(j * 2, i * 2, 2, 2) = matpower(A, j - i) * B;
//     }
//   }

//   MatrixXd THETA;
//   THETA.setIdentity(n_sample * 2, n_sample * 2);

//   Matrix2d I2;
//   I2.setIdentity();

//   for (int i = 0; i < n_sample - 1; i++)
//   {
//     THETA.block(i * 2 + 2, i * 2, 2, 2) = -I2;
//   }

//   MatrixXd e1;
//   e1.setZero(2 * n_sample, 2);
//   e1.block(0, 0, 2, 2) = I2;

//   double q_par = 1.0;
//   double r_par = 0.4;

//   MatrixXd H, I_nsample;
//   I_nsample.setIdentity(2 * n_sample, 2 * n_sample);
//   MatrixXd Q, R;
//   Q = q_par * I_nsample;
//   R = r_par * I_nsample;

//   H = THETA.transpose() * R * THETA + F_p.transpose() * Q * F_p;

//   VectorXd g;
//   VectorXd cp_ref_t;
//   cp_ref_t.setZero(n_sample * 2);
//   for (int i = 0; i < n_sample; i++)
//   {
//     cp_ref_t.segment(i * 2, 2) = getcpref(time, i * t_sample);
//   }
//   std::cout << " pk-1" << std::endl;
//   std::cout << p_k_1 << std::endl;
//   std::cout << " cp_ref_t : " << std::endl;
//   std::cout << cp_ref_t << std::endl;
//   std::cout << " com_CP : " << com_.CP << std::endl;
//   std::cout << " g1 : " << std::endl
//             << F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) << std::endl;
//   std::cout << " g2 : " << std::endl
//             << THETA.transpose() * R * e1 * p_k_1 << std::endl;

//   g = F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) - THETA.transpose() * R * e1 * p_k_1;

//   VectorXd r(n_sample * 2);
//   Vector2d sf[8];
//   sf[0] << -0.04, 0;
//   sf[1] << -0.04, 0.1024;
//   sf[2] << 0.06, -0.1024;
//   sf[3] << 0.16, 0.1024;
//   sf[4] << 0.26, -0.1024;
//   sf[5] << 0.36, 0.1024;
//   sf[6] << 0.46, -0.1024;
//   sf[7] << 0.46, 0.0;

//   for (int i = 0; i < n_sample; i++)
//   {
//     r.segment(i * 2, 2) = sf[(int)(time + t_sample * i)];
//   }

//   VectorXd lb, ub, f;
//   f.setZero(n_sample * 2);

//   for (int i = 0; i < n_sample * 2; i++)
//   {
//     f(i) = 0.1;
//   }
//   lb = lb.setZero(n_sample * 2);
//   lb = r - f;
//   ub = ub.setZero(n_sample * 2);
//   ub = r + f;

//   Vector2d res_2;
//   res_2.setZero();
//   VectorXd res;
//   res.setZero(n_sample * 2);
//   if (mpc_init)
//   {
//     QP_mpc.InitializeProblemSize(2 * n_sample, 1);
//     QP_mpc.UpdateMinProblem(H, g);
//     QP_mpc.PrintMinProb();
//     QP_mpc.PrintSubjectTox();
//     QP_mpc.SolveQPoases(1000);
//     set_defaults();
//     setup_indexing();
//     settings.verbose = 0;

//     for (int i = 0; i < 2 * n_sample; i++)
//       for (int j = 0; j < 2 * n_sample; j++)
//         params.H[i + j * 2 * n_sample] = H(i, j);

//     for (int i = 0; i < 2 * n_sample; i++)
//     {
//       params.g[i] = g(i);
//       params.r[i] = r(i);
//     }

//     params.f[0] = 0.025;
//     int num_inters = solve();

//     std::cout << "ANSWER IS " << vars.P[0] << vars.P[1] << std::endl;

//     res_2(0) = vars.P[0];
//     res_2(1) = vars.P[1];

//     p_k_1 = res_2;
//   }

//   if (mpc_init == false)
//   {
//     p_k_1 = zmp;
//   }
//   mpc_init = true;

//   return res_2;
// }

VectorQd Wholebody_controller::contact_torque_calc_from_QP(VectorQd command_torque)
{
    VectorXd ContactForce__ = get_contact_force(command_torque);

    double a1 = 0.0;
    double a2 = 1.0;
    double friction_ratio = 0.3;
    //qptest

    int constraint_per_contact = 8;
    QP_test.InitializeProblemSize(6 * contact_index, 6 + constraint_per_contact * contact_index);

    MatrixXd H, A, M;
    H.setZero(6 * contact_index, 6 * contact_index);
    M.setZero(6 * contact_index, 6 * contact_index);
    for (int i = 0; i < contact_index; i++)
    {
        M(6 * i, 6 * i) = 1;
        M(6 * i + 1, 6 * i + 1) = 1;
        M(6 * i + 2, 6 * i + 2) = 0.01;
        M(6 * i + 3, 6 * i + 3) = 100;
        M(6 * i + 4, 6 * i + 4) = 100;
        M(6 * i + 5, 6 * i + 5) = 100;
    }
    H = a1 * MatrixXd::Identity(contact_index * 6, contact_index * 6) + a2 * M;

    A.setZero(6 + constraint_per_contact * contact_index, 6 * contact_index);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(rk_.link_[contact_part[i]].xpos_contact);
    }

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i * constraint_per_contact + 0, 2 + 6 * i) = -0.12;
        A(6 + i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        A(6 + i * constraint_per_contact + 1, 2 + 6 * i) = -0.12;
        A(6 + i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        A(6 + i * constraint_per_contact + 2, 2 + 6 * i) = -0.04;
        A(6 + i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        A(6 + i * constraint_per_contact + 3, 2 + 6 * i) = -0.04;
        A(6 + i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        A(6 + i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        A(6 + i * constraint_per_contact + 4, 2 + 6 * i) = -friction_ratio;
        A(6 + i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        A(6 + i * constraint_per_contact + 5, 2 + 6 * i) = -friction_ratio;

        A(6 + i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        A(6 + i * constraint_per_contact + 6, 2 + 6 * i) = -friction_ratio;
        A(6 + i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        A(6 + i * constraint_per_contact + 7, 2 + 6 * i) = -friction_ratio;
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);
    g = -a1 * ContactForce__;

    lbA.setZero(6 + constraint_per_contact * contact_index);
    ubA.setZero(6 + constraint_per_contact * contact_index);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(6 * contact_index);
    lb.setZero(6 * contact_index);
    for (int i = 0; i < 6 * contact_index; i++)
    {
        lb(i) = -1000;
        ub(i) = 1000;
    }
    for (int i = 0; i < contact_index; i++)
    {
        ub(6 * i + 2) = 0.0;

        ub(6 * i + 5) = 0.0;
        lb(6 * i + 5) = 0.0;
    }

    for (int i = 0; i < constraint_per_contact * contact_index; i++)
    {
        lbA(6 + i) = 0.0;
        ubA(6 + i) = 1000.0;
    }

    QP_test.EnableEqualityCondition(0.0001);
    QP_test.UpdateMinProblem(H, g);
    QP_test.UpdateSubjectToAx(A, lbA, ubA);
    QP_test.UpdateSubjectToX(lb, ub);

    ROS_INFO("l8");
    VectorXd force_redistribute = QP_test.SolveQPoases(100);

    ROS_INFO("l9");
    result_temp = force_redistribute;

    ROS_INFO("l10");
    VectorXd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);

    // std::cout << "redistribute" << std::endl;
    // std::cout << force_redistribute << std::endl;
    // std::cout << "resultant force" << std::endl;
    // std::cout << force_res << std::endl;
    // std::cout << "A matrix " << std::endl;
    // std::cout << A << std::endl;
    // std::cout << "lbA" << std::endl;
    // std::cout << lbA << std::endl;
    // std::cout << "ubA" << std::endl;
    // std::cout << ubA << std::endl;
    // std::cout << "ub" << std::endl;
    // std::cout << ub << std::endl;
    // std::cout << "lb" << std::endl;
    // std::cout << lb << std::endl;

    ROS_INFO("l2");
    return torque_contact_;
}
/*
VectorQd Wholebody_controller::contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio)
{
    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        H(6 * i, 6 * i) = 1;
        H(6 * i + 1, 6 * i + 1) = 1;
        H(6 * i + 2, 6 * i + 2) = 0.01;
        H(6 * i + 3, 6 * i + 3) = 100;
        H(6 * i + 4, 6 * i + 4) = 100;
        H(6 * i + 5, 6 * i + 5) = 100;
    }
    A.setZero(6 + contact_index, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);
    lbA.setZero(6 + contact_index);
    ubA.setZero(6 + contact_index);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i, 1 + 6 * i) = 1.0;

        ubA(6 + i) = 0.0;
        lbA(6 + i) = 0.0;
        if (contact_part[i] == Right_Foot)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        /*if (contact_part[i] == Left_Foot)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        } 
}
for (int i = 0; i < contact_index * 6; i++)
{
    lb(i) = -1000;
    ub(i) = 1000;
}

for (int i = 0; i < contact_index; i++)
    ub(2 + 6 * i) = 0;

QP_test.EnableEqualityCondition(0.001);
QP_test.UpdateMinProblem(H, g);
QP_test.UpdateSubjectToAx(A, lbA, ubA);
QP_test.UpdateSubjectToX(lb, ub);
VectorXd force_redistribute = QP_test.SolveQPoases(100);

std::cout << "Contact Force now :  " << std::endl;
std::cout << ContactForce__ << std::endl;
std::cout << "Contact Force Redistribution : " << std::endl;
std::cout << force_redistribute << std::endl;

VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
return torque_contact_;
}

VectorQd Wholebody_controller::contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio)
{
    double a1, a2;

    a1 = 10.0; //Contactforce control ratio
    a2 = 1.0;  //moment minimize ratio

    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index * 5);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);

    MatrixXd M;
    M.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        M(6 * i, 6 * i) = 1.0;
        M(6 * i + 1, 6 * i + 1) = 1.0;
        M(6 * i + 2, 6 * i + 2) = 0.1;
        M(6 * i + 3, 6 * i + 3) = 100.0;
        M(6 * i + 4, 6 * i + 4) = 100.0;
        M(6 * i + 5, 6 * i + 5) = 100.0;
    }

    H = a1 * MatrixXd::Identity(contact_index * 6, contact_index * 6) + a2 * M;

    A.setZero(6 + contact_index * 5, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);

    g = -a1 * ContactForce__;
    lbA.setZero(6 + contact_index * 5);
    ubA.setZero(6 + contact_index * 5);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i * 5, 1 + 6 * i) = 1.0;
        A(6 + i * 5 + 1, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 2, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 3, 4 + 6 * i) = 1.0;
        A(6 + i * 5 + 4, 4 + 6 * i) = 1.0;

        if (contact_part[i] == Right_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;
            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;

            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 2, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 4, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Left_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.02;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.02;

            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
    }
    for (int i = 0; i < contact_index * 6; i++)
    {
        lb(i) = -1000;
        ub(i) = 1000;
    }

    for (int i = 0; i < contact_index; i++)
        ub(2 + 6 * i) = 0;

    QP_test.EnableEqualityCondition(0.01);
    QP_test.UpdateMinProblem(H, g);
    QP_test.UpdateSubjectToAx(A, lbA, ubA);
    QP_test.UpdateSubjectToX(lb, ub);
    VectorXd force_redistribute = QP_test.SolveQPoases(200);

    std::cout << "Contact Force now :  " << std::endl;
    std::cout << ContactForce__ << std::endl;
    std::cout << "Contact Force Redistribution : " << std::endl;
    std::cout << force_redistribute << std::endl;

    VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
    result_temp = force_redistribute;
    return torque_contact_;
}
*/

VectorQd Wholebody_controller::CP_control_init(double dT)
{
    double w_ = sqrt(9.81 / rk_.com_.pos(2));
    double b_ = exp(w_ * dT);

    Vector2d CP_displace;
    CP_displace(0) = 0.0;
    CP_displace(1) = 0.015;

    CP_ref[0] = rk_.com_.pos.segment(0, 2);
    CP_ref[1] = rk_.link_[Left_Foot].xpos.segment(0, 2) - CP_displace;
    CP_ref[2] = rk_.link_[Right_Foot].xpos.segment(0, 2) + CP_displace;
    CP_ref[3] = rk_.com_.pos.segment(0, 2);
}

VectorQd Wholebody_controller::CP_controller()
{
}

Vector6d Wholebody_controller::zmp_controller(Vector2d ZMP, double height)
{
    double w_ = sqrt(9.81 / rk_.com_.pos(2));
    Vector3d desired_accel;
    desired_accel.segment(0, 2) = pow(w_, 2) * (rk_.com_.pos.segment(0, 2) - ZMP);
    desired_accel(2) = 0.0;
    Vector3d desired_vel = rk_.com_.vel + desired_accel * abs(d_time_);
    desired_vel(2) = 0.0;
    Vector3d desired_pos = rk_.com_.pos + desired_vel * abs(d_time_);
    desired_pos(2) = height;
    Eigen::Vector3d kp_, kd_;
    kp_ << 400, 400, 400;
    kd_ << 40, 40, 40;
    Vector3d fstar = getfstar(kp_, kd_, desired_pos, rk_.com_.pos, desired_vel, rk_.com_.vel);

    Vector3d fstar_r;
    fstar_r(0) = -ZMP(1) / (rk_.com_.mass * 9.81);
    fstar_r(1) = -ZMP(0) / (rk_.com_.mass * 9.81);
    fstar_r(2) = 0;

    Vector6d r_z;
    r_z.segment(0, 3) = fstar;
    r_z.segment(3, 3) = fstar_r;

    return r_z;
}

VectorQd Wholebody_controller::gravity_compensation_torque(bool fixed, bool redsvd)
{
    ROS_DEBUG_ONCE("gravity torque calc start ");
    G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        G -= rk_.link_[i].Jac_COM_p.transpose() * rk_.link_[i].Mass * Grav_ref;
    }
    if (fixed)
        return G.segment(6, MODEL_DOF);

    Eigen::MatrixXd J_g;
    J_g.setZero(MODEL_DOF, MODEL_DOF + 6);
    J_g.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();

    Eigen::VectorXd torque_grav(MODEL_DOF);
    Eigen::MatrixXd aa = J_g * A_matrix_inverse * N_C * J_g.transpose();
    /*
    double epsilon = 1e-7;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(aa.cols(), aa.rows()) *svd.singularValues().array().abs()(0);
    Eigen::MatrixXd ppinv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

    Eigen::MatrixXd ppinv = aa.completeOrthogonalDecomposition().pseudoInverse();
    torque_grav = (J_g*A_matrix.inverse()*N_C*J_g.transpose()).completeOrthogonalDecomposition().pseudoInverse()*J_g*A_matrix.inverse()*N_C*G;
    torque_grav.setZero();
    Eigen::MatrixXd ppinv = DyrosMath::pinv_QR(aa);
    */
    Eigen::MatrixXd ppinv;
    double epsilon = 1e-7;
    if (redsvd)
    {
        RedSVD::RedSVD<Eigen::MatrixXd> svd(aa);
        double tolerance = epsilon * std::max(aa.cols(), aa.rows()) * svd.singularValues().array().abs()(0);
        ppinv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
    else
    {
        ppinv = DyrosMath::pinv_SVD(aa);
    }

    Eigen::MatrixXd tg_temp = ppinv * J_g * A_matrix_inverse * N_C;
    torque_grav = tg_temp * G;

    ROS_DEBUG_ONCE("gravity torque calc end ");
    return torque_grav;
}

VectorQd Wholebody_controller::task_control_torque(MatrixXd J_task, VectorXd f_star_)
{

    ROS_DEBUG_ONCE("task torque calc start ");
    task_dof = J_task.rows();

    //Task Control Torque;
    J_task_T.resize(MODEL_DOF + 6, task_dof);
    J_task_T.setZero();
    lambda_inv.resize(task_dof, task_dof);
    lambda_inv.setZero();
    lambda.resize(task_dof, task_dof);
    lambda.setZero();

    J_task_T = J_task.transpose();

    lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

    lambda = lambda_inv.inverse();
    J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

    Q = J_task_inv_T * Slc_k_T;
    Q_T_ = Q.transpose();

    Q_temp = Q * W_inv * Q_T_;

    Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

    //_F=lambda*(f_star);
    //Jtemp=J_task_inv_T*Slc_k_T;
    //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
    //Q.svd(s2,u2,v2);

    VectorQd torque_task;
    if (task_force_control)
    {
        VectorXd F_;
        F_.resize(task_dof);
        F_ = lambda * task_selection_matrix * f_star_;
        VectorXd F_2;
        F_2 = F_ + task_desired_force;
        torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;
    }
    else if (task_force_control_feedback)
    {
        VectorXd F_;
        F_.resize(task_dof);

        static double right_i, left_i;

        double pd = 0.1;
        double pi = 4.0;

        double left_des = -50.0;
        double right_des = 50.0;

        double right_err = task_desired_force(10) + task_feedback_reference(1);
        double left_err = task_desired_force(16) + task_feedback_reference(7);

        right_i += right_err * d_time_;
        left_i += left_err * d_time_;

        VectorXd fc_fs;
        fc_fs = task_desired_force;
        fc_fs.setZero();

        fc_fs(10) = pd * right_err + pi * right_i;
        fc_fs(16) = pd * left_err + pi * left_i;
        F_ = lambda * (task_selection_matrix * f_star_ + fc_fs);

        VectorXd F_2;
        F_2 = F_ + task_desired_force;

        torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;
    }
    else if (zmp_control)
    {
        int zmp_dof = 6;
        VectorXd F_;
        F_.resize(task_dof);
        task_selection_matrix.setIdentity(task_dof, task_dof);
        task_selection_matrix.block(0, 0, 2, 2).setZero();

        F_ = lambda * task_selection_matrix * f_star_;
        VectorXd F_2;

        Vector2d Fd_com = zmp_gain * 9.81 / 0.811 * (rk_.com_.pos.segment(0, 2) - ZMP_task) * rk_.com_.mass;
        task_desired_force.setZero(task_dof);

        task_desired_force.segment(0, 2) = Fd_com;
        //task_desired_force(3) = ZMP_task(1) * (com_.mass * 9.81);
        //task_desired_force(4) = ZMP_task(0) * (com_.mass * 9.81);

        F_2 = F_ + task_desired_force;

        torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;
    }
    else
    {
        torque_task = W_inv * Q_T_ * Q_temp_inv * (lambda * (f_star_));
    }

    //W.svd(s,u,v);
    //V2.resize(28,6);
    //V2.zero();

    ROS_DEBUG_ONCE("task torque calc end ");

    return torque_task;
}
/*
VectorQd Wholebody_controller::task_control_torque_custom_force(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  F_ = lambda * selection_matrix * f_star_;

  VectorXd F_2;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

VectorQd Wholebody_controller::task_control_torque_custom_force_feedback(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  static double right_i, left_i;

  double pd = 0.1;
  double pi = 4.0;

  double left_des = -50.0;
  double right_des = 50.0;

  double right_err = desired_force(10) + ft_hand(1);
  double left_err = desired_force(16) + ft_hand(7);

  right_i += right_err * d_time_;
  left_i += left_err * d_time_;

  VectorXd fc_fs; // = desired_force;
  fc_fs = desired_force;
  fc_fs.setZero();

  fc_fs(10) = pd * right_err + pi * right_i;
  fc_fs(16) = pd * left_err + pi * left_i;

  //std::cout << "right : " << fc_fs(10) << std::endl;
  //std::cout << "left : " << fc_fs(16) << std::endl;

  F_ = lambda * (selection_matrix * f_star_ + fc_fs);

  //F_ = selection_matrix * lambda * f_star_;

  VectorXd F_2;

  //desired_force(10) = desired_force(10) + right_des;
  //desired_force(16) = desired_force(16) + left_des;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}
*/
void Wholebody_controller::set_force_control(MatrixXd selection_matrix, VectorXd desired_force)
{
    task_force_control = true;
    task_selection_matrix = selection_matrix;
    task_desired_force = desired_force;
}
void Wholebody_controller::set_force_control_feedback(MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{
    task_force_control_feedback = true;
    task_selection_matrix = selection_matrix;
    task_desired_force = desired_force;
    task_feedback_reference = ft_hand;
}

void Wholebody_controller::set_zmp_control(Vector2d ZMP, double gain)
{
    zmp_control = true;
    ZMP_task = ZMP;
    zmp_gain = gain;
}

Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now)
{

    ROS_DEBUG_ONCE("fstar calc");
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kp(i) * (p_desired(i) - p_now(i)) + kd(i) * (d_desired(i) - d_now(i));
    }

    return fstar_;
}

Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now)
{

    ROS_DEBUG_ONCE("fstar calc");
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw_radian);
    Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(r_now, r_desired);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kp(i) * angle_d_global(i) - kd(i) * w_now(i));
    }

    return fstar_;
}

Vector3d Wholebody_controller::getfstar_tra(int link_id, Vector3d kpt, Vector3d kdt)
{
    ROS_DEBUG_ONCE("fstar calc");
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kpt(i) * (rk_.link_[link_id].x_traj(i) - rk_.link_[link_id].xpos(i)) + kdt(i) * (rk_.link_[link_id].v_traj(i) - rk_.link_[link_id].v(i));

        //std::cout << i << "\t traj : " << rk_.link_[link_id].x_traj(i) << "\t pos : " << rk_.link_[link_id].xpos(i) << "\t init : " << rk_.link_[link_id].x_init(i) << std::endl;
    }
    return fstar_;
}

Vector3d Wholebody_controller::getfstar_rot(int link_id, Vector3d kpa, Vector3d kda)
{
    ROS_DEBUG_ONCE("fstar calc");
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw_radian);

    Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(rk_.link_[link_id].Rotm, rk_.link_[link_id].r_traj);

    //Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw_radian);

    //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(link_[link_id].Rotm, link_[link_id].r_traj);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kpa(i) * angle_d_global(i) - kda(i) * rk_.link_[link_id].w(i));
    }
    /*
  std::cout << "fstar check " << std::endl
            << link_[link_id].name << std::endl
            << " rotation now " << std::endl
            << link_[link_id].Rotm << std::endl
            << "desired rotation " << std::endl
            << link_[link_id].r_traj << std::endl
            << "angle d " << std::endl
            << angle_d << std::endl
            << "global angle d " << std::endl
            << angle_d_global << std::endl
            << "fstar " << std::endl
            << fstar_ << std::endl
            << " ////////////////////////////////////////////////////////////////" << std::endl;
  */

    return fstar_;
}

Vector6d Wholebody_controller::getfstar6d(int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda)
{
    Vector6d f_star;
    f_star.segment(0, 3) = getfstar_tra(link_id, kpt, kdt);
    f_star.segment(3, 3) = getfstar_rot(link_id, kpa, kda);
    return f_star;
}

VectorQd Wholebody_controller::contact_force_custom(VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired)
{
    JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
    svd_U = svd.matrixU();

    MatrixXd V2;

    int singular_dof = 6;
    int contact_dof = J_C.rows();
    V2.setZero(MODEL_DOF, contact_dof - singular_dof);
    V2 = svd_U.block(0, MODEL_DOF - contact_dof + singular_dof, MODEL_DOF, contact_dof - singular_dof);

    MatrixXd Scf_;
    Scf_.setZero(contact_dof - singular_dof, contact_dof);
    Scf_.block(0, 0, contact_dof - singular_dof, contact_dof - singular_dof).setIdentity();

    // std::cout << contact_force_desired << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << contact_force_now << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << std::endl;

    VectorXd desired_force = contact_force_desired - contact_force_now;

    MatrixXd temp = Scf_ * J_C_INV_T * Slc_k_T * V2;
    MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
    MatrixXd Vc_ = V2 * temp_inv;

    VectorXd reduced_desired_force = Scf_ * desired_force;
    VectorQd torque_contact_ = Vc_ * reduced_desired_force;

    return torque_contact_;
}

VectorXd Wholebody_controller::get_contact_force(VectorQd command_torque)
{
    VectorXd contactforce = J_C_INV_T * Slc_k_T * command_torque - Lambda_c * J_C * A_matrix_inverse * G;
    return contactforce;
}

VectorQd Wholebody_controller::contact_force_redistribution_torque(double yaw_radian, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    //Contact Jacobian task : rightfoot to leftfoot

    int contact_dof_ = J_C.rows();

    VectorQd torque_contact_;

    ForceRedistribution.setZero();

    if (contact_dof_ == 12)
    {

        Vector12d ContactForce_ = J_C_INV_T * Slc_k_T * command_torque - Lambda_c * J_C * A_matrix_inverse * G;

        Vector3d P1_, P2_;

        P1_ = rk_.link_[Right_Foot].xpos;
        P2_ = rk_.link_[Left_Foot].xpos;

        Matrix3d Rotyaw = DyrosMath::rotateWithZ(-yaw_radian);

        Vector3d P1_local, P2_local;
        P1_local = Rotyaw * P1_;
        P2_local = Rotyaw * P2_;

        MatrixXd force_rot_yaw;
        force_rot_yaw.setZero(12, 12);
        for (int i = 0; i < 4; i++)
        {
            force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
        }

        Vector6d ResultantForce_;
        ResultantForce_.setZero();

        Vector12d ResultRedistribution_;
        ResultRedistribution_.setZero();

        torque_contact_.setZero();

        double eta_cust = 0.99;
        double foot_length = 0.26;
        double foot_width = 0.1;

        Vector12d ContactForce_Local_yaw;
        ContactForce_Local_yaw = force_rot_yaw * ContactForce_;

        ZMP_pos = GetZMPpos(P1_local, P2_local, ContactForce_Local_yaw);

        ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.8, 0.8, P1_local, P2_local, ContactForce_Local_yaw, ResultantForce_, ResultRedistribution_, eta);

        ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        svd_U = svd.matrixU();

        MatrixXd V2;

        int singular_dof = 6;
        int contact_dof = J_C.rows();

        V2.setZero(MODEL_DOF, singular_dof);
        V2 = svd_U.block(0, MODEL_DOF - contact_dof + 6, MODEL_DOF, contact_dof - 6);

        Vector12d desired_force;

        desired_force.setZero();
        MatrixXd Scf_;

        bool right_master = false;

        if (right_master)
        {
            Scf_.setZero(6, 12);
            Scf_.block(0, 0, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i) = -ContactForce_(i) + ForceRedistribution(i);
            }
        }
        else
        {

            Scf_.setZero(6, 12);
            Scf_.block(0, 6, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6);
            }
        }

        MatrixXd temp = Scf_ * J_C_INV_T * Slc_k_T * V2;
        MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
        MatrixXd Vc_ = V2 * temp_inv;

        Vector6d reduced_desired_force = Scf_ * desired_force;
        torque_contact_ = Vc_ * reduced_desired_force;
    }
    else
    {
        torque_contact_.setZero();
    }

    return torque_contact_;
}

Vector3d Wholebody_controller::GetZMPpos(Vector3d P_right, Vector3d P_left, Vector12d ContactForce)
{

    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();

    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    zmp_pos(0) = (-ContactForce(4) - (P_right(2) - P_(2)) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - (P_left(2) - P_(2)) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
    zmp_pos(1) = (ContactForce(3) - (P_right(2) - P_(2)) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - (P_left(2) - P_(2)) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));

    return (zmp_pos);
}

void Wholebody_controller::ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    ROS_DEBUG_ONCE("force redistribution start");
    Eigen::MatrixXd W;
    W.setZero(6, 12);

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    for (int i = 0; i < 3; i++)
    {
        W(i, i) = 1.0;
        W(i + 3, i + 3) = 1.0;
        W(i, i + 6) = 1.0;
        W(i + 3, i + 9) = 1.0;

        for (int j = 0; j < 3; j++)
        {
            W(i + 3, j) = P1_hat(i, j);
            W(i + 3, j + 6) = P2_hat(i, j);
        }
    }
    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }

    //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else //sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    eta = eta_s;
    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }

    if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
    {
        eta = 0.5;
    }

    //std::cout<<"ETA :: "<<eta<<std::endl;

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

    ForceRedistribution(0) = eta * ResultantForce(0);
    ForceRedistribution(1) = eta * ResultantForce(1);
    ForceRedistribution(2) = eta * ResultantForce(2);
    ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
    ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
    ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
    ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
    ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
    ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
    ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
    ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
    ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

void Wholebody_controller::ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    Eigen::MatrixXd W;
    W.setZero(6, 12);

    W.block(0, 0, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(0, 6, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(3, 0, 3, 3) = P1_hat;
    W.block(3, 6, 3, 3) = P2_hat;

    // link_[Right_Leg].Rotm;

    // for (int i = 0; i < 3; i++)
    // {
    //   W(i, i) = 1.0;
    //   W(i + 3, i + 3) = 1.0;
    //   W(i, i + 6) = 1.0;
    //   W(i + 3, i + 9) = 1.0;

    //   for (int j = 0; j < 3; j++)
    //   {
    //     W(i + 3, j) = P1_hat(i, j);
    //     W(i + 3, j + 6) = P2_hat(i, j);
    //   }
    // }

    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    double A_threshold = 0.001;
    ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0.");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1.");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2.");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3.");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4.");
            }
        }
    }

    ////printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0;");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1;");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2;");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3;");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4;");
            }
        }
    }

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0,");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1,");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2,");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3,");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4,");
            }
        }
    }
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }
    else
    {
        eta = eta_s;
    }

    if (eta_ub < eta_lb) //임시...roundoff error로 정확한 해가 안나올때
    {
        //printf("-");
    }
    else if (sqrt(eta_ub * eta_ub + eta_lb * eta_lb) > 1.0) //너무 큰 경계값이 섞여 있을 때
    {
        //printf("_");
    }

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

    ForceRedistribution(0) = eta * ResultantForce(0);
    ForceRedistribution(1) = eta * ResultantForce(1);
    ForceRedistribution(2) = eta * ResultantForce(2);
    ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
    ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
    ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
    ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
    ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
    ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
    ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
    ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
    ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}
