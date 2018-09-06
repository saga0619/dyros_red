#include "dyros_red_controller/wholebody_controller.h"



namespace dyros_red_controller
{

Wholebody_controller::Wholebody_controller(DyrosRedModel& model, const VectorQd& current_q, const double hz, const double& control_time):
  total_dof_(DyrosRedModel::MODEL_DOF), model_(model),
  current_q_(current_q), hz_(hz), control_time_(control_time),
  start_time_{}, end_time_{}, target_arrived_{true,true,true,true} {
  //debug_.open("/home/suhan/jet_test.txt");
}

void Wholebody_controller::update_dynamics_mode(int mode){

  ROS_DEBUG_ONCE("dynamics update start ");

  A_matrix.setZero(total_dof_+6,total_dof_+6);
  A_matrix = model_.A_;
  A_matrix_inverse = A_matrix.inverse();

  Grav_ref.setZero(3);
  Grav_ref(2) = -9.81;

  J_COM.setZero(3,total_dof_+6);

  for(int i=0;i<total_dof_+1;i++){
    J_COM = J_COM + model_.link_[i].Jac_COM_p*model_.link_[i].Mass;
  }
  J_COM = J_COM / model_.total_mass;


  if(mode == DOUBLE_SUPPORT){
    Eigen::Vector3d left_leg_contact, right_leg_contact;

    left_leg_contact<<0,0,-0.1368;
    right_leg_contact<<0,0,-0.1368;

    model_.Link_Set_Contact(model_.Left_Foot,left_leg_contact);
    model_.Link_Set_Contact(model_.Right_Foot,right_leg_contact);


    J_C.setZero(12, total_dof_+6);
    J_C.block(0, 0, 6, total_dof_+6) = model_.link_[model_.Right_Foot].Jac_Contact;
    J_C.block(6, 0, 6, total_dof_+6) = model_.link_[model_.Left_Foot].Jac_Contact;

    Lambda_c=(J_C*A_matrix_inverse*(J_C.transpose())).inverse();
    J_C_INV_T = Lambda_c*J_C*A_matrix_inverse;

    N_C.setZero(total_dof_+6, total_dof_+6);
    I37.setIdentity(total_dof_+6, total_dof_+6);
    N_C =I37-J_C.transpose()*J_C_INV_T;



    //Control only COM for now.
    J_task = J_COM;
  }


  ROS_DEBUG_ONCE("dynamics update end ");


}

VectorQd Wholebody_controller::gravity_compensation_torque()
{
  ROS_DEBUG_ONCE("gravity torque calc start ");
  Eigen::VectorXd Gtemp;
  G.setZero(total_dof_+6);
  Gtemp.setZero(total_dof_+6);

  for(int i=0;i<total_dof_+1;i++){
    Gtemp = G - model_.link_[i].Jac_COM_p.transpose()*model_.link_[i].Mass*Grav_ref;
    G=Gtemp;
  }

  Eigen::MatrixXd J_g;
  J_g.setZero(total_dof_, total_dof_+6);
  J_g.block(0, 6, total_dof_, total_dof_).setIdentity();


  Eigen::VectorXd torque_grav(total_dof_);
  Eigen::MatrixXd aa = J_g*A_matrix_inverse*N_C*J_g.transpose();

  //std::cout << " aa size " << aa.rows() << "    " <<aa.cols() << std::endl;

/*
  double epsilon = 1e-7;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(aa.cols(), aa.rows()) *svd.singularValues().array().abs()(0);
  Eigen::MatrixXd ppinv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
*/
 // Eigen::MatrixXd ppinv = aa.completeOrthogonalDecomposition().pseudoInverse();

  //torque_grav = (J_g*A_matrix.inverse()*N_C*J_g.transpose()).completeOrthogonalDecomposition().pseudoInverse()*J_g*A_matrix.inverse()*N_C*G;

  //torque_grav.setZero();
  //ROS_WARN("test0");

  //Eigen::MatrixXd ppinv = DyrosMath::pinv_QR(aa);
  Eigen::MatrixXd ppinv = DyrosMath::pinv_SVD(aa);

  //std::cout << " ppinv cols : "<<(int)ppinv.cols() <<"   ppinv rows : "<< (int)ppinv.rows() <<std::endl;
  //std::cout << " ppinv2 cols : "<<(int)ppinv2.cols() <<"   ppinv2 rows : "<< (int)ppinv.rows() <<std::endl;

  //ROS_WARN("test1");
  Eigen::MatrixXd tg_temp;

 // ROS_WARN("test2");



  tg_temp = ppinv*J_g*A_matrix_inverse*N_C;


  //ROS_WARN("test3");
  torque_grav = tg_temp*G;

  ROS_DEBUG_ONCE("gravity torque calc end ");
  return torque_grav;

}


VectorQd Wholebody_controller::task_control_torque(MatrixXd J_task, Vector6d f_star_){

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  Slc_k.setZero(total_dof_,total_dof_+6);


  Slc_k.block(0,6, total_dof_,total_dof_).setIdentity();


  Slc_k_T = Slc_k.transpose();


  //Task Control Torque;
  J_task_T.resize(total_dof_+6,task_dof);J_task_T.setZero();
  lambda_inv.resize(task_dof,task_dof);lambda_inv.setZero();
  lambda.resize(task_dof,task_dof);lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv=J_task*A_matrix_inverse*N_C*J_task_T;


  lambda=lambda_inv.inverse();
  J_task_inv_T=lambda*J_task*A_matrix_inverse*N_C;

  W=Slc_k*A_matrix_inverse*N_C*Slc_k_T;
  W_inv = DyrosMath::pinv_SVD(W);



  Q=J_task_inv_T*Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp=Q*W_inv*Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);

  //Jtemp=J_task_inv_T*Slc_k_T;

  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);



  JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);

  //std::cout<<"svd_U"<<std::endl<<svd.matrixU()<<std::endl;

  svd_U = svd.matrixU();



  //Q.svd(s2,u2,v2);


  VectorQd torque_task;
  torque_task=W_inv*Q_T_*Q_temp_inv*(lambda*(f_star_));


        //W.svd(s,u,v);
        //Show_matrix(s,"W svd S");
        //Show_matrix(u,"W svd u");
        //Show_matrix(v,"W svd v");

        //V2.resize(28,6);
        //V2.zero();

        /*
        for(int i=0;i<28;i++){
          for(int k=0;k<6;k++){
            V2(i,k)=u(i,k+22);
          }
        }
        */

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;

}

Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now)
{

  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  for(int i=0;i<3;i++){
    fstar_(i) = kp(i) *(p_desired(i) - p_now(i)) + kd(i)*(d_desired(i) - d_now(i));
  }

  return fstar_;

}


Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now)
{

  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  Vector3d s1,s2,s3,s1d,s2d,s3d,angle_d;

  s1(0)=r_now(0,0);s1(1)=r_now(1,0);s1(2)=r_now(2,0);
  s2(0)=r_now(0,1);s2(1)=r_now(1,1);s2(2)=r_now(2,1);
  s3(0)=r_now(0,2);s3(1)=r_now(1,2);s3(2)=r_now(2,2);

  s1d.setZero(); s2d.setZero(); s3d.setZero();
  s1d(0)=r_desired(0,0);s1d(1)=r_desired(1,0);s1d(2)=r_desired(2,0);
  s2d(0)=r_desired(0,1);s2d(1)=r_desired(1,1);s2d(2)=r_desired(2,1);
  s3d(0)=r_desired(0,2);s3d(1)=r_desired(1,2);s3d(2)=r_desired(2,2);
  angle_d=-(DyrosMath::skm(s1)*s1d+DyrosMath::skm(s2)*s2d+DyrosMath::skm(s3)*s3d)/2;

  for(int i=0;i<3;i++){
    fstar_(i)=(-kp(i)*angle_d(i)-kd(i)*w_now(i));
  }

  return fstar_;
}

VectorQd Wholebody_controller::contact_force_redistribution_torque(VectorQd command_torque,Eigen::Vector12d& ForceRedistribution)
{

  ROS_DEBUG_ONCE("contact redistribution start");

  Vector12d ContactForce_=J_C_INV_T * Slc_k_T * command_torque - Lambda_c*J_C*A_matrix_inverse*G;

  ROS_DEBUG_ONCE("contactForce calc");

  Vector3d P1_, P2_;

  P1_= model_.link_[model_.Right_Foot].xpos - model_.com_;
  P2_= model_.link_[model_.Left_Foot].xpos - model_.com_;

  Vector6d ResultantForce_;
  ResultantForce_.setZero();

  Vector12d ResultRedistribution_;
  ResultRedistribution_.setZero();

  VectorQd torque_contact_;
  torque_contact_.setZero();

  double eta_cust= 0.95;
  double foot_length = 0.26;
  double foot_width = 0.1;
  double eta;
  ForceRedistributionTwoContactMod2(0.95,foot_length,foot_width,1.0,0.8,0.8,P1_,P2_, ContactForce_ , ResultantForce_, ResultRedistribution_);

  ForceRedistribution = ResultRedistribution_;

  //std::cout << "Result redistribution " << std::endl;
  //std::cout << ResultRedistribution_ << std::endl <<std::endl;

  ROS_DEBUG_ONCE("contact redistribution end");


  MatrixXd V2;
  V2.setZero(total_dof_,task_dof);
  V2 = svd_U.block(0,total_dof_-task_dof,total_dof_,task_dof);

  Vector12d desired_force;

  desired_force.setZero();
  MatrixXd Scf_;
  Scf_.setZero(6,12);
  Scf_.block(0,6,6,6).setIdentity();

  for(int i=0;i<6;i++){
    desired_force(i+6) = -ContactForce_(i+6) + ResultRedistribution_(i+6);
  }

  Vector6d reduced_desired_force = Scf_*desired_force;

  MatrixXd temp = Scf_*J_C_INV_T*Slc_k_T*V2;

  MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);

  torque_contact_ = V2*temp_inv*reduced_desired_force;


  ContactForce_=J_C_INV_T * Slc_k_T *(command_torque+torque_contact_) - Lambda_c*J_C*A_matrix_inverse*G;



  //std::cout << "redistribution result" <<std::endl;
  //std::cout << ContactForce_ << std::endl;




  return torque_contact_;

  //torque_contact_

}


void Wholebody_controller::ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d& ResultantForce,  Eigen::Vector12d& ForceRedistribution)
{
  ROS_DEBUG_ONCE("force redistribution start");
  Eigen::MatrixXd W;
  W.setZero(6,12);

  Eigen::Matrix3d P1_hat, P2_hat;
  P1_hat=DyrosMath::skm(P1);
  P2_hat=DyrosMath::skm(P2);


  for(int i =0; i<3; i++)
  {
    W(i,i) = 1.0;
    W(i+3,i+3) = 1.0;
    W(i,i+6) = 1.0;
    W(i+3,i+9) = 1.0;

    for(int j=0; j<3; j++)
    {
      W(i+3,j) = P1_hat(i,j);
      W(i+3,j+6) = P2_hat(i,j);
    }
  }
  ResultantForce.resize(6);
  ResultantForce = W*F12;//F1F2;

  double eta_lb = 1.0 - eta_cust;
  double eta_ub = eta_cust;
  //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mx, A*eta + B < 0
  double A = (P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2);
  double B = ResultantForce(3) + P2(2)*ResultantForce(1) -P2(1)*ResultantForce(2);
  double C = ratio_y*footwidth/2.0*abs(ResultantForce(2));
  double a = A*A;
  double b = 2.0*A*B;
  double c = B*B-C*C;
  double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta My, A*eta + B < 0
  A = - (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2);
  B = ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2);
  C = ratio_x*footlength/2.0*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;
  sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
  A = - (P1(0) - P2(0))*ResultantForce(1) + (P1(1) - P2(1))*ResultantForce(0);
  B = ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1);
  C = staticFrictionCoeff*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;
  sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }
  //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

  double eta_s = (-ResultantForce(3) - P2(2)*ResultantForce(1) + P2(1)*ResultantForce(2))/((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2));

  double eta = eta_s;
  if(eta_s > eta_ub)
  {
    eta = eta_ub;
  }
  else if(eta_s < eta_lb)
  {
    eta = eta_lb;
  }


  if ((eta>eta_cust)||(eta<1.0 - eta_cust))
  {
    eta=0.5;
  }



  std::cout<<"ETA :: "<<eta<<std::endl;


  //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

  //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
  //double etaMx = eta*Mx1Mx2;
  //printf("%f %f \n", Mx1Mx2,etaMx);
  //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
  //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
  //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

  ForceRedistribution(0) = eta*ResultantForce(0);
  ForceRedistribution(1) = eta*ResultantForce(1);
  ForceRedistribution(2) = eta*ResultantForce(2);
  ForceRedistribution(3) = ((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2))*eta;
  ForceRedistribution(4) = (- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2))*eta;
  ForceRedistribution(5) = (- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1))*eta;
  ForceRedistribution(6) = (1.0-eta)*ResultantForce(0);
  ForceRedistribution(7) = (1.0-eta)*ResultantForce(1);
  ForceRedistribution(8) = (1.0-eta)*ResultantForce(2);
  ForceRedistribution(9) = (1.0-eta)*(((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2)));
  ForceRedistribution(10) = (1.0-eta)*((- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2)));
  ForceRedistribution(11) = (1.0-eta)*((- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1)));
  //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
  //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
  //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);




  ROS_DEBUG("force redistribution end");
}


void Wholebody_controller::ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d& ResultantForce,  Eigen::Vector12d& ForceRedistribution, double& eta)
{
  Eigen::MatrixXd W;
  W.setZero(6,12);

  Eigen::Matrix3d P1_hat, P2_hat;
  P1_hat=DyrosMath::skm(P1);
  P2_hat=DyrosMath::skm(P2);


  for(int i =0; i<3; i++)
  {
    W(i,i) = 1.0;
    W(i+3,i+3) = 1.0;
    W(i,i+6) = 1.0;
    W(i+3,i+9) = 1.0;

    for(int j=0; j<3; j++)
    {
      W(i+3,j) = P1_hat(i,j);
      W(i+3,j+6) = P2_hat(i,j);
    }
  }
  ResultantForce.resize(6);
  ResultantForce = W*F12;//F1F2;

  double eta_lb = 1.0 - eta_cust;
  double eta_ub = eta_cust;
  double A_threshold = 0.001;
  ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta Mx, A*eta + B < 0
  double A = (P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2);
  double B = ResultantForce(3) + P2(2)*ResultantForce(1) -P2(1)*ResultantForce(2);
  double C = ratio_y*footwidth/2.0*abs(ResultantForce(2));
  double a = A*A;
  double b = 2.0*A*B;
  double c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0.");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1.");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2.");
      }
    }
    else//sol_eta2 이 upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3.");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
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
  A = - (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2);
  B = ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2);
  C = ratio_x*footlength/2.0*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0;");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1;");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2;");
      }
    }
    else//sol_eta2 이 upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3;");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
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
  A = - (P1(0) - P2(0))*ResultantForce(1) + (P1(1) - P2(1))*ResultantForce(0);
  B = ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1);
  C = staticFrictionCoeff*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0,");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1,");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2,");
      }
    }
    else//sol_eta2 이 upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3,");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("4,");
      }
    }
  }
  //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

  double eta_s = (-ResultantForce(3) - P2(2)*ResultantForce(1) + P2(1)*ResultantForce(2))/((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2));

  if(eta_s > eta_ub)
  {
    eta = eta_ub;
  }
  else if(eta_s < eta_lb)
  {
    eta = eta_lb;
  }
  else
  {
    eta = eta_s;
  }

  if(eta_ub < eta_lb) //임시...roundoff error로 정확한 해가 안나올때
  {
    //printf("-");
  }
  else if(sqrt(eta_ub*eta_ub + eta_lb*eta_lb) > 1.0) //너무 큰 경계값이 섞여 있을 때
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

  ForceRedistribution(0) = eta*ResultantForce(0);
  ForceRedistribution(1) = eta*ResultantForce(1);
  ForceRedistribution(2) = eta*ResultantForce(2);
  ForceRedistribution(3) = ((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2))*eta;
  ForceRedistribution(4) = (- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2))*eta;
  ForceRedistribution(5) = (- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1))*eta;
  ForceRedistribution(6) = (1.0-eta)*ResultantForce(0);
  ForceRedistribution(7) = (1.0-eta)*ResultantForce(1);
  ForceRedistribution(8) = (1.0-eta)*ResultantForce(2);
  ForceRedistribution(9) = (1.0-eta)*(((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2)));
  ForceRedistribution(10) = (1.0-eta)*((- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2)));
  ForceRedistribution(11) = (1.0-eta)*((- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1)));
  //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
  //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
  //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}


}
