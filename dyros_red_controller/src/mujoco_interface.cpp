#include "dyros_red_controller/mujoco_interface.h"

namespace dyros_red_controller {

mujoco_interface::mujoco_interface(ros::NodeHandle &nh, double Hz):
    ControlBase(nh,Hz), rate_(Hz), dyn_hz(Hz)
{
    mujoco_joint_set_pub_=nh.advertise<sensor_msgs::JointState>("/mujoco_ros_interface/joint_set",1);
    mujoco_sim_command_pub_=nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim",100);
    mujoco_sim_command_sub_=nh.subscribe("/mujoco_ros_interface/sim_command_sim2con",100,&mujoco_interface::simCommandCallback,this);


    mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states",1,&mujoco_interface::jointStateCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time",1,&mujoco_interface::simTimeCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_sensor_state_sub_=nh.subscribe("/mujoco_ros_interface/sensor_states",1,&mujoco_interface::sensorStateCallback,this,ros::TransportHints().tcpNoDelay(true));

    mujoco_joint_set_msg_.position.resize(total_dof_);
    mujoco_joint_set_msg_.effort.resize(total_dof_);

    ros::Rate poll_rate(100);

    ROS_INFO("Waiting for connection with Mujoco Ros interface ");

    while(mujoco_sim_command_pub_.getNumSubscribers() ==0 &&ros::ok())
        poll_rate.sleep();

    ROS_INFO("Mujoco Ros interface Connected");

    mujoco_sim_time =0.0;

}
void mujoco_interface::simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
  mujoco_sim_time = msg->data;


  if(mujoco_sim_time < 0.00001){

    mujoco_sim_last_time = 0.0;

  }


}

void mujoco_interface::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    ROS_INFO_ONCE("jointCB");
    static bool once = true;
    for(int i=0;i<total_dof_;i++)
    {
        for(int j=0; j<msg->name.size();j++){
            if(DyrosRedModel::JOINT_NAME[i] == msg->name[j].data())
            {
              if(once ==true){

              std::cout<<i<<" :   "<<DyrosRedModel::JOINT_NAME[i]<<" = "<<j<<" : "<<msg->name[j].data()<<std::endl;
              }
                q_(i) = msg->position[j];
                q_virtual_(i+6) = msg->position[j];

                q_dot_(i) = msg->velocity[j];
                q_dot_virtual_(i+6) = msg->velocity[j];

                torque_(i) = msg->effort[j];

            }
        }

        joint_name_mj[i] = msg->name[i+6].data();
    }


    //virtual joint

    for(int i=0;i<6;i++){
      q_virtual_(i) = msg->position[i];
      q_dot_virtual_(i) = msg->velocity[i];
    }

    once=false;
}


void mujoco_interface::sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr &msg)
{
    ROS_INFO_ONCE("sensorCB");
    for(int i=0;i<msg->sensor.size();i++){
        if(msg->sensor[i].name=="Gyro_Pelvis_IMU"){
            for(int j=0;j<3;j++){
                q_dot_virtual_(j+3)=msg->sensor[i].data[j];
            }

        }

    }
    ROS_INFO_ONCE("sensor data   %f",q_dot_virtual_(0));
}


void mujoco_interface::simCommandCallback(const std_msgs::StringConstPtr &msg)
{


}

void mujoco_interface::update()
{
    ControlBase::update();
}

void mujoco_interface::compute()
{
    ControlBase::compute();
}

void mujoco_interface::writeDevice()
{
    for(int i=0;i<total_dof_;i++)
    {
        for(int j=0; j<total_dof_;j++){
            if(DyrosRedModel::JOINT_NAME[i] ==joint_name_mj[j])
            {
       mujoco_joint_set_msg_.effort[j] = torque_desired[i];

            }
        }
    }



    mujoco_joint_set_msg_.header.stamp=ros::Time::now();
    mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);

    mujoco_sim_last_time = mujoco_sim_time;
}

void mujoco_interface::wait()
{
    ros::Rate poll_rate(1000);

    while((mujoco_sim_time<(mujoco_sim_last_time+1.0/dyn_hz))&&ros::ok()){
        //ROS_INFO("WAIT WHILE");
        ros::spinOnce();
        poll_rate.sleep();

    }


}


}


