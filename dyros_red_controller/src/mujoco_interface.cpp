#include "dyros_red_controller/mujoco_interface.h"

MujocoInterface::MujocoInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    ros::NodeHandle nh = dc.nh;

    mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 1);
    mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
    mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &MujocoInterface::simCommandCallback, this);

    mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states", 1, &MujocoInterface::jointStateCallback, this, ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time", 1, &MujocoInterface::simTimeCallback, this, ros::TransportHints().tcpNoDelay(true));
    mujoco_sensor_state_sub_ = nh.subscribe("/mujoco_ros_interface/sensor_states", 1, &MujocoInterface::sensorStateCallback, this, ros::TransportHints().tcpNoDelay(true));

    mujoco_joint_set_msg_.position.resize(MODEL_DOF);
    mujoco_joint_set_msg_.torque.resize(MODEL_DOF);
}

void MujocoInterface::updateState()
{
    ros::spinOnce();
    //updateKinematics();
}

void MujocoInterface::sendCommand(Eigen::VectorQd command)
{
    mujoco_joint_set_msg_.MODE = 1;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < MODEL_DOF; j++)
        {
            if (RED::ACTUATOR_NAME[i] == joint_name_mj[j])
            {
                mujoco_joint_set_msg_.torque[j] = command[i];
            }
        }
    }

    mujoco_joint_set_msg_.header.stamp = ros::Time::now();
    mujoco_joint_set_msg_.time = control_time_;
    mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
    mujoco_sim_last_time = mujoco_sim_time;
}

void MujocoInterface::connect()
{
    //std::cout << "________________________________________________________________________________\n\n";

    //std::cout << "\tConnecting to Mujoco ..." << std::flush;

    int w_y, w_x;
    getyx(stdscr, w_y, w_x);
    mvprintw(w_y + 2, 10, "Press any key to stop ");
    mvprintw(w_y + 1, 10, "Connecting to Mujoco ");
    refresh();
    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();
    int cnt = 0;
    int kbhit = -1;
    while (!mujoco_ready & ros::ok())
    {
        kbhit = getch();
        cnt++;
        r.sleep();
        ros::spinOnce();
        if ((cnt % 10) == 0)
        {
            addch('.');
            refresh();
        }
        if (!(kbhit == -1))
        {
            printw("Stopping");
            refresh();
            break;
        }
        if ((ros::Time::now().toSec() - start_time.toSec()) > 60.0)
        {
            printw("Stopping");
            refresh();
            break;
        }
    }
    start_time = ros::Time::now();
    if (mujoco_ready)
    {
        while (!mujoco_init_receive & ros::ok())
        {
            r.sleep();
            ros::spinOnce();
            if ((ros::Time::now().toSec() - start_time.toSec()) > 1.0)
                break;
        }
    }

    if ((!mujoco_init_receive) && (!mujoco_ready))
    {
        //std::cout << "\tConnection failed. \n"                  << std::flush;
    }
    else if (mujoco_init_receive && mujoco_ready)
    {
        //std::cout << "\tConnected! \n"                  << std::flush;
    }

    mujoco_init_receive = false;
    mujoco_ready = false;
}

void MujocoInterface::playMujoco()
{
    std_msgs::String rst_msg_;
    rst_msg_.data = "PAUSE";
    mujoco_sim_command_pub_.publish(rst_msg_);
}

void MujocoInterface::simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
    mujoco_sim_time = msg->data;
    control_time_ = mujoco_sim_time;
}

void MujocoInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 0; j < msg->name.size(); j++)
        {
            if (RED::ACTUATOR_NAME[i] == msg->name[j].data())
            {
                q_(i) = msg->position[j];
                q_virtual_(i + 6) = msg->position[j];
                q_dot_(i) = msg->velocity[j];
                q_dot_virtual_(i + 6) = msg->velocity[j];
                q_ddot_virtual_(i + 6) = msg->effort[j];
                torque_(i) = msg->effort[j];
            }
        }

        joint_name_mj[i] = msg->name[i + 6].data();
    }

    //virtual joint

    for (int i = 0; i < 6; i++)
    {
        q_virtual_(i) = msg->position[i];
        q_dot_virtual_(i) = msg->velocity[i];
        q_ddot_virtual_(i) = msg->effort[i];
    }

    q_virtual_(MODEL_DOF + 6) = msg->position[MODEL_DOF + 6];

    //tf::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(total_dof_ + 6));
    //q.normalize();
}

void MujocoInterface::sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr &msg)
{
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "Gyro_Pelvis_IMU")
        {
            for (int j = 0; j < 3; j++)
            {
                //q_dot_virtual_(j+3)=msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RF_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_foot_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RF_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_foot_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LF_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //left_foot_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LF_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //left_foot_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LH_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //left_hand_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "LH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //left_hand_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Force_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_hand_ft_(j) = msg->sensor[i].data[j];
            }
        }
    }
    for (int i = 0; i < msg->sensor.size(); i++)
    {
        if (msg->sensor[i].name == "RH_Torque_sensor")
        {
            for (int j = 0; j < 3; j++)
            {
                //right_hand_ft_(j + 3) = msg->sensor[i].data[j];
            }
        }
    }
}

void MujocoInterface::simCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    //ROS_INFO("CB from simulator : %s", buf.c_str());
    if (buf == "RESET")
    {
        //parameterInitialize();
        mujoco_sim_last_time = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        mujoco_sim_time = 0.0;
        control_time_ = 0.0;
        mujoco_reset = true;
    }
}