/* =========== */
/*  Libraries  */
/* =========== */
#include "../include/gmr_intro_poo/gmr_intro_poo.hpp"

/* ============================ */
/*  RobotClass, Implementation  */
/* ============================ */
RobotClass::RobotClass(ros::NodeHandle *nh){
    _nh = nh;
    
    // Params Initialization
    _nh->param<std::string>("topic_name_left_rpm", _params.topic_name_left_rpm, "/left_rpm");
    _nh->param<std::string>("topic_name_right_rpm", _params.topic_name_right_rpm, "right_rpm");
    _nh->param("/axle_track", _params.axle_track, 1.0);
    _nh->param("/gear_ratio", _params.gear_ratio, 1.0);
    _nh->param("/wheel_radius", _params.wheel_radius, 1.0);
    
    // Topics Subscription
    _sub_left = _nh->subscribe(_params.topic_name_left_rpm, 1, &RobotClass::subLeft, this);
    _sub_right = _nh->subscribe(_params.topic_name_right_rpm, 1, &RobotClass::subRight, this);

    _vel_m_s.left = 0.0;
    _vel_m_s.right = 0.0;
}

void RobotClass::subLeft(const std_msgs::Float32::ConstPtr &msg){
    _vel_m_s.left = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    //ROS_INFO_STREAM( "velocidade linear esquerda: " << _vel_m_s.left );
    //ROS_INFO_STREAM("motor_left, angular speed (RPM): " << msg->data << "\t" << "motor_left, linear velocity (m/s): " << _vel_m_s.left);
    ROS_INFO_STREAM("motor_left,  ang_speed (RPM): " << msg->data << "\t" << "motor_left,  lin_vel (m/s): " << _vel_m_s.left);

}

void RobotClass::subRight(const std_msgs::Float32::ConstPtr &msg){
    _vel_m_s.right = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    //ROS_INFO_STREAM( "velocidade linear direita: " << _vel_m_s.right );
    //ROS_INFO_STREAM("motor_right, angular speed (RPM): " << msg->data << "\t" << "motor_right, lin velocity (m/s): " << _vel_m_s.left);
    ROS_INFO_STREAM("motor_right, ang_speed (RPM): " << msg->data << "\t" << "motor_right, lin_vel (m/s): " << _vel_m_s.left);
}