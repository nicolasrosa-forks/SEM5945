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
    _nh->param<std::string>("topic_name_left_rpm", _params.topic_name_left_rpm, "/left_rpm");  //_nh->param("topic_name_left_rpm", _params.topic_name_left_rpm, std::string("/left_rpm"));
    _nh->param<std::string>("topic_name_right_rpm", _params.topic_name_right_rpm, "/right_rpm");  //_nh->param("topic_name_right_rpm", _params.topic_name_left_rpm, std::string("/left_rpm"));
    _nh->param("/axle_track", _params.axle_track, 1.0);
    _nh->param("/gear_ratio", _params.gear_ratio, 1.0);
    _nh->param("/wheel_radius", _params.wheel_radius, 1.0);
    
    // Subscribers Initialization
    _sub_left = _nh->subscribe(_params.topic_name_left_rpm, 1, &RobotClass::subLeft, this);
    _sub_right = _nh->subscribe(_params.topic_name_right_rpm, 1, &RobotClass::subRight, this);
    
    // Publishers Initialization
    _pub_odom = _nh->advertise<nav_msgs::Odometry>("/odom",1);
    _robot_pose = (const struct RobotPose){0};
    _prev_timestamp = ros::Time::now();

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

/*
nav_msgs/Odometry:
    std_msgs/Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
*/
void RobotClass::calculateOdom(){
    ros::Time cur_timestamp = ros::Time::now();
    double vl = _vel_m_s.left;
    double vr = _vel_m_s.right;
    double dt, vx, wz;
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;
    tf::TransformBroadcaster odom_broadcaster;


    // Cinematic equations for a differential traction robot on unicycle model representation
    dt = cur_timestamp.toSec() - _prev_timestamp.toSec();
    _prev_timestamp = cur_timestamp;
    vx = 0.5*(vr+vl);                                   // Eq. 1, vx = (vr+vl)/2
    wz = (vr-vl)/_params.axle_track;                    // Eq. 2, wz = (vr-vl)/L
    _robot_pose.theta += wz*dt;                         // Eq. 8, θ(t) = θ(t-1) + wz*dt
    _robot_pose.x += vx*std::cos(_robot_pose.theta);    // Eq. 6, x(t) = x(t-1) + vx*cosθ
    _robot_pose.y += vx*std::sin(_robot_pose.theta);    // Eq. 7, y(t) = y(t-1) + vx*sinθ

    //Set the quaternion using fixed axis RPY (Roll, Pitch, Yaw).
    odom_quat.setRPY(0, 0, _robot_pose.theta);  
    
    // geometry_msgs/Twist: This expresses velocity in free space broken into its linear and angular parts.
    odom.twist.twist.angular.z = wz;
    odom.twist.twist.linear.x = vx;
    
    // geometry_msgs/Pose: A representation of pose in free space, composed of position and orientation. 
    odom.pose.pose.position.x = _robot_pose.x;
    odom.pose.pose.position.y = _robot_pose.y;
    
    // geometry_msgs/Quaternion: This represents an orientation in free space in quaternion form.
    odom.pose.pose.orientation = tf2::toMsg(odom_quat); // tf2::Quaternion -> geometry_msgs::Quaternion
    
    // First, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = cur_timestamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = _robot_pose.x;
    odom_trans.transform.translation.y = _robot_pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf2::toMsg(odom_quat);
    // odom_trans.transform.rotation.x = odom_quat;
    // odom_trans.transform.rotation.y = odom_quat;
    // odom_trans.transform.rotation.z = odom_quat;


    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // Publish nav_msgs/Odometry message
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
        _pub_odom.publish(odom);
}