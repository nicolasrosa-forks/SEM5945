/* ============ */
/*  Libraries   */
/* ============ */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

/* ============ */
/*  Functions   */
/* ============ */
/**
 * @brief Subscribler CallBack Function
 * @param std_msgs/Float32 messages from "/left_rpm" topic
 * std_msgs/Float32:
 *  float32 data
 */
void subCb(const std_msgs::Float32::ConstPtr &msg){
    ROS_INFO_STREAM(msg->data);
}

/* ======= */
/*  Main   */
/* ======= */
int main(int argc, char **argv){
    // ROS Node Initialization
    ros::init(argc, argv, "sub_rpm_node");
    ros::NodeHandle nh;

    // Topic Subscription
    std::string topic_name = "/left_rpm";
    int queue_size = 10;
    ros::Subscriber sub_rpm = nh.subscribe(topic_name, queue_size, subCb);

    ros::spin();
}