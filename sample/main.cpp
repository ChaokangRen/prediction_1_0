#include <ros/ros.h>
#include <sglog/sglog.h>

#include "prediction/prediction_interface.h"
#include "prediction_component.h"
#include "sg_perception_msgs/TrackedObjects.h"
using namespace jarvis::prediction_lib;

ObstaclesInfo obstacles_info;
std::string debug_info;

std::shared_ptr<jarvis::prediction_lib::PredictionInterface>
    prediction_interface_ptr;

void PerceptionCallback(
    const sg_perception_msgs::TrackedObjects::ConstPtr &tracked_objects_ptr) {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prediction_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Rate loop_rate(10);
    ros::Subscriber perception_sub =
        nh.subscribe("/perception/lidar/tracked", 10, PerceptionCallback);

    SG_INFO("version: %s", prediction_interface_ptr->get_version());
    while (ros::ok()) {
        prediction_interface_ptr->execute(&obstacles_info, debug_info);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}