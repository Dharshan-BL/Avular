

#include "../include/localization/loc_tools.hpp"


class PathEstimator : public ParamNode  
{

private: 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcastTF;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;
    nav_msgs::msg::Path imuPath;

    std::string rotatedOdomFrame = "rotated_odom";

    rclcpp::Time timeInfoStamp;

public:
    PathEstimator(const rclcpp::NodeOptions &options) : ParamNode("pathExtraction", options)
    {
        broadcastTF = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        subOdom = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind( &PathEstimator::odomHandler, this, std::placeholders::_1));
        
        pubImuPath = this->create_publisher<nav_msgs::msg::Path>("/localization/imu_path", 10);



    }
    ~PathEstimator(){}



    void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msgIn)
    {
        geometry_msgs::msg::TransformStamped newTF;

        // Set the header
        newTF.header.stamp = timeInfoStamp;
        newTF.header.frame_id = odometryFrame;
        newTF.child_frame_id = rotatedOdomFrame;

        // Set the translation
        newTF.transform.translation.x = 0.0;
        newTF.transform.translation.y = 0.0;
        newTF.transform.translation.z = 0.0;

        // Create a quaternion for 90 degrees CW rotation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, -M_PI / 2); // 90 degrees in radians

        // Set the rotation
        newTF.transform.rotation.x = q.x();
        newTF.transform.rotation.y = q.y();
        newTF.transform.rotation.z = q.z();
        newTF.transform.rotation.w = q.w();

        // Publish the transform
        broadcastTF->sendTransform(newTF);
        
        
        
        geometry_msgs::msg::PoseStamped pose3D;

        timeInfoStamp = msgIn->header.stamp;

        pose3D.header.stamp = timeInfoStamp;
        pose3D.header.frame_id = odometryFrame;
        
        pose3D.pose.position.x = msgIn->pose.pose.position.x;
        pose3D.pose.position.y = msgIn->pose.pose.position.y;
        pose3D.pose.position.z = msgIn->pose.pose.position.z;
        pose3D.pose.orientation.w = msgIn->pose.pose.orientation.w;
        pose3D.pose.orientation.x = msgIn->pose.pose.orientation.x;
        pose3D.pose.orientation.y = msgIn->pose.pose.orientation.y;
        pose3D.pose.orientation.z = msgIn->pose.pose.orientation.z;

        imuPath.header.stamp = msgIn->header.stamp;
        imuPath.header.frame_id = odometryFrame;
        imuPath.poses.push_back(pose3D);

        pubImuPath->publish(imuPath);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto PE = std::make_shared<PathEstimator>(options);
    exec.add_node(PE);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Path Extraction Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
