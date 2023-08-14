#include "../include/localization/loc_tools.hpp"
#include "localization/msg/cloud_info.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <robot_localization/ros_filter.hpp>
#include <robot_localization/ros_filter_types.hpp>
#include <robot_localization/ros_filter_utilities.hpp>
#include <robot_localization/filter_utilities.hpp>
#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_base.hpp>
#include <robot_localization/ekf.hpp>
#include <robot_localization/ukf.hpp>


struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)


// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;


class ImuIntegration : public ParamNode
{
private:
    //World stuff
    Eigen::Vector3d gravityWorld;
    
    //TF2 stuff
    std::shared_ptr<tf2_ros::TransformBroadcaster> linkFrames;
    
    //Cloud points related stuff
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointCloudRaw;
    pcl::PointCloud<PointXYZIRT>::Ptr cloudIn;
    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudProcessed;

    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    std_msgs::msg::Header cloudHeader;

    //IMU related stuff
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;

    rclcpp::Time timeInfoStamp;

    //Path related stuff
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    nav_msgs::msg::Path globalPath;
    

    Eigen::Vector3d estimated_position_;
    // Define a global rotation matrix initialized as identity
    Eigen::Matrix3d global_rotation_matrix = Eigen::Matrix3d::Identity();

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));

    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::NavState prevStateOdom;

    double last_imu_time_;
    bool is_initialized_;


    float totalTime = 0.0;

    double estimated_roll_, estimated_pitch_, estimated_yaw_;

    Eigen::Vector3d velWorld;

    int count = 0;

    
    
public: 

    ImuIntegration(const rclcpp::NodeOptions &options) : ParamNode("dead_reckoning", options)
    {   

        is_initialized_ = false;

        allocateMemory();
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/robot/imu/data", qos_imu, std::bind(&ImuIntegration::imuHandlerBASIC, this, std::placeholders::_1));

        // subPointCloudRaw = create_subscription<sensor_msgs::msg::PointCloud2>(
            // pointCloudTopic, qos_lidar, std::bind(&ImuIntegration::cloudHandler, this, std::placeholders::_1));

        pubCloudProcessed = create_publisher<sensor_msgs::msg::PointCloud2>("localization/cloud_points", 1);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("/localization/imu_path", 10);


        linkFrames = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread

    }

    void allocateMemory()
    {
        // setEKFParams();
        velWorld << 0.0 , 0.0 ,0.0;

        gravityWorld << 0.0, 0.0, -imuGravity;

        cloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        cloudIn->clear();
        
        // // Initialize the EKF instance
        // ekf_.initialize(ekf_params);

    }


    void initializeOrientation(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
    
        // Initialize position based on initial linear acceleration (assuming static start)
        estimated_position_ = Eigen::Vector3d(
            0.0,
            0.0,
            0.0);
    }


    void imuHandlerGTSAM(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        timeInfoStamp = imu_msg->header.stamp;
        
        double current_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
        
        if (!is_initialized_)   
        {
            initializeOrientation(imu_msg);
            last_imu_time_ = current_time;
            // Initialize prevStateOdom assuming the robot starts from a stationary position
            prevStateOdom = gtsam::NavState(gtsam::Pose3::identity(), gtsam::Vector3::Zero());

            is_initialized_ = true;
            return;
        }


        double dt =  current_time - last_imu_time_;
        
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
            gtsam::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
            gtsam::Vector3(imu_msg->angular_velocity.x,    imu_msg->angular_velocity.y,    imu_msg->angular_velocity.z), dt);


        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prior_imu_bias);

        // Update the previous state for the next iteration
        prevStateOdom = currentState;

        // std::cout<<dt<<std::endl;
        // Print prevStateOdom components
        // std::cout << "Prev State Odom: " << prevStateOdom.pose() << std::endl;
        // std::cout << "Prev State Velocity: " << prevStateOdom.velocity() << std::endl;


        // publish odometry
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = imu_msg->header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";


        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = imu_msg->angular_velocity.x;//+ prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = imu_msg->angular_velocity.y;// + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = imu_msg->angular_velocity.z;// + prevBiasOdom.gyroscope().z();


 
        globalPath.header.stamp = odometry.header.stamp;
        globalPath.header.frame_id = odometry.header.frame_id;

        // Create a PoseStamped message for each odometry update
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = odometry.header;
        pose_msg.pose = odometry.pose.pose;
        // Add the PoseStamped message to the Path
        globalPath.header.stamp = timeInfoStamp;
        globalPath.header.frame_id = odometryFrame;
        globalPath.poses.push_back(pose_msg);

        // Now you can publish the Path
        pubPath->publish(globalPath);

        last_imu_time_ = current_time;
    }

    void imuHandlerBASIC(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        timeInfoStamp = imu_msg->header.stamp;
        
        
        double current_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
        
        if (!is_initialized_)   
        {
            initializeOrientation(imu_msg);
            last_imu_time_ = current_time;
            // Initialize prevStateOdom assuming the robot starts from a stationary position
            prevStateOdom = gtsam::NavState(gtsam::Pose3::identity(), gtsam::Vector3::Zero());


            // Extract roll, pitch, and yaw from the initial quaternion
            tf2::Quaternion initial_quaternion;
            tf2::fromMsg(imu_msg->orientation, initial_quaternion);
            tf2::Matrix3x3(initial_quaternion).getRPY(estimated_roll_, estimated_pitch_, estimated_yaw_);

            is_initialized_ = true;


            // debug IMU data
            // cout << std::setprecision(6);
            // cout << "IMU acc: " << endl;
            // cout << "x: " << imu_msg->linear_acceleration.x << 
            //     ", y: " << imu_msg->linear_acceleration.y << 
            //     ", z: " << imu_msg->linear_acceleration.z << endl;
            // cout << "IMU gyro: " << endl;
            // cout << "x: " << imu_msg->angular_velocity.x << 
            //     ", y: " << imu_msg->angular_velocity.y << 
            //     ", z: " << imu_msg->angular_velocity.z << endl;

            // cout << "IMU roll pitch yaw: " << endl;
            // cout << "roll: " << estimated_roll_ << ", pitch: " << estimated_pitch_ << ", yaw: " << estimated_yaw_ << endl << endl;

            return;
        }


        double dt =  current_time - last_imu_time_;
        
        totalTime += dt;
        
        double acc_thresh = 0.1, vel_thresh = 0.1;

        // For simplicity, let's assume the IMU provides angular velocity in rad/s
        double angular_velocity_x = imu_msg->angular_velocity.x;
        double angular_velocity_y = imu_msg->angular_velocity.y;
        double angular_velocity_z = imu_msg->angular_velocity.z;


        double linear_acceleration_x = imu_msg->linear_acceleration.x;
        double linear_acceleration_y = imu_msg->linear_acceleration.y;
        double linear_acceleration_z = imu_msg->linear_acceleration.z;

        // Integrate angular velocity to estimate orientation change
        double theta_x = angular_velocity_x * dt;
        double theta_y = angular_velocity_y * dt;
        double theta_z = angular_velocity_z * dt;



        //   debug IMU data
        cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << imu_msg->linear_acceleration.x << 
        //       ", y: " << imu_msg->linear_acceleration.y << 
        //       ", z: " << imu_msg->linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << imu_msg->angular_velocity.x << 
        //       ", y: " << imu_msg->angular_velocity.y << 
        //       ", z: " << imu_msg->angular_velocity.z << endl;
        
        double imuRoll, imuPitch, imuYaw;
        tf2::Quaternion orientation;
        tf2::convert(imu_msg->orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
        // cout << "estroll: " << theta_x << ", estpitch: " << theta_y << ", estyaw: " << theta_z << endl << endl;

        if(totalTime > stopTimer)
            rclcpp::shutdown();
        // Update robot pose using the estimated orientation change
        // theta_x = imuRoll;
        // theta_y = imuPitch ;
        // theta_z =  imuYaw ;

        // Convert RPY angles to a rotation matrix
        Eigen::Matrix3d Rx, Ry, Rz, rotation_matrix;

        Rx << 1, 0, 0,
              0, cos(theta_x), sin(theta_x),
              0, -sin(theta_x), cos(theta_x);
        

        Ry << cos(theta_y), 0, -sin(theta_y),
              0, 1, 0,
              sin(theta_y), 0, cos(theta_y);

        
        Rz << cos(theta_z), sin(theta_z), 0,
              -sin(theta_z), cos(theta_z), 0,
              0, 0, 1;
                  
        rotation_matrix = Rz * Ry * Rx;
        // rotation_matrix = Rx * Ry * Rz;

        
        Eigen::Vector3d gravityBody = global_rotation_matrix.transpose() * gravityWorld;
        Eigen::Vector3d accBody, accWorld;
  
        accBody << linear_acceleration_x , linear_acceleration_y, linear_acceleration_z;
        
        accBody += gravityBody;
        //Convert BOdy to world frame
        accWorld = global_rotation_matrix.transpose() * accBody;
        

        if(abs(accWorld[0]) < acc_thresh)
            accWorld[0] = 0;
        if(abs(accWorld[1]) < acc_thresh)
            accWorld[1] = 0;
        if(abs(accWorld[2]) < acc_thresh)
            accWorld[2] = 0;
        
        
        if(abs(angular_velocity_x) < vel_thresh) 
            angular_velocity_x = 0;

        if(abs(angular_velocity_y) < vel_thresh) 
            angular_velocity_y = 0;
        
        if(abs(angular_velocity_z) < vel_thresh) 
            angular_velocity_z = 0;
        
        Eigen::Vector3d position_change = 0.5 * accWorld * dt * dt + velWorld * dt; 

        velWorld += accWorld * dt;    

        std::cout<<"Acc. in world: "<< std::endl;
        std::cout<<"x: "<<accWorld[0]<<" y: "<<accWorld[1]<<" z: "<<accWorld[2]<<std::endl;
        std::cout<<"Vel. in world: "<< std::endl;
        std::cout<<"x: "<<velWorld[0]<<" y: "<<velWorld[1]<<" z: "<<velWorld[2]<<std::endl;
        std::cout<<std::endl<<std::endl;
       

        // Update initial velocity based on acceleration

        
        estimated_position_ += position_change;

        global_rotation_matrix = rotation_matrix * global_rotation_matrix;

       

        // Create a PoseStamped message to publish estimated pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = imu_msg->header.stamp;
        pose_msg.header.frame_id = odometryFrame; // Set the frame to map
        pose_msg.pose.position.x = estimated_position_.x();
        pose_msg.pose.position.y = estimated_position_.y();
        pose_msg.pose.position.z = estimated_position_.z();
        tf2::Quaternion q;
        q.setRPY(theta_x, theta_y, theta_z);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();


        globalPath.poses.push_back(pose_msg);
        // Now you can use transformedPosition and transformedOrientation for your fixed world frame


        // Publish the estimated path
        globalPath.header.stamp = timeInfoStamp;
        globalPath.header.frame_id = odometryFrame;

       
        last_imu_time_ = current_time;


        pubPath->publish(globalPath);
    }


/*
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg)
    {
        
       if (!cachePointCloud(cloudMsg))
            return;


        projectPointCloud();

        
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloudMsg)
    {   

        cloudHeader = cloudMsg->header;

        cloudQueue.push_back(*cloudMsg);
                
        if(cloudQueue.size() <= 2)
            return false;
        
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();


        pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn(new pcl::PointCloud<OusterPointXYZIRT>());

        pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
        
        cloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        cloudIn->points.resize(tmpOusterCloudIn->size());
        cloudIn->is_dense = tmpOusterCloudIn->is_dense;



        for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
        {
            auto &src = tmpOusterCloudIn->points[i];
            auto &dst = cloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;
        }


        if (cloudIn->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        return true;

    }

    void projectPointCloud()
    {
        int cloudSize = cloudIn->points.size();

         PointType thisPoint;
        thisPoint.x = cloudIn->points[i].x;
        thisPoint.y = cloudIn->points[i].y;
        thisPoint.z = cloudIn->points[i].z;
        thisPoint.intensity = cloudIn->points[i].intensity;

        float range = pointDistance(thisPoint);
        if (range < lidarMinRange || range > lidarMaxRange)
            continue;
  
    
        int columnIdn = -1;    

        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

    }
*/

    ~ImuIntegration(){}



   
};




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImuIntegration>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Integration Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}



