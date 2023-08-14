

#include "localization/msg/cloud_info.hpp"
#include "../include/localization/loc_tools.hpp"

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw)
                                  (double, time, time))





typedef PointXYZIRPYT  PointTypePose;


class Localization : public ParamNode
{
    public:   

    bool newData;


    rclcpp::Time timeInfoStamp;
    
    //Search params
    float historyKeyframeSearchRadius = 15.0;
    float historyKeyframeFitnessScore = 1;               
    int iterationNum = 100;
    int searchNum = 10;        
    double fitnessEpsilon = 1e-6;

    float searchRadius = 20.0;



    //Map cloud data stored from LIO-SAM and keyframed PCD
    pcl::PointCloud<PointType>::Ptr mapCloud;
    pcl::PointCloud<PointType>::Ptr currCloud;
    pcl::PointCloud<PointType>::Ptr currCloudDS;
    pcl::PointCloud<PointType>::Ptr neighbourClouds;
    std::vector<pcl::PointCloud<PointType>::Ptr> neighbourCloudKeyFrames;
    

    //Downsampling params
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    const float leafSize = 0.8;


    pcl::PointCloud<PointType>::Ptr fullCloud;
    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    std::deque<sensor_msgs::msg::Imu> imuQueue;


    nav_msgs::msg::Path correctedPath;
    nav_msgs::msg::Path ActualPath;

    //Key frame extraction parameters
    pcl::PointCloud<PointType>::Ptr trajectory;
    pcl::PointCloud<PointTypePose>::Ptr transformations;

    // nav_msgs::msg::Path globalPath;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subTrajectory;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloudInfo;

    

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubNeighbourClouds;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCurrCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCorrectedCloud;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubActualPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubCorrectedPath;
       
    
      
    Localization(const rclcpp::NodeOptions & options, std::string path) : 
                ParamNode("localization_node", options)
    {
        
        
        allocateMemory();

        //pre-registered keys (From LIO-SAM)
        getMap(path);
        getTrajectory(path);
        createCloudKeyFrames();

        downSizeFilterICP.setLeafSize(leafSize, leafSize, leafSize);

        std::cout<<"----------------> Starting extarctor"<<std::endl;
        
        // subCloudInfo = create_subscription<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", qos, std::bind(&Localization::getCurrCloudInfo, this, std::placeholders::_1));

        subTrajectory = create_subscription<nav_msgs::msg::Path>("localization/imu_path", qos, std::bind(&Localization::findNeighbourKeyFrames, this, std::placeholders::_1));
        
       

        pubNeighbourClouds = create_publisher<sensor_msgs::msg::PointCloud2>("localization/neighbour_cloud_data", 1);
        pubCurrCloud = create_publisher<sensor_msgs::msg::PointCloud2>("localization/curr_cloud_data", 1);
        pubCorrectedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("localization/corrected_cloud", 1);
        pubActualPath = create_publisher<nav_msgs::msg::Path>("localization/uncorrected_path", 1);
        pubCorrectedPath = create_publisher<nav_msgs::msg::Path>("localization/corrected_path", 1);



    
    }

    ~Localization(){}

    
    void allocateMemory()
    {
        mapCloud.reset(new pcl::PointCloud<PointType>());
        currCloud.reset(new pcl::PointCloud<PointType>());
        neighbourClouds.reset(new pcl::PointCloud<PointType>());
        trajectory.reset(new pcl::PointCloud<PointType>());
        transformations.reset(new  pcl::PointCloud<PointTypePose>());
        currCloudDS.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
    
    }

    void getMap(std::string path)
    {
        pcl::PCDReader reader;
        reader.read(path + std::string("GlobalMap.pcd"), *mapCloud);
        PCL_INFO( "------> Reading MAP Data \033[0m");
        std::cout<<std::endl;
        
        if(mapCloud->width * mapCloud->height > 0)
            PCL_INFO_STREAM( "\033[1;34m------> MAP Loaded Successfully. Size: "<<mapCloud->size()<< "\033[0m"<<std::endl);
     
        std::cout<<std::endl;
    }

    void getTrajectory(std::string path)
    {
        pcl::PCDReader reader;
        reader.read(path + std::string("trajectory.pcd"), *trajectory);
        PCL_INFO( "------> Reading TRAJECTORY Data \033[0m");
        std::cout<<std::endl;
        
        if(trajectory->width * trajectory->height > 0)
            PCL_INFO_STREAM( "\033[1;34m------>TRAJECTORY Loaded Successfully. Size: "<<trajectory->size()<< "\033[0m"<<std::endl);
        // else 
        // { 
        //     PCL_ERROR( "> TRAJECTORY Load Failed");
        //     rclcpp::shutdown();
        // }

        std::cout<<std::endl;
    }

    void createCloudKeyFrames()
    {
        int keyFrameCount = trajectory->size();
        PCL_INFO( "------> Creating Key Frames \033[0m");
        std::cout<<std::endl;
        for(int i = 0; i < keyFrameCount; ++i)
        {
            //Extract clouds and store as keyframes based on index
            pcl::PointCloud<PointType>::Ptr thisCloud(new pcl::PointCloud<PointType>);

            for(size_t j = 0; j < mapCloud->size(); ++j)
            {
                // std::cout<<"Comparing - "<<static_cast<int>(mapCloud->points[j].intensity)<<" == "<<i<<std::endl;
                if(static_cast<int>(mapCloud->points[j].intensity) == i)
                    thisCloud->push_back(mapCloud->points[j]);
            }

            neighbourCloudKeyFrames.push_back(thisCloud);

        }
        if(neighbourCloudKeyFrames.size() > 0)
           PCL_INFO_STREAM( "\033[1;34m------> Key Frames Created Successfully. Size: "<<neighbourCloudKeyFrames.size()<< "\033[0m"<<std::endl);
    
        std::cout<<std::endl;
    }






    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    void getCurrCloudInfo(const sensor_msgs::msg::PointCloud2::SharedPtr msgCloud)
    {
      
        pcl::fromROSMsg(*msgCloud, *currCloud);
        timeInfoStamp = msgCloud->header.stamp;
        newData = true;


        // std::cout<<"Current Cloud size: "<<currCloud->size()<<std::endl;
    

    }
    
    void findNeighbourKeyFrames(const nav_msgs::msg::Path::SharedPtr msg)
    {

        PointTypePose pt6D;
        PointType pt3D;

        
        auto getPt = msg->poses.back();

        timeInfoStamp = msg->header.stamp;
        
        tf2::Quaternion q(getPt.pose.orientation.x, getPt.pose.orientation.y, getPt.pose.orientation.z, getPt.pose.orientation.w);

        double roll, pitch, yaw;

        tf2::Matrix3x3 mat(q);
        mat.getRPY(roll,pitch,yaw);

        pt6D.x =  getPt.pose.position.x;
        pt6D.y =  getPt.pose.position.y;
        pt6D.z =  getPt.pose.position.z;
        pt6D.roll = roll;
        pt6D.pitch = pitch;
        pt6D.yaw = yaw;

        pt3D.x = pt6D.x;
        pt3D.y = pt6D.y;
        pt3D.z = pt6D.z;

        // extract near keyframes
        nav_msgs::msg::Path keyFramePath;
        geometry_msgs::msg::PoseStamped Poses;

        

        

        Poses.header.stamp = timeInfoStamp;
        Poses.header.frame_id = odometryFrame;
        Poses.pose.position.x = pt3D.x;
        Poses.pose.position.y = pt3D.y;
        Poses.pose.position.z = pt3D.z;
        tf2::Quaternion qt;
        qt.setRPY(roll, pitch, yaw);
        Poses.pose.orientation.x = qt.x();
        Poses.pose.orientation.y = qt.y();
        Poses.pose.orientation.z = qt.z();
        Poses.pose.orientation.w = qt.w();
        
        ActualPath.header.stamp = timeInfoStamp;
        ActualPath.header.frame_id = odometryFrame;
        ActualPath.poses.push_back(Poses);

        
        pcl::KdTreeFLANN<PointType> kdtree;
        std::vector<int> kIndices;
        std::vector<float> kSquaredDistances;
        

        neighbourClouds->clear();

        kdtree.setInputCloud(trajectory);
        kdtree.nearestKSearch(pt3D, searchNum, kIndices, kSquaredDistances);

        for(int i = 0; i < searchNum; ++i)
        {
            int index = kIndices[i];
            *neighbourClouds += *neighbourCloudKeyFrames[index];
        }

        /* for radius search
        //create suubset from mapcloud
        kdtree.setInputCloud(mapCloud);
        kdtree.radiusSearch(pt3D , searchRadius, kIndices, kSquaredDistances);
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(mapCloud);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        inliers->indices = kIndices;
        extract.setIndices(inliers);
        extract.filter(*neighbourClouds);
        
        kIndices.clear();
        kSquaredDistances.clear();
        inliers.reset(new pcl::PointIndices);

        //create subset from current cloud

        kdtree.setInputCloud(currCloud);
        kdtree.radiusSearch(pt3D , searchRadius, kIndices, kSquaredDistances);
        extract.setInputCloud(currCloud);
        inliers->indices = kIndices;
        extract.setIndices(inliers);
        extract.filter(*currCloud);

        
        pubNeighbourTrajectory->publish(keyFramePath);
        downSizeFilterICP.setInputCloud(currCloud);
        downSizeFilterICP.filter(*currCloudDS);*/

        pubActualPath->publish(ActualPath);

        publishCloud(pubNeighbourClouds, neighbourClouds, timeInfoStamp, odometryFrame);
       
        loopClosure(pt6D);
    
    }

    void loopClosure(PointTypePose pose6D)
    {
        if(!newData)
            return;
        
        newData = false;

        
        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;



        std::cout<<"----------------> Performing LoopClosure with: "<<currCloud->size()<< " target: "<< neighbourClouds->size()<<std::endl;


        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(iterationNum);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(fitnessEpsilon);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(currCloud);
        icp.setInputTarget(neighbourClouds);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        {
            std::cout<<"Unable to converge  or not fit enough"<<std::endl;
            return;

        }   
        // publish corrected cloud
        if (pubCorrectedCloud->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*currCloud, *closed_cloud, icp.getFinalTransformation());
            
            publishCloud(pubCorrectedCloud, closed_cloud, timeInfoStamp, odometryFrame);
        }
        
        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        geometry_msgs::msg::PoseStamped corrPoseStamped;

        Eigen::Affine3f TransMatrix;
        TransMatrix = icp.getFinalTransformation();

        // transform pcl to affine of the uncorrected pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(pose6D);

        // transform corrected pose
        Eigen::Affine3f tCorrect = TransMatrix * tWrong;// pre-multiplying -> successive rotation about a fixed frame

        //Find euler params for transformations for the coorrected
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

        corrPoseStamped.header.stamp = timeInfoStamp;
        corrPoseStamped.header.frame_id = odometryFrame;
        corrPoseStamped.pose.position.x = x;
        corrPoseStamped.pose.position.y = y;
        corrPoseStamped.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        corrPoseStamped.pose.orientation.x = q.x();
        corrPoseStamped.pose.orientation.y = q.y();
        corrPoseStamped.pose.orientation.z = q.z();
        corrPoseStamped.pose.orientation.w = q.w();


        correctedPath.header.stamp = timeInfoStamp;
        correctedPath.header.frame_id = odometryFrame;
        correctedPath.poses.push_back(corrPoseStamped);

        std::cout<<"-----------> publishing corrected path"<<std::endl;
        pubCorrectedPath->publish(correctedPath);
    }
  

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;

  rclcpp::executors::MultiThreadedExecutor exec;

  std::string path = "/home/kenobi/internship/map/rooftop/";

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m>>>Initializing Localization Tools... \033[0m");

  
  auto CD = std::make_shared<Localization>(options, path);
  
  exec.add_node(CD);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}