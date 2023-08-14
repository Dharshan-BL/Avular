#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <rclcpp/rclcpp.hpp>


#include<iostream>


typedef pcl::PointXYZI PointType;


class Voxelizer : public rclcpp::Node
{
    
    private: 
        pcl::PointCloud<PointType>::Ptr cloud;
        pcl::PCLPointCloud2::Ptr voxel_cloud;
        std::string path = "/home/kenobi/internship/map/rooftop/";
            
    public:

        Voxelizer() : Node("point_cloud_voxelize")
        {
            //init cloud and normal var.
            
            allocate_memory();

            read_cloud_data();

            create_voxel_grid();
            
            write_cloud_data();
        }

        ~Voxelizer(){}


        void allocate_memory()
        {
            cloud.reset(new pcl::PointCloud<PointType>);
            voxel_cloud.reset(new pcl::PCLPointCloud2);


        }

        void read_cloud_data()
        {
            //Read PCD data from file
            pcl::PCDReader reader;
            reader.read(path + std::string("GlobalMap.pcd"), *cloud);
            PCL_INFO( "\033[1;33m----> PCL Read Cloud Data \033[0m");
            std::cout<<std::endl;
            
            if(cloud->width * cloud->height > 0)
                PCL_INFO( "\033[1;32m----> PCL Read Success \033[0m");
            else 
                PCL_WARN( "----> PCL No Data Extracted");

            std::cout<<std::endl;
        }


        void create_voxel_grid()
        {
            pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);

            PCL_INFO("\033[1;33m----> PCL Creating Voxel Grid \033[0m");
            std::cout<<std::endl;

            pcl::toPCLPointCloud2(*cloud, *tempCloud);
            
            pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
            
            float leafSize = 0.8;
            voxel_filter.setInputCloud(tempCloud);
            voxel_filter.setLeafSize(0.2, 0.2, 0.2);
            voxel_filter.filter(*voxel_cloud);

            if(voxel_cloud->width * voxel_cloud->height > 0)
                PCL_INFO( "\033[1;32m----> PCL Voxel Grids Creation Success \033[0m");
            else 
                PCL_WARN( "----> PCL Voxel Grid Creation Failed");
            
            
            std::cout<<std::endl;

        }

        void write_cloud_data()
        {
            PCL_INFO( "\033[1;33m----> PCL Writing Voxel Grids \033[0m");
            std::cout<<std::endl;

            pcl::PCDWriter writer;
            writer.write(path + std::string("voxelized.pcd"), voxel_cloud);
           
            PCL_INFO( "\033[1;32m----> PCL Writing success \033[0m"); 
            std::cout<<std::endl;
        }

        

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Creating Voxel Grids \033[0m");

  auto CD = std::make_shared<Voxelizer>();
  
  rclcpp::spin(CD);

  rclcpp::shutdown();

  return 0;
}
