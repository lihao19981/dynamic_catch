
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h> 
#include <pcl/point_types.h>
#include "ros/time.h"
#include <tf/tf.h>
#include <sstream>    

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>
#include <stdlib.h>

#include <pcl/console/time.h>
#include <stdlib.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Twist.h>

#include "ndt2.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int pos=0;
int ros_pos = 0;
int b=0;
ros::Time time1;
std::string time_stamp ="0";
bool if_target = 1;
bool if_input = 1;

float x_ = 0;
float y_ = 0;
float theta_ = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr input_load_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr outvioxl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

float out_pos_x ;
float out_pos_y ;

template <typename T> std::string to_string(T value)
{
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}

void outvioxl()  //获得目标点云的质心
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("target.pcd", *target_cloud) == -1)
    		{
       			PCL_ERROR("Couldn't read file target.pcd \n");
        		
    		}
	std::cout << "Loaded " << target_cloud->width
		<< "data points from target_pcd.pcd with the following fields:"
		<< std::endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *input_load_cloud) == -1)
    		{
       			PCL_ERROR("Couldn't read file input.pcd \n");
        		
    		}
	std::cout << "Loaded " << input_cloud->width
		<< "data points from input_pcd.pcd with the following fields:"
		<< std::endl;

	
    	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    	approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
    	approximate_voxel_filter.setInputCloud (input_load_cloud);
    	approximate_voxel_filter.filter (*input_cloud);	

	//初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.0001);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(0.5);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(500);
    // 设置要配准的点云
    ndt.setInputSource(input_cloud);
    //设置点云配准目标
    ndt.setInputTarget(target_cloud);

	
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    ndt.align(*output_cloud);
	cout<<"ndt "<<endl;

    vector<int> out_vioxl_points;
    out_vioxl_points=ndt.get_outpoints();

    cout<<"it:"<<ndt.getFinalNumIteration()<<endl;
    cout<<"out_vioxl:"<<out_vioxl_points.size()<<endl;


    
	
	
    //////////
    outvioxl_cloud->width =out_vioxl_points.size();
    outvioxl_cloud->height=1;
    outvioxl_cloud->is_dense = false;
    outvioxl_cloud->points.resize(outvioxl_cloud->width * outvioxl_cloud->height);
    

    cout << "x , y = " << out_pos_x << " , " << out_pos_y<< endl;
    pcl::transformPointCloud(*outvioxl_cloud, *outvioxl_cloud, ndt.getFinalTransformation());
    /*______________________________________________________*/
    vector<int> if_PointsDynamic(input_cloud->points.size(),0);
    for(int i=0;i<out_vioxl_points.size();i++)
    {
        if_PointsDynamic[out_vioxl_points[i]]=1;
    }
    // -------------------------------------------欧式聚类--------------------------------------------
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input_cloud);
    vector<pcl::PointIndices> cluster_indices; // 聚类索引
    pcl::EuclideanClusterExtraction<PointT> ec;// 欧式聚类对象
    ec.setClusterTolerance(0.3);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
    ec.setMinClusterSize(4);                 // 设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(25000);               // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);                  // 设置点云的搜索机制
    ec.setInputCloud(input_cloud);                   // 设置输入点云
    ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中

    int chose_cluster = 0;
    for(int i = 0;i<cluster_indices.size();i++)
    {
	if(cluster_indices[i].indices.size() > out_vioxl_points.size())
	{
		continue;
	}
	chose_cluster = i;

    }
	cout<<"chose_cluster size: "<<cluster_indices[chose_cluster].indices.size()<<endl;
    vector<int> if_CloudsDynamic(cluster_indices.size(),0);
    for(int i=0;i<cluster_indices.size();i++)
    {
        int cnt=0;
        for(int j=0;j<cluster_indices[i].indices.size();j++)
        {
            if (if_PointsDynamic[cluster_indices[i].indices[j]]==1)
            {
                //cnt++;
                if_CloudsDynamic[i]++;

            }
            if (if_CloudsDynamic[i] /cluster_indices[i].indices.size() >0.25)
            {
                if_CloudsDynamic[i] = 2000;
                break;
            }
            /*if(cnt ==50)
            {
                if_CloudsDynamic[i]=1;
                break;
            }*/
        }
    }
    /* get static point cloud*/
    
    out_pos_x = 0;  // 目标质点
    out_pos_y = 0;
    int out_len = 0 ;
    vector<int> if_point_static(input_cloud->points.size(), 1);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_Cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    
    for (int i=0;i<cluster_indices.size();i++)
    {

        	for(int j=0;j<cluster_indices[i].indices.size();j++)
            	{
        		out_pos_x += input_cloud->points[cluster_indices[i].indices[j]].x; 
    			out_pos_y += input_cloud->points[cluster_indices[i].indices[j]].y;
    			out_len +=1;
                	if_point_static[cluster_indices[i].indices[j]] = 0;
					
			break;
    		}
	
    }
    	out_pos_x = -out_pos_x/out_len;
    	out_pos_y = -out_pos_y/out_len;

    static_Cluster_cloud->width =input_cloud->points.size() - out_len;
    static_Cluster_cloud->height=1;
    static_Cluster_cloud->is_dense = false;
        static_Cluster_cloud->points.resize(input_cloud->points.size() - out_len);
        int cnt_=0;
        for (int i = 0; i < input_cloud->points.size(); i++)
        {
            if (if_point_static[i] == 1)
            {
                static_Cluster_cloud->points[cnt_] = input_cloud->points[i];
 		cnt_ ++;
            }
        }
	pcl::io::savePCDFileASCII ("target.pcd", *static_Cluster_cloud);
/*

    out_pos_x = 0;  // 目标质点
    out_pos_y = 0;
    
    for (int i=0;i<cluster_indices[chose_cluster].indices.size();i++)
    {
 
        out_pos_x += input_cloud->points[cluster_indices[chose_cluster].indices[i]].x;
        out_pos_y += input_cloud->points[cluster_indices[chose_cluster].indices[i]].y;
    }
    out_pos_x = out_pos_x / cluster_indices[chose_cluster].indices.size();
    out_pos_y = out_pos_y / cluster_indices[chose_cluster].indices.size();

    cout << "x , y = " << out_pos_x << " , " << out_pos_y<< endl;
	*/
    /*
    vector<int> if_CloudsDynamic(cluster_indices.size(),0);
    for(int i=0;i<cluster_indices.size();i++)
    {
        int cnt=0;
        for(int j=0;j<cluster_indices[i].indices.size();j++)
        {
            if (if_PointsDynamic[cluster_indices[i].indices[j]]==1)
            {
                //cnt++;
                if_CloudsDynamic[i]++;

            }
            if (if_CloudsDynamic[i] /cluster_indices[i].indices.size() >0.25)
            {
                if_CloudsDynamic[i] = 2000;
                break;
            }
            if(cnt ==50)
            {
                if_CloudsDynamic[i]=1;
                break;
            }
        }
    }*/


    
}

void callback(const  sensor_msgs::PointCloud &msg){
	/*long int pcd_time =msg.header.stamp.sec ;
	time_stamp=to_string(pcd_time);
	time_stamp += "_";

	long int image_time2= msg.header.stamp.nsec/100000 ;
	if(image_time2<1000)
	{
		time_stamp+="0";
	}
	std::string pcd_time_2=to_string(pcd_time2);
	time_stamp += image_time_2;
	*/

	sensor_msgs::PointCloud2 laserCloudMsg;
        convertPointCloudToPointCloud2(msg, laserCloudMsg);
  
	pcl::PointCloud<pcl::PointXYZ> cloud;
  	pcl::fromROSMsg(laserCloudMsg, cloud);//从ROS类型消息转为PCL类型消息
	
	if(pos == 0)
	{
		pcl::io::savePCDFileASCII ("target.pcd", cloud);
    		
		std::cout<<"target_cloud : saved"<<endl;
		pos=pos+1;
	}
	if(pos != 0)
	{
		pcl::io::savePCDFileASCII ("input.pcd", cloud);
    		
		std::cout<<"input_cloud : saved"<<endl;
		pos=pos+1;
	}
	/*
	std::string file_name = "./pcd/";
	file_name += time_stamp;
  	file_name += ".pcd";
  	pcl::io::savePCDFileASCII (file_name, cloud);//保存pcd
	*/
	
	//std::cout<<"save pcd:"<<file_name<<endl;
}
void callback2(const  sensor_msgs::PointCloud2 &laserCloudMsg){
	/*long int pcd_time =msg.header.stamp.sec ;
	time_stamp=to_string(pcd_time);
	time_stamp += "_";

	long int image_time2= msg.header.stamp.nsec/100000 ;
	if(image_time2<1000)
	{
		time_stamp+="0";
	}
	std::string pcd_time_2=to_string(pcd_time2);
	time_stamp += image_time_2;
	*/

  
	pcl::PointCloud<pcl::PointXYZ> cloud;
  	pcl::fromROSMsg(laserCloudMsg, cloud);//从ROS类型消息转为PCL类型消息
	
	if(pos == 0)
	{
		pcl::io::savePCDFileASCII ("target.pcd", cloud);
    		
		std::cout<<"target_cloud : saved"<<endl;
		pos=pos+1;
	}
	if(pos != 0)
	{
		pcl::io::savePCDFileASCII ("input.pcd", cloud);
    		
		std::cout<<"input_cloud : saved"<<endl;
		pos=pos+1;
	}
	/*
	std::string file_name = "./pcd/";
	file_name += time_stamp;
  	file_name += ".pcd";
  	pcl::io::savePCDFileASCII (file_name, cloud);//保存pcd
	*/
	
	//std::cout<<"save pcd:"<<file_name<<endl;
}


void callback3(const geometry_msgs::PoseStamped &msg){
	x_ = msg.pose.position.x;
	y_ = msg.pose.position.y;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
	theta_ = yaw;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "target_pos_get");
  	ros::NodeHandle n;
        ros::Subscriber sub_a = n.subscribe("/slam_cloud", 1, callback);
	ros::Subscriber sub_b = n.subscribe("/scan_matched_points2", 1, callback2);
	ros::Subscriber sub_c = n.subscribe("/slam_out_pose",1,callback3);
	ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Twist>("/target_pos_x_y", 1);
	
	
    	
	
	
	ros::Rate loop_rate(5);	
while (ros::ok())
{
	ros::spinOnce();
	if((!if_target)& if_input)
	{
		std::cout<<"step 2? "<<pos<<endl;
		std::cin>>b;
		if_input = 0;
	}

	if(if_target)
	{
		std::cout<<"step 1? "<<pos<<endl;
		std::cin>>b;
		if_target = 0;
	}
	
	
	if(pos >2)
	{
		cout<<"start:"<<endl;
		outvioxl();
		geometry_msgs::Twist pos_msg;  //发布位置
		
		pos_msg.linear.x = out_pos_x*cos(theta_) +  out_pos_y*sin(theta_) + x_;
		pos_msg.linear.y = out_pos_y*cos(theta_) -  out_pos_x*sin(theta_) + y_;
		
		/*
		pos_msg.linear.x = out_pos_x;
		pos_msg.linear.y = out_pos_y;
		*/
		pos_msg.linear.z = ros_pos; 
		ros_pos += 1;
		target_pos_pub.publish(pos_msg);
	}

}
    
    return 0;
}

