//C++
#include <ctime>
#include <iostream>
#include <Eigen/Core>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>

#include <sos_fusion/MinMax.h>
#include <sos_fusion/PointArray.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


#define _USE_MATH_DEFINES
using namespace std;

#pragma region parameter

typedef pcl::PointXYZ PointT;

enum E_MODE
{
	M_ESTOP = -1,
	M_JOYSTICK = 0,
	M_SELF_DRIVING,
	M_SELF_DRIVING_BACK,
	M_STACK_RESET
};


//----------------------    PARAMETER   -------------------------------------
std::string frame_id_;

float x_axis_min_ = -5;
float x_axis_max_ = 0;

float y_axis_min_ = -4;
float y_axis_max_ = 4;

float z_axis_min_ = 0;
float z_axis_max_ = 0;
float z_axis_angle_ = 0;

float cluster_size_;
int cluster_size_min_;
int cluster_size_max_;

float cart_length = 1.0f;
//-----------------------------------------------------------------------------


sos_fusion::MinMax cluster_info;
sos_fusion::PointArray cluster_array;

ros::Publisher pub, cluster_point_pub, cluster_array_pub;

// float degree;

float min_degree;
float max_degree;


int degree_cnt = -1;

E_MODE button = M_JOYSTICK;

bool print_fps_;
int frames;
clock_t start_time;
bool reset = true; //fps

#pragma endregion parameter

void degree_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    degree_cnt = (degree_cnt + 1) % 10; 

    min_degree = -(msg->data[2]);
	max_degree = -(msg->data[1]);
    cout << "min_degree = " << min_degree << endl;
    cout << "max_degree = " << max_degree << endl;

}

void button_cb(const std_msgs::Int8::ConstPtr& input)
{
	button = (E_MODE)input->data;
}

#pragma region detect_person

void detect_person(std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> data, float input_min_d, float input_max_d){
    

    float close_x = 999;
    float close_y = 999;

    int find_index = -1;

    cluster_array.cluster_center.clear();
    cluster_array.cluster_max.clear();
    cluster_array.cluster_min.clear();


    geometry_msgs::Point center_point;
    geometry_msgs::Point min_point;
    geometry_msgs::Point max_point;

    for(int i =0; i<data.size(); i++){

        Eigen::Vector4f centroid;
        Eigen::Vector4f min_p;
        Eigen::Vector4f max_p;

        pcl::compute3DCentroid(*data[i], centroid);
        pcl::getMinMax3D(*data[i], min_p, max_p);
      
        geometry_msgs::Point temp_center_point;
        temp_center_point.x = centroid[0];
        temp_center_point.y = centroid[1];
        temp_center_point.z = 0;

        geometry_msgs::Point temp_min_point;
        temp_min_point.x = min_p[0];
        temp_min_point.y = min_p[1];
        temp_min_point.z = 0;

        geometry_msgs::Point temp_max_point;
        temp_max_point.x = max_p[0];
        temp_max_point.y = max_p[1];
        temp_max_point.z = 0;

        cluster_array.cluster_center.push_back(temp_center_point);
        cluster_array.cluster_min.push_back(temp_min_point);
        cluster_array.cluster_max.push_back(temp_max_point);

    }

    if(button == M_SELF_DRIVING_BACK){
	    for(int i = 0; i < cluster_array.cluster_center.size(); i++){
		if((abs(cluster_array.cluster_max[i].y - cluster_array.cluster_min[i].y) >= 0.05 && abs(cluster_array.cluster_max[i].y - cluster_array.cluster_min[i].y) <= 0.35) 
		|| (abs(cluster_array.cluster_max[i].x - cluster_array.cluster_min[i].x) >= 0.05 && abs(cluster_array.cluster_max[i].x - cluster_array.cluster_min[i].x) <= 0.30)){
		    
		    ROS_INFO("[First Condition] find person condition");

		    float lidar_degree = atan(cluster_array.cluster_center[i].y / cluster_array.cluster_center[i].x) * 180 / M_PI;
		    ROS_INFO("[First Condition] lidar_degree : %f", lidar_degree);

		    if((lidar_degree > input_min_d && lidar_degree < input_max_d) && (sqrt(pow(close_x, 2)+pow(close_y, 2)) >= sqrt(pow(cluster_array.cluster_center[i].x, 2) + pow(cluster_array.cluster_center[i].y, 2)))){
		        find_index = i;
		        
		        ROS_INFO("[Second Condition] find person condition");

		        center_point.x = cluster_array.cluster_center[i].x;
		        center_point.y = cluster_array.cluster_center[i].y;
		        center_point.z = 0;
		    

		        min_point.x = cluster_array.cluster_min[i].x;
		        min_point.y = cluster_array.cluster_min[i].y;
		        min_point.z = 0;

		        max_point.x = cluster_array.cluster_max[i].x;
		        max_point.y = cluster_array.cluster_max[i].y;
		        max_point.z = 0;

		        close_x = center_point.x;
		        close_y = center_point.y;

		    }
		}
	    }


	    if(find_index != -1){
		cluster_array.cluster_center.erase(cluster_array.cluster_center.begin() + find_index);
		cluster_array.cluster_min.erase(cluster_array.cluster_min.begin() + find_index);
		cluster_array.cluster_max.erase(cluster_array.cluster_max.begin() + find_index);
	    }

	    cluster_info.cen = center_point;
	    cluster_info.min = min_point;
	    cluster_info.max = max_point;    
	}
}

#pragma endregion detect_person


sensor_msgs::PointCloud2 cloud_cb(const pcl::PCLPointCloud2 &input)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input, *cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ECE;

    ECE.setClusterTolerance(cluster_size_); // 1m 클러스터 간 차이 작으면 하나로 본다.

    ECE.setMinClusterSize(cluster_size_min_); // 몇 개부터 한 군집?
    ECE.setMaxClusterSize(cluster_size_max_); // 몇 개까지 한 군집?
    ECE.setSearchMethod(tree);
    ECE.setInputCloud(cloud_filtered);
    ECE.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;

    // 클러스터별 정보 수집, 출력, 저장
    std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {

            pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pcl::PointXYZ pt3;


            pt3.x = pt2.x = pt.x - cart_length, pt3.y = pt2.y = pt.y, pt3.z = pt2.z = pt.z;

            pt2.intensity = (float)(j + 1);

            cluster->points.push_back(pt3);

            TotalCloud.push_back(pt2);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }


    if (min_degree != 999 || degree_cnt != -1){
        detect_person(clusters, min_degree, max_degree);
    }


    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);

    output.header.frame_id = frame_id_;


    return output;
}

#pragma region scanconvert
//PCL convert, ROI ->
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    if (print_fps_)
        if (reset)
        {
            frames = 0;
            start_time = clock();
            reset = false;
        } //fps

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 input;
    projector.projectLaser(*scan_in, input);

    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    pcl_conversions::toPCL(input, *cloud);
    // Convert to PCL data type

    pcl::PCLPointCloud2 cloud_filtered;

    // Perform the set ROI
    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setInputCloud(cloudPtr);
    boxFilter.setMin(Eigen::Vector4f(x_axis_min_, y_axis_min_, z_axis_min_, 0));
    boxFilter.setMax(Eigen::Vector4f(x_axis_max_, y_axis_max_, z_axis_max_, 0));
    boxFilter.setTranslation(Eigen::Vector3f(0.0, 0.0, 0.0));        // 평행이동
    boxFilter.setRotation(Eigen::Vector3f(0.0, 0.0, z_axis_angle_)); //
    boxFilter.filter(cloud_filtered);

    pub.publish(cloud_cb(cloud_filtered));
    cluster_array_pub.publish(cluster_array);

    if(button == M_SELF_DRIVING_BACK){
        cluster_point_pub.publish(cluster_info);
    }

    if (print_fps_)
        if (++frames > 10)
        {
            std::cout << "[rpliadr_clustering] fps = " << float(frames) / (float(clock() - start_time) / CLOCKS_PER_SEC) << ", timestamp = " << clock() / CLOCKS_PER_SEC << std::endl;
            reset = true;
        } //fps

    ROS_INFO("published it.");
}
#pragma endregion scanconvert



#pragma region main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_back_cluster");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/lidar1/scan", 10, scan_cb);
    ros::Subscriber deg_sub = nh.subscribe("/ob_degree", 2, degree_Callback);
    ros::Subscriber button_sub = nh.subscribe("/button", 10, button_cb);
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("frame_id", frame_id_, "laser");
    private_nh.param<bool>("print_fps", print_fps_, false);
    private_nh.param<float>("x_axis_min", x_axis_min_, -7.0);
    private_nh.param<float>("x_axis_max", x_axis_max_, 0.0);

    private_nh.param<float>("y_axis_min", y_axis_min_, -3.0);
    private_nh.param<float>("y_axis_max", y_axis_max_, 3.0);

    private_nh.param<float>("z_axis_min", z_axis_min_, -3.0);
    private_nh.param<float>("z_axis_max", z_axis_max_, 3.0);

    private_nh.param<float>("cluster_size", cluster_size_, 0.08);

    private_nh.param<int>("cluster_size_min", cluster_size_min_, 3);
    private_nh.param<int>("cluster_size_max", cluster_size_max_, 1000);

    pub = private_nh.advertise<sensor_msgs::PointCloud2>("/rp_cluster_back", 5);
    cluster_point_pub = nh.advertise<sos_fusion::MinMax> ("/target_object_back_position", 5);
    cluster_array_pub = nh.advertise<sos_fusion::PointArray> ("/objects_back_positions", 5);


    ros::spin();
}
#pragma endregion main
