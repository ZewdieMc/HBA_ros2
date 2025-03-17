#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <rosbag2_cpp/writer.hpp>
#include <ceres/ceres.h>

#include "ba.hpp"
#include "tools.hpp"
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

class VisualizeNode : public rclcpp::Node
{
public:
    VisualizeNode() : Node("visualize")
    {
        pub_map = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 100);
        pub_debug = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_debug", 100);
        pub_pose = this->create_publisher<geometry_msgs::msg::PoseArray>("/poseArrayTopic", 10);
        pub_trajectory = this->create_publisher<visualization_msgs::msg::Marker>("/trajectory_marker", 100);
        pub_pose_number = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pose_number", 100);

        this->declare_parameter("file_path", "");
        this->declare_parameter("downsample_size", 0.0);
        this->declare_parameter("pcd_name_fill_num", 0);
        this->declare_parameter("marker_size", 0.0);

        file_path = this->get_parameter("file_path").as_string();
        downsample_size = this->get_parameter("downsample_size").as_double();
        pcd_name_fill_num = this->get_parameter("pcd_name_fill_num").as_int();
        marker_size = this->get_parameter("marker_size").as_double();

        run();
    }

private:
    void run()
    {
        sensor_msgs::msg::PointCloud2 debugMsg, cloudMsg, outMsg;
        vector<mypcl::pose> pose_vec;

        pose_vec = mypcl::read_pose(file_path + "pose.json");
        size_t pose_size = pose_vec.size();
        cout << "pose size " << pose_size << endl;

        pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_full(new pcl::PointCloud<pcl::PointXYZRGB>);

        rclcpp::Time cur_t = this->now();
        geometry_msgs::msg::PoseArray parray;
        parray.header.frame_id = "camera_init";
        parray.header.stamp = cur_t;
        visualization_msgs::msg::MarkerArray markerArray;

        cout << "push enter to view" << endl;
        getchar();
        for (size_t i = 0; i < pose_size; i++)
        {
            mypcl::loadPCD(file_path + "pcd/", pcd_name_fill_num, pc_surf, i);

            pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
            pc_filtered->resize(pc_surf->points.size());
            int cnt = 0;
            for (size_t j = 0; j < pc_surf->points.size(); j++)
            {
                pc_filtered->points[cnt] = pc_surf->points[j];
                cnt++;
            }
            pc_filtered->resize(cnt);

            mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);
            downsample_voxel(*pc_filtered, downsample_size);

            pcl::toROSMsg(*pc_filtered, cloudMsg);
            cloudMsg.header.frame_id = "camera_init";
            cloudMsg.header.stamp = cur_t;
            pub_map->publish(cloudMsg);

            geometry_msgs::msg::Pose apose;
            apose.orientation.w = pose_vec[i].q.w();
            apose.orientation.x = pose_vec[i].q.x();
            apose.orientation.y = pose_vec[i].q.y();
            apose.orientation.z = pose_vec[i].q.z();
            apose.position.x = pose_vec[i].t(0);
            apose.position.y = pose_vec[i].t(1);
            apose.position.z = pose_vec[i].t(2);
            parray.poses.push_back(apose);
            pub_pose->publish(parray);

            // publish pose trajectory
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_init";
            marker.header.stamp = cur_t;
            marker.ns = "basic_shapes";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.pose.position.x = pose_vec[i].t(0);
            marker.pose.position.y = pose_vec[i].t(1);
            marker.pose.position.z = pose_vec[i].t(2);
            pose_vec[i].q.normalize();
            marker.pose.orientation.x = pose_vec[i].q.x();
            marker.pose.orientation.y = pose_vec[i].q.y();
            marker.pose.orientation.z = pose_vec[i].q.x();
            marker.pose.orientation.w = pose_vec[i].q.w();
            marker.scale.x = marker_size; // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.y = marker_size;
            marker.scale.z = marker_size;
            marker.color.r = float(1 - float(i) / pose_size);
            marker.color.g = float(float(i) / pose_size);
            marker.color.b = float(float(i) / pose_size);
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration::from_seconds(0);
            pub_trajectory->publish(marker);

            // publish pose number
            visualization_msgs::msg::Marker marker_txt;
            marker_txt.header.frame_id = "camera_init";
            marker_txt.header.stamp = cur_t;
            marker_txt.ns = "marker_txt";
            marker_txt.id = i; // Any marker sent with the same namespace and id will overwrite the old one
            marker_txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            ostringstream str;
            str << i;
            marker_txt.text = str.str();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker_txt.action = visualization_msgs::msg::Marker::ADD;
            marker_txt.pose.position.x = pose_vec[i].t(0) + marker_size;
            marker_txt.pose.position.y = pose_vec[i].t(1) + marker_size;
            marker_txt.pose.position.z = pose_vec[i].t(2);
            marker_txt.pose.orientation.x = pose_vec[i].q.x();
            marker_txt.pose.orientation.y = pose_vec[i].q.y();
            marker_txt.pose.orientation.z = pose_vec[i].q.x();
            marker_txt.pose.orientation.w = 1.0;
            marker_txt.scale.x = marker_size;
            marker_txt.scale.y = marker_size;
            marker_txt.scale.z = marker_size;
            marker_txt.color.r = 1.0f;
            marker_txt.color.g = 1.0f;
            marker_txt.color.b = 1.0f;
            marker_txt.color.a = 1.0;
            marker_txt.lifetime = rclcpp::Duration::from_seconds(0);
            if (i % GAP == 0)
                markerArray.markers.push_back(marker_txt);
            pub_pose_number->publish(markerArray);

            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }

        rclcpp::Rate loop_rate(1);
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_number;

    string file_path;
    double downsample_size, marker_size;
    int pcd_name_fill_num;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizeNode>());
    rclcpp::shutdown();
    return 0;
}