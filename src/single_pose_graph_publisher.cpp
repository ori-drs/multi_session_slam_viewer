// Reads pose-graph file (g2o) and publishes data to be visualized in Rviz. 

#include <string>
#include <iostream>

#include <ros/ros.h> // ros node
#include <sensor_msgs/PointCloud2.h> // rviz msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h> // pointcloud
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> // convert from pcl and ros_msgs

#include <tf2/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h> // conversion between tf and eigen

int main(int argc, char **argv){

    // ================= NODE =================
    ros::init(argc, argv, "single_pose_graph_publisher");
    ros::NodeHandle nh;
    // ========================================


    // =============== PUBLISHER ==============
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped> ("pose", 100, true);
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path> ("path", 100, true);
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("raw", 100, true);
    // ========================================


    // =========== INPUT ARGUMENTS ============    
    std::string path_input = argv[1]; // ! (g2o_file_path /home/ori/Desktop/2022-03-03-hilti-runs-super-dense/exp01/exp01-01/slam_pose_graph.g2o)
    // ========================================


    // ============ OBTAIN INPUT ==============
    // path
    std::string path_parent = path_input.substr(0, path_input.rfind("/"));
    
    // define variable (for rviz path output)
    nav_msgs::Path path_msgs; 
    path_msgs.header.frame_id = "map";

    // fixed rate
    ros::Rate rate(atof(argv[2])); // ! (rate 10)

    // read line by line
    std::ifstream infile(path_input); std::string line;
    while (std::getline(infile, line) && ros::ok()){
        std::stringstream line_ss; line_ss << line; std::string type; line_ss >> type;

        // find line with node information
        if (type == "VERTEX_SE3:QUAT_TIME"){ 

            // =========== read pose ============
            // define variables
            int id; double x, y, z, qx, qy, qz, qw; uint32_t sec, nsec;

            // extract value
            line_ss >> id >> x >> y >> z >> qx >> qy >> qz >> qw >> sec >> nsec;
            ros::Time stamp; stamp.sec = sec; stamp.nsec = nsec;

            // store in tf transform 
            tf::Quaternion quat(qx, qy, qz, qw);
            tf::Vector3 origin(x, y, z);
            tf::Transform transform(quat, origin);
            // ========================================


            // ========== load pointcloud from file ===========
            // compute path
            std::stringstream path_sub;  path_sub << "/individual_clouds/cloud_" << sec << "_" << std::setfill('0') << std::setw(9) << nsec << ".pcd";
            std::string path_total = path_parent + path_sub.str();

            // read file
            pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
            pcl::io::loadPCDFile<pcl::PointXYZ>(path_total, cloud_xyz);
            // ========================================


            // =========== convert to msgs ============
            // pose msgs
            tf::Stamped<tf::Pose> pose_tf(transform, stamp, "map");
            geometry_msgs::PoseStamped pose_msgs;
            tf::poseStampedTFToMsg(pose_tf, pose_msgs);

            // path msgs
            path_msgs.poses.push_back(pose_msgs);
            
            // convert pointcloud to map frame
            pcl::PointCloud<pcl::PointXYZ> cloud_map;
            Eigen::Isometry3d transform_eigen;
            tf::transformTFToEigen(transform, transform_eigen);
            pcl::transformPointCloud(cloud_xyz, cloud_map, transform_eigen.translation().cast<float>(),
            Eigen::Quaternionf(transform_eigen.rotation().cast<float>()));  
         

            // pointcloud msgs
            sensor_msgs::PointCloud2 cloud_msgs;
            pcl::toROSMsg(cloud_map, cloud_msgs);
            cloud_msgs.header.frame_id = "map";
            cloud_msgs.header.stamp = stamp;
            // ========================================
            
            // ============== PUBLISH =================
            pub_pose.publish(pose_msgs);
            pub_path.publish(path_msgs);
            pub_cloud.publish(cloud_msgs);
            // ========================================

            // fixed rate
            rate.sleep();

        } else { continue; };
    };

    return 0;
};