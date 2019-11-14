//
// Created by lisilin on 19-10-25.
//
#pragma once
#include <mutex>
#include <deque>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>

#include <pf_localization/parameters.h>

namespace pf_localization{
/**
 * @brief pf localization based on mrpt
 * input: 3D point cloud(real time), odometry(real time), map(point cloud)
 * output: 6DOF pose in map
 */
class PFLocalization {
public:
    PFLocalization() {
        init();
    }

    ~PFLocalization() {};

    void run();
private:
    /**
     * @brief pf localization init
     */
    void init();

    /**
     * particle filter parameters init
     */
    void particleFilterfInit();

    /**
     * @brief 3d point cloud callback
     *        do pf localization algorithm
     * @param msg
     */
    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    /**
     * @brief odometry callback
     * @param msg
     */
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);


private:
    // pf params
    Parameters param_;

    // map
    pcl::PointCloud<pcl::PointXYZ> global_map_;
    sensor_msgs::PointCloud2 global_map_ros_;

    // ros related
    ros::NodeHandle nh_;
    ros::Publisher global_map_pub_, pub_particles_, pub_pose_;
    ros::Subscriber cloud_sub_, odom_sub_;

    // pf related
    CMonteCarloLocalization3D pdf_;
    CParticleFilter pf_;
    CParticleFilter::TParticleFilterStats pf_stats_;
    CTicTac tictac_;
    CMonteCarloLocalization3D::type_value pdf_estimation_;

    // action
    CPose3D pending_most_recent_odo_, last_used_abs_odo_;
    CPose2D odometry_estimation_;
    mutex odom_mutex_;
    bool odom_received_ = false;
};

}


