//
// Created by lisilin on 19-10-25.
//
#include <pf_localization/pf_localization.h>

namespace pf_localization{


void PFLocalization::init() {
    ros::console::set_logger_level("pf_localization", ros::console::levels::Debug);
    // load global map and publish, for debug in rviz
    pcl::io::loadPCDFile(param_.map_file, global_map_);
    pcl::toROSMsg(global_map_, global_map_ros_);
    global_map_ros_.header.frame_id = "map";
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);//latch is true
    global_map_pub_.publish(global_map_ros_);
    printf("\033[1;32mGlobal Map is loaded and published!\033[0m\n");

    // ros subscribers
    cloud_sub_ = nh_.subscribe("/velodyne_points", 1, &PFLocalization::pointsCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &PFLocalization::odometryCallback, this);

    // ros publishers
    pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pf_pose", 2, true);

    particleFilterfInit();
}

void PFLocalization::particleFilterfInit() {
    tictac_.Tic();
    // calculate effective particles density
    auto init_min = TPose3D(param_.init_PDF_min_x, param_.init_PDF_min_y, param_.init_PDF_min_z,
            DEG2RAD(-180), DEG2RAD(0), DEG2RAD(0));
    auto init_max = mrpt::math::TPose3D(param_.init_PDF_max_x, param_.init_PDF_max_y, param_.init_PDF_max_z,
            DEG2RAD(180), DEG2RAD(0), DEG2RAD(0));
    double effective_map_area = (init_max.x - init_min.x)*(init_max.y - init_min.y);
    printf("\033[1;32mInitial PDF: %f particles/m2 \033[0m\n", param_.particles_count/effective_map_area);

    // load CSimplePointsMap from point cloud map
    CSimplePointsMap::Ptr simple_points_map = CSimplePointsMap::Create();
    fromROS(global_map_ros_, *simple_points_map);

    // init metric map
    CMultiMetricMap metric_map;
    metric_map.setListOfMaps(param_.mapList);
    metric_map.maps.push_back(mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr>(simple_points_map));

    // init CMonteCarloLocalization3D
    pdf_.options = param_.pdfPredictionOptions;
    pdf_.options.metricMap = &metric_map;

    // init CParticleFilter
    pf_.m_options = param_.pfOptions;

    // Reset uniform:
    pdf_.resetUniform(init_min, init_max, param_.particles_count);
    printf("\033[1;32mPDF of %u particles initialized in %.03fms \033[0m\n", param_.particles_count, 1000*tictac_.Tac());

    // init odometry_estimation_
    odometry_estimation_ = CPose2D(0, 0, 0);
}


void PFLocalization::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    static int init_cnt = 0;
    if (init_cnt <= 5) {
        init_cnt++;
        return;
    }
    // convert to mrpt format
    CSimplePointsMap::Ptr cloud = std::make_shared<CSimplePointsMap>();
    fromROS(*msg, *cloud);

    // add observations
    CObservationPointCloud::Ptr obs_cloud = std::make_shared<CObservationPointCloud>(); // new CObservationPointCloud
    obs_cloud->pointcloud = cloud; //都是利用多态
    obs_cloud->timestamp = fromROS(msg->header.stamp);
    obs_cloud->sensorLabel = "3D Lidar";
    CSensoryFrame::Ptr observations = std::make_shared<CSensoryFrame>(); // 每次重新初始化
    observations->insert(obs_cloud); // insert CObservation

    unique_lock<mutex> lock(odom_mutex_);
    if (!odom_received_) {
        printf("Still not received odometry topic!");
        return;
    }

    CPose3D odo_incr = CPose3D(pending_most_recent_odo_ - last_used_abs_odo_);
    last_used_abs_odo_ = pending_most_recent_odo_;
    CActionRobotMovement3D act_odom3d;
    act_odom3d.timestamp = fromROS(msg->header.stamp);
    act_odom3d.computeFromOdometry(odo_incr, param_.actOdom3dParams_);
    CActionCollection::Ptr action = std::make_shared<CActionCollection>(); //每次重新初始化
    action->insert(act_odom3d);

    printf("odom inc: %f %f %f\n", odo_incr.x(), odo_incr.y(), odo_incr.yaw());

    // save sensor data into queue
    sensor_data_.push_back(SensorData(dynamic_cast<CSensoryFrame *>(observations->clone()), dynamic_cast<CActionCollection *>(action->clone())));
}

void PFLocalization::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    unique_lock<mutex> lock(odom_mutex_);
    pending_most_recent_odo_ = fromROS(msg->pose.pose);

    if (!odom_received_) {
        odom_received_ = true;
        last_used_abs_odo_ = pending_most_recent_odo_;
    }
}

void PFLocalization::run() {
    // get sensor data
    if (sensor_data_.empty()) {
        printf("There is no sensor data\n");
        return;
    }
    auto data = sensor_data_.front();
    sensor_data_.pop_front();
    CSensoryFrame::Ptr obs = std::make_shared<CSensoryFrame>();
    CActionCollection::Ptr act = std::make_shared<CActionCollection>();
    obs = data.first;
    act = data.second;
    printf("size: obs %d, act %d\n", static_cast<int>(obs->size()), static_cast<int>(act->size()));
    if (static_cast<int>(obs->size()) == 0 || static_cast<int>(act->size()) == 0) {
        return;
    }

    // execute pf
    tictac_.Tic();
    printf("Execute PF, data size: %d\n", static_cast<int>(sensor_data_.size()));
    pf_.executeOn(pdf_, act.get(), obs.get(), &pf_stats_);

    // get run time
    double run_time = tictac_.Tac();
    printf("Done! in %.03fms, ESS=%f\n", 1e3f*run_time, pdf_.ESS());

    // get particle filter result
//    CActionRobotMovement2D::Ptr best_mov_estim = action->getBestMovementEstimation();
//    if (best_mov_estim) {
//        odometry_estimation_ += best_mov_estim->poseChange->getMeanVal();
//    }

    pdf_.getMean(pdf_estimation_);
    const auto[cov, mean] = pdf_.getCovarianceAndMean();
//    cout << "Odometry est: " << odometry_estimation_ << "\n";
    cout << "PDF est: " << pdf_estimation_
         << ", ESS (B.R.): " << pf_stats_.ESS_beforeResample
         << ", tr(cov): " << std::sqrt(cov.trace()) << "\n";

    // if convergence?
    if (sqrt(cov.det()) < 2) {
        printf("PF Localized Successfully!\n");
    }

    // publish particles, publish msg per 500ms
    static int loop_cnt = 0;
    if (loop_cnt++%3 == 0 && pub_particles_.getNumSubscribers() > 0) {
        static int seq = 0;
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = "map";
        poseArray.header.stamp = ros::Time::now();
        poseArray.header.seq = seq++;
        poseArray.poses.resize(pdf_.particlesCount());
        for (size_t i = 0; i < pdf_.particlesCount(); i++) {
            TPose3D p = pdf_.getParticlePose(i);
            poseArray.poses[i] = toROS_Pose(p);
        }
        pub_particles_.publish(poseArray);
    }

    // publish pf pose
    if (pub_pose_.getNumSubscribers() > 0) {
        geometry_msgs::PoseWithCovarianceStamped p;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();
        p.pose.pose = toROS_Pose(mean);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                p.pose.covariance[i*6 + j] = cov(i, j);
            }
        }
        pub_pose_.publish(p);
    }

}
}//namespace end