//
// Created by lisilin on 19-10-25.
//
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/ops_vectors.h>  // << for vector<>
#include <mrpt/math/utils.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h> // 验证传感器携带的sensor_pose是什么
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/version.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/bayes/CParticleFilter.h>

#include <Eigen/Dense>
#include <thread>


namespace pf_localization{
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::img;
using namespace mrpt::config;
using namespace mrpt::ros1bridge;
using namespace std;

class Parameters {
public:
    Parameters() {
        // load ros params
        ros::NodeHandle nh("~");
        if (!nh.getParam("map_file", map_file)) {
            ROS_ERROR("Can not load map_file param");
        }
        ASSERT_FILE_EXISTS_(map_file);
        ROS_INFO("map file is ready %s", map_file.c_str());

        // load pf params in pf_global.ini file
        string ini_file;
        if (!nh.getParam("ini_file", ini_file)) {
            ROS_ERROR("can not find ini file of pf localization");
        }
        ASSERT_FILE_EXISTS_(ini_file);
        ROS_INFO("ini_file is ready %s", ini_file.c_str());

        CConfigFile cfg;
        cfg.setFileName(ini_file);

        // -------------LocalizationExperiment-------------
        const string sect("LocalizationExperiment");
        particles_count = cfg.read_int(sect, "particles_count", 10000, /*Fail if not found*/true);
        logOutput_dir = cfg.read_string(sect, "logOutput_dir", "", /*Fail if not found*/ true);
        init_PDF_min_x = cfg.read_double(sect, "init_PDF_min_x", 0);
        init_PDF_min_y = cfg.read_double(sect, "init_PDF_min_y", 0);
        init_PDF_min_z = cfg.read_double(sect, "init_PDF_min_z", 0);
        init_PDF_max_x = cfg.read_double(sect, "init_PDF_max_x", 0);
        init_PDF_max_y = cfg.read_double(sect, "init_PDF_max_y", 0);
        init_PDF_max_z = cfg.read_double(sect, "init_PDF_max_z", 0);
        ROS_INFO("init PDF: %f %f %f %f %f %f\n", init_PDF_min_x, init_PDF_min_y, init_PDF_min_z, init_PDF_max_x, init_PDF_max_y, init_PDF_max_z);

        // dummy odom
        actOdom3dParams_.mm6DOFModel.additional_std_XYZ = cfg.read_double("DummyOdometryParams", "additional_std_XYZ", 0.01);
        actOdom3dParams_.mm6DOFModel.additional_std_angle = DEG2RAD(cfg.read_double("DummyOdometryParams", "additional_std_angle", 0.1));

        // -------------PF_options-------------
        pfOptions.loadFromConfigFile(cfg, "PF_options");

        // -------------KLD_options-------------
        pdfPredictionOptions.KLD_params.loadFromConfigFile(cfg, "KLD_options");

        // -------------MetricMap-------------
        mapList.loadFromConfigFile(cfg, "MetricMap");

        // 几个重要配置参数的输出
        pfOptions.dumpToConsole();
        mapList.dumpToConsole();
    }

public:
    string map_file;

    //--------------------------------------
    // pf params
    //--------------------------------------
    int experimentTestConvergenceAtStep = 1000;
    string logOutput_dir = "RESULTS_GLOBAL_CONVERGENCE_2019";
    bool SAVE_STATS_ONLY = false;
    int particles_count = 150000;
    double init_PDF_min_x = -320;
    double init_PDF_max_x = 100;
    double init_PDF_min_y = -300;
    double init_PDF_max_y = 20;
    double init_PDF_min_z = -0.80;
    double init_PDF_max_z = -0.70;

    CParticleFilter::TParticleFilterOptions pfOptions;
    TMonteCarloLocalizationParams pdfPredictionOptions;
    TSetOfMetricMapInitializers mapList;//metric map
    CActionRobotMovement3D::TMotionModelOptions actOdom3dParams_;
};

}//namespace end