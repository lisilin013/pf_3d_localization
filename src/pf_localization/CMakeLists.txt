cmake_minimum_required(VERSION 3.1)
project(pf_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


# fine PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# find eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find ros pkg
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        tf
        )
catkin_package(
        INCLUDE_DIRS include
        #          LIBRARIES pf_localization
        CATKIN_DEPENDS roscpp tf nav_msgs sensor_msgs pcl_ros pcl_conversions
        #        DEPENDS
)


# find mrpt
set(MRPT_DIR "/home/nrsl/Downloads/mrpt-demo")
file(GLOB MRPT_LIBS "${MRPT_DIR}/build/lib/*.so")
include_directories(
        ${MRPT_DIR}/libs/bayes/include
        ${MRPT_DIR}/libs/comms/include
        ${MRPT_DIR}/libs/config/include
        ${MRPT_DIR}/libs/containers/include
        ${MRPT_DIR}/libs/core/include
        ${MRPT_DIR}/libs/db/include
        ${MRPT_DIR}/libs/detectors/include
        ${MRPT_DIR}/libs/expr/include
        ${MRPT_DIR}/libs/graphs/include
        ${MRPT_DIR}/libs/graphslam/include
        ${MRPT_DIR}/libs/gui/include
        ${MRPT_DIR}/libs/hmtslam/include
        ${MRPT_DIR}/libs/hwdrivers/include
        ${MRPT_DIR}/libs/img/include
        ${MRPT_DIR}/libs/io/include
        ${MRPT_DIR}/libs/kinematics/include
        ${MRPT_DIR}/libs/maps/include
        ${MRPT_DIR}/libs/math/include
        ${MRPT_DIR}/libs/nanoflann/include
        ${MRPT_DIR}/libs/nav/include
        ${MRPT_DIR}/libs/obs/include
        ${MRPT_DIR}/libs/opengl/include
        ${MRPT_DIR}/libs/pbmap/include
        ${MRPT_DIR}/libs/poses/include
        ${MRPT_DIR}/libs/random/include
        ${MRPT_DIR}/libs/rtti/include
        ${MRPT_DIR}/libs/serialization/include
        ${MRPT_DIR}/libs/slam/include
        ${MRPT_DIR}/libs/system/include
        ${MRPT_DIR}/libs/tclap/include
        ${MRPT_DIR}/libs/tfest/include
        ${MRPT_DIR}/libs/topography/include
        ${MRPT_DIR}/libs/typemeta/include
        ${MRPT_DIR}/libs/vision/include
        ${MRPT_DIR}/libs/vision/include-lgpl
)


# find ros1bridge src
include_directories(
        src/ros1bridge/include
        include
        ${catkin_INCLUDE_DIRS}
        )
file(GLOB MRPT_ROS1_BRIDGE_SRC "src/ros1bridge/src/*")


########################################################
# build ros exe
########################################################
add_executable(pf_localization_node
        src/pf_localization_node.cpp
        src/pf_localization.cpp
        ${MRPT_ROS1_BRIDGE_SRC}
        )


target_link_libraries(pf_localization_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${MRPT_LIBS}
        ${OpenCV_LIBS}
        )


