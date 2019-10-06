#pragma once
#include <ros/ros.h>
#include "utils/xform.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <rosflight_msgs/ROSflightSimState.h>
#include <Eigen/Core>
#include <random>

using namespace transforms;
using namespace Eigen;

// uav_dynamics linear and angular drag
// ...

namespace air_dynamics {

class AirDynamics
{
public:
    AirDynamics();

private:
    void onUpdate(const ros::TimerEvent &event);
    void getWindVelocities();
    void drawArrow();
    void resetWrench();
    void uavStateCallback(const rosflight_msgs::ROSflightSimState &msg);

    std::default_random_engine random_generator_;
    std::normal_distribution<double> w_gust_dist_;

    double Va_;
    double sigma_u_;
    double sigma_v_;
    double sigma_w_;
    double L_u_;
    double L_v_;
    double L_w_;

    Vector3d air_v_;
    Vector3d uav_v_;
    Vector3d uav_w_;

    Vector3d Force_;
    Vector3d Torque_;
    geometry_msgs::Wrench wrench_;

    ros::Subscriber uav_state_sub_;
    ros::Publisher wrench_pub_;
    ros::Publisher marker_pub_;

    ros::NodeHandle nh_;
    ros::Timer timer_;
};

} // end namespace air_dynamics
