#pragma once
#include <ros/ros.h>
#include "utils/xform.h"
#include "dryden_model.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <rosflight_msgs/ROSflightSimState.h>
#include <std_msgs/Int32MultiArray.h>
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
    void resetWrenches();
    void uavStateCallback(const rosflight_msgs::ROSflightSimState &msg);
    void uavMotorPWMCallback(const std_msgs::Int32MultiArray &msg);

    double sat(double x, double max, double min)
    {
      if(x > max)
        return max;
      else if(x < min)
        return min;
      else
        return x;
    }

    double max(double x, double y)
    {
      return (x > y) ? x : y;
    }

    // Container for an Actuator
    struct Actuator{
      double max;
      double tau_up;
      double tau_down;
    };

    // Struct of Actuators
    // This organizes the physical limitations of the abstract torques and Force
    struct Actuators{
      Actuator l;
      Actuator m;
      Actuator n;
      Actuator F;
    } actuators_;

    struct Rotor{
      double max;
      std::vector<double> F_poly;
      std::vector<double> T_poly;
      double tau_up; // time constants for response
      double tau_down;
    };

    struct Motor{
      Rotor rotor;
      Eigen::Vector3d position;
      Eigen::Vector3d normal;
      int direction; // 1 for CW -1 for CCW
    };

    int num_rotors_;
    std::vector<Motor> motors_;
    double uav_mass_;
    Eigen::MatrixXd rotor_position_;
    Eigen::MatrixXd rotor_plane_normal_;
    Eigen::VectorXd rotor_rotation_direction_;
    std::vector<double> ground_effect_;
    Eigen::MatrixXd force_allocation_matrix_;
    Eigen::MatrixXd torque_allocation_matrix_;
    Eigen::VectorXd desired_forces_;
    Eigen::VectorXd desired_torques_;
    Eigen::VectorXd actual_forces_;
    Eigen::VectorXd actual_torques_;

    double motor_k1_;
    double motor_kT_;
    double motor_kB_;
    double motor_kf_;
    Vector4d motor_speeds_;

    ros::Subscriber motor_pwm_sub_;
    int pwm_outputs_[14];
//    Vector3d motor_Force_;
//    Vector3d motor_Torque_;
    ros::Publisher motor_wrench_pub_;

    dryden_model::DrydenWind wind_model_;
    double prev_time_;

//    Vector3d air_v_NED;
    Vector3d uav_v_UAV_;
    Vector3d uav_w_UAV_;
    Quatd q_NED_UAV_;
    double linear_mu_;
    double angular_mu_;

    Vector3d Force_;
    Vector3d Torque_;
    geometry_msgs::Wrench wrench_;
    visualization_msgs::Marker marker_;

    ros::Subscriber uav_state_sub_;
    ros::Publisher wrench_pub_;
    ros::Publisher marker_pub_;

    ros::NodeHandle nh_;
    ros::Timer timer_;
};

} // end namespace air_dynamics
