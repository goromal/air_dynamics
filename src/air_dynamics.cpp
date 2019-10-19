#include "air_dynamics/air_dynamics.h"

namespace air_dynamics {

AirDynamics::AirDynamics() : nh_()
{
    uav_state_sub_ = nh_.subscribe("uav_truth_NED", 1, &AirDynamics::uavStateCallback, this);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("uav_ext_wrench", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("wind_marker", 1);

    double w_x_mean  = nh_.param<double>("wind_x_mean", 0.0);
    double w_x_bound = nh_.param<double>("wind_x_bound", 0.5);
    double w_y_mean  = nh_.param<double>("wind_y_mean", 0.0);
    double w_y_bound = nh_.param<double>("wind_y_bound", 0.5);
    double w_z_mean  = nh_.param<double>("wind_z_mean", 0.0);
    double w_z_bound = nh_.param<double>("wind_z_bound", 0.1);
    linear_mu_ = nh_.param<double>("uav_linear_mu", 0.05);
    angular_mu_ = nh_.param<double>("uav_angular_mu", 0.0005);
    std::string wind_frame = nh_.param<std::string>("wind_vis_frame", "boat");

    uav_v_UAV_.setZero();
    uav_w_UAV_.setZero();
    memset(&wrench_, 0.0, sizeof(wrench_));

    double arrow_scale = 0.1;
    marker_.header.frame_id = wind_frame;
    marker_.id = 3;
    marker_.type = visualization_msgs::Marker::ARROW;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose = geometry_msgs::Pose();
    marker_.pose.position.x = -2.0;
    marker_.pose.position.y = 0.0;
    marker_.pose.position.z = 7.5;
    marker_.scale.x = arrow_scale;
    marker_.scale.y = arrow_scale;
    marker_.scale.z = arrow_scale;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;

    wind_model_.initialize(w_x_mean, w_y_mean, w_z_mean, w_x_bound, w_y_bound, w_z_bound);
    prev_time_ = ros::Time::now().toSec();
    timer_ = nh_.createTimer(ros::Duration(ros::Rate(100)), &AirDynamics::onUpdate, this);
}

void AirDynamics::onUpdate(const ros::TimerEvent &event)
{
    // TODO: lookup transform between NED and boat ++++

    resetWrench();

    double time = ros::Time::now().toSec();
    double dt = time - prev_time_;
    prev_time_ = time;
    Vector3d air_v_NED = wind_model_.getWind(dt);

    Vector3d air_v_UAV = q_NED_UAV_.rotp(air_v_NED);
    Force_ = -linear_mu_ * (uav_v_UAV_ - air_v_UAV);
    Torque_ = -angular_mu_ * uav_w_UAV_.cwiseProduct(uav_w_UAV_);
    wrench_.force.x = Force_.x();
    wrench_.force.y = Force_.y();
    wrench_.force.z = Force_.z();
    wrench_.torque.x = Torque_.x();
    wrench_.torque.y = Torque_.y();
    wrench_.torque.z = Torque_.z();
    wrench_pub_.publish(wrench_);

    Vector3d air_v_vis = Vector3d(air_v_NED.x(), -air_v_NED.y(), -air_v_NED.z());
    double arrow_scale = air_v_NED.norm();
    if (arrow_scale > 0.0)
    {
        Quatd q_arrow = Quatd::from_two_unit_vectors(Vector3d(1.,0.,0.),air_v_vis.normalized());
        marker_.scale.x = 3.0 * arrow_scale;
        marker_.pose.orientation.w = q_arrow.w();
        marker_.pose.orientation.x = q_arrow.x();
        marker_.pose.orientation.y = q_arrow.y();
        marker_.pose.orientation.z = q_arrow.z();
        marker_pub_.publish(marker_);
    }
}

void AirDynamics::uavStateCallback(const rosflight_msgs::ROSflightSimState &msg)
{
    uav_v_UAV_ = Vector3d(msg.vel.x, msg.vel.y, msg.vel.z);
    uav_w_UAV_ = Vector3d(msg.w.x, msg.w.y, msg.w.z);
    q_NED_UAV_ = Quatd((Vector4d() << msg.att.w, msg.att.x, msg.att.y, msg.att.z).finished());
}

void AirDynamics::resetWrench()
{
    Force_.setZero();
    Torque_.setZero();
}

} // end namespace air_dynamics
