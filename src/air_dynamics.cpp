#include "air_dynamics/air_dynamics.h"

#define TO_ULONG(i) static_cast<unsigned long>(i)

namespace air_dynamics {

AirDynamics::AirDynamics() : nh_(), nh_private_("~")
{
    uav_state_sub_ = nh_.subscribe("uav_truth_NED", 1, &AirDynamics::uavStateCallback, this);
    motor_pwm_sub_ = nh_.subscribe("uav_motor_pwm", 1, &AirDynamics::uavMotorPWMCallback, this);
    motor_wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("uav_motor_wrench", 1);
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("uav_ext_wrench", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("wind_marker", 1);

//    motor_k1_ = nh_private_.param<double>("motor/k_1", 0.57);
//    motor_kB_ = nh_private_.param<double>("motor/k_beta", 0.7);
//    motor_kT_ = nh_private_.param<double>("motor/kn", 2.0e-5);
//    motor_kf_ = nh_private_.param<double>("motor/k_f", 0.0);

    ROS_ASSERT(nh_private_.getParam("k_1", motor_k1_));
    ROS_ASSERT(nh_private_.getParam("k_beta", motor_kB_));
    ROS_ASSERT(nh_private_.getParam("k_f", motor_kf_));
    ROS_ASSERT(nh_private_.getParam("num_rotors", num_rotors_));

    motor_kns_ = std::vector<double>(TO_ULONG(num_rotors_));
    ROS_ASSERT(nh_private_.getParam("kn", motor_kns_));

    memset(&pwm_outputs_, 0, sizeof(pwm_outputs_));

    uav_mass_ = nh_private_.param<double>("mass", 2.0);

    std::vector<double> rotor_positions; //(3 * num_rotors_);
    std::vector<double> rotor_vector_normal; //(3 * num_rotors_);
    std::vector<int> rotor_rotation_directions; //(num_rotors_);

    // No longer assuming that rotors are the same
    std::vector<Rotor> rotors(TO_ULONG(num_rotors_));

    ROS_ASSERT(nh_private_.getParam("rotor_positions", rotor_positions));
    ROS_ASSERT(nh_private_.getParam("rotor_vector_normal", rotor_vector_normal));
    ROS_ASSERT(nh_private_.getParam("rotor_rotation_directions", rotor_rotation_directions));

    double rotor_max = nh_private_.param<double>("rotor_max_thrust", 20.0);
    double tau_up = nh_private_.param<double>("rotor_tau_up", 0.2164);
    double tau_down = nh_private_.param<double>("rotor_tau_down", 0.1644);

    std::vector<double> F_poly_set(TO_ULONG(3*num_rotors_));
    std::vector<double> T_poly_set(TO_ULONG(3*num_rotors_));
    ROS_ASSERT(nh_private_.getParam("thrust_poly", F_poly_set));
    ROS_ASSERT(nh_private_.getParam("torque_poly", T_poly_set));

    for (int i = 0; i < num_rotors_; i++)
    {
        rotors[TO_ULONG(i)].max = rotor_max;
        rotors[TO_ULONG(i)].F_poly =
            {F_poly_set[TO_ULONG(3*i+0)],
             F_poly_set[TO_ULONG(3*i+1)],
             F_poly_set[TO_ULONG(3*i+2)]};
        rotors[TO_ULONG(i)].T_poly =
            {T_poly_set[TO_ULONG(3*i+0)],
             T_poly_set[TO_ULONG(3*i+1)],
             T_poly_set[TO_ULONG(3*i+2)]};
        rotors[TO_ULONG(i)].tau_up = tau_up;
        rotors[TO_ULONG(i)].tau_down = tau_down;
    }

    /* Load Rotor Configuration */
    motors_= std::vector<Motor>(TO_ULONG(num_rotors_));

    force_allocation_matrix_.resize(4,num_rotors_);
    torque_allocation_matrix_.resize(4,num_rotors_);
    for(int i = 0; i < num_rotors_; i++)
    {
      motors_[TO_ULONG(i)].rotor = rotors[TO_ULONG(i)];
      motors_[TO_ULONG(i)].position.resize(3);
      motors_[TO_ULONG(i)].normal.resize(3);
      for (int j = 0; j < 3; j++)
      {
        motors_[TO_ULONG(i)].position(j) = rotor_positions[TO_ULONG(3*i + j)];
        motors_[TO_ULONG(i)].normal(j) = rotor_vector_normal[TO_ULONG(3*i + j)];
      }
      motors_[TO_ULONG(i)].normal.normalize();
      motors_[TO_ULONG(i)].direction = rotor_rotation_directions[TO_ULONG(i)];

      Eigen::Vector3d moment_from_thrust = motors_[TO_ULONG(i)].position.cross(motors_[TO_ULONG(i)].normal);
      Eigen::Vector3d moment_from_torque = motors_[TO_ULONG(i)].direction * motors_[TO_ULONG(i)].normal;

      // build allocation_matrices
      force_allocation_matrix_(0,i) = moment_from_thrust(0); // l
      force_allocation_matrix_(1,i) = moment_from_thrust(1); // m
      force_allocation_matrix_(2,i) = moment_from_thrust(2); // n
      force_allocation_matrix_(3,i) = motors_[TO_ULONG(i)].normal(2); // F

      torque_allocation_matrix_(0,i) = moment_from_torque(0); // l
      torque_allocation_matrix_(1,i) = moment_from_torque(1); // m
      torque_allocation_matrix_(2,i) = moment_from_torque(2); // n
      torque_allocation_matrix_(3,i) = 0.0; // F
    }

    ROS_INFO_STREAM("allocation matrices:\nFORCE \n" << force_allocation_matrix_ << "\nTORQUE\n" << torque_allocation_matrix_ << "\n");

    // Initialize size of dynamic force and torque matrices
    desired_forces_.resize(num_rotors_);
    desired_torques_.resize(num_rotors_);
    actual_forces_.resize(num_rotors_);
    actual_torques_.resize(num_rotors_);

    for (int i = 0; i < num_rotors_; i++)
    {
      desired_forces_(i)=0.0;
      desired_torques_(i)=0.0;
      actual_forces_(i)=0.0;
      actual_torques_(i)=0.0;
    }

    double w_x_mean  = nh_private_.param<double>("wind_x_mean", 0.0);
    double w_x_bound = nh_private_.param<double>("wind_x_bound", 0.5);
    double w_y_mean  = nh_private_.param<double>("wind_y_mean", 0.0);
    double w_y_bound = nh_private_.param<double>("wind_y_bound", 0.5);
    double w_z_mean  = nh_private_.param<double>("wind_z_mean", 0.0);
    double w_z_bound = nh_private_.param<double>("wind_z_bound", 0.1);
    linear_mu_ = nh_private_.param<double>("linear_mu", 0.05);
    angular_mu_ = nh_private_.param<double>("angular_mu", 0.0005);
    std::string wind_frame = nh_private_.param<std::string>("wind_vis_frame", "boat");

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
    timer_ = nh_.createTimer(ros::Duration(ros::Rate(1000)), &AirDynamics::onUpdate, this);
}

void AirDynamics::onUpdate(const ros::TimerEvent &event)
{
    // TODO: lookup transform between NED and boat if boat in case vis frame ever different ++++

    resetWrenches();

    double time = ros::Time::now().toSec();
    double dt = time - prev_time_;
    prev_time_ = time;
    Vector3d air_v_NED = wind_model_.getWind(dt);
//    std::cout << air_v_NED.transpose() << std::endl;
    Vector3d air_v_UAV = q_NED_UAV_.rotp(air_v_NED);
    Vector3d airspeed_UAV = uav_v_UAV_ - air_v_UAV;

    /// Calculate motor forces
    double omega_Total = 0.0;
//    std::cout << "RECEIVED: " << pwm_outputs_[0] << " " << pwm_outputs_[1] << " " << pwm_outputs_[2] << " " << pwm_outputs_[3] << std::endl; // ----
    for (int i = 0; i < num_rotors_; i++)
    {
      // First, figure out the desired force output from passing the signal into the quadratic approximation
      double signal = pwm_outputs_[i];
      desired_forces_(i,0) = motors_[TO_ULONG(i)].rotor.F_poly[0]*signal*signal +
                             motors_[TO_ULONG(i)].rotor.F_poly[1]*signal +
                             motors_[TO_ULONG(i)].rotor.F_poly[2];
      desired_torques_(i,0) = motors_[TO_ULONG(i)].rotor.T_poly[0]*signal*signal +
                              motors_[TO_ULONG(i)].rotor.T_poly[1]*signal +
                              motors_[TO_ULONG(i)].rotor.T_poly[2];

      // Then, Calculate Actual force and torque for each rotor using first-order dynamics
      double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[TO_ULONG(i)].rotor.tau_up : motors_[TO_ULONG(i)].rotor.tau_down;
      double alpha = time/(tau + time);
      actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[TO_ULONG(i)].rotor.max, 0.0);
      actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[TO_ULONG(i)].rotor.max, 0.0);
      motor_speeds_(i, 0) = sqrt(abs(actual_forces_(i) / motor_kns_[TO_ULONG(i)]));
      omega_Total += motor_speeds_(i, 0);
    }

    // Use the allocation matrix to calculate the body-fixed force and torques
    Eigen::Vector4d output_forces = force_allocation_matrix_*actual_forces_;
    Eigen::Vector4d output_torques = torque_allocation_matrix_*actual_torques_;
    Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;

    // Publish motor wrench
    geometry_msgs::Wrench motor_wrench;
    motor_wrench.force.x = 0.0;
    motor_wrench.force.y = 0.0;
    motor_wrench.force.z = output_forces_and_torques(3);
    motor_wrench.torque.x = output_forces_and_torques(0);
    motor_wrench.torque.y = output_forces_and_torques(1);
    motor_wrench.torque.z = output_forces_and_torques(2);
    motor_wrench_pub_.publish(motor_wrench);

    /// Calculate external aerodynamic forces

    // Linear and angular drag forces
    Force_ += -linear_mu_ * airspeed_UAV;
    Torque_ += -angular_mu_ * uav_w_UAV_.cwiseProduct(uav_w_UAV_);

    // Blade flapping effect
    if (omega_Total > 0.0)
    {
        for (int i = 0; i < num_rotors_; i++)
        {
            // determine allocation of k_1 to each motor and calculate force and torques
            double k1i = motor_k1_ * motor_speeds_(i, 0) / omega_Total;

            // Calculate forces
            Force_ -= k1i * (Matrix3d::Identity() - motors_[TO_ULONG(i)].normal*motors_[TO_ULONG(i)].normal.transpose()) * airspeed_UAV;

            // Calculate torque about roll axis
            double as1phi = -motor_kf_ * airspeed_UAV(1);
            Torque_(0) -= actual_forces_(i) * motors_[TO_ULONG(i)].position(2) * sin(as1phi);
            Torque_(0) += motor_kB_ * as1phi;

            // Calcualte torque about pitch axis
            double as1theta = motor_kf_ * airspeed_UAV(0);
            Torque_(0) -= actual_forces_(i) * motors_[TO_ULONG(i)].position(2) * sin(as1theta);
            Torque_(0) += motor_kB_ * as1theta;
        }
    }

    // Publish aerodynamic wrench
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

void AirDynamics::uavStateCallback(const rosflight_sil::ROSflightSimState &msg)
{
    uav_v_UAV_ = Vector3d(msg.vel.x, msg.vel.y, msg.vel.z);
    uav_w_UAV_ = Vector3d(msg.w.x, msg.w.y, msg.w.z);
    q_NED_UAV_ = Quatd((Vector4d() << msg.att.w, msg.att.x, msg.att.y, msg.att.z).finished());
}

void AirDynamics::uavMotorPWMCallback(const std_msgs::Int32MultiArray &msg)
{
    int i = 0;
    for (std::vector<int>::const_iterator it = msg.data.begin(); it != msg.data.end(); ++it)
    {
        pwm_outputs_[i] = *it;
        i++;
    }
}

void AirDynamics::resetWrenches()
{
//    motor_Force_.setZero();
//    motor_Torque_.setZero();
    Force_.setZero();
    Torque_.setZero();
}

} // end namespace air_dynamics
