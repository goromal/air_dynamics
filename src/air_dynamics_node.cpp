#include "air_dynamics/air_dynamics.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "air_dynamics_node");
    air_dynamics::AirDynamics AD;
    ros::spin();
    return 0;
}
