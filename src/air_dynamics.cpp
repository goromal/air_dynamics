#include "air_dynamics/air_dynamics.h"

namespace air_dynamics {

AirDynamics::AirDynamics() : nh_()
{
    random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());

}

} // end namespace air_dynamics
