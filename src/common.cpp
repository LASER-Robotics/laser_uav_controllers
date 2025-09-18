#include <laser_uav_controllers/common.hpp>

namespace laser_uav_controllers
{
/* thrustToThrotle() //{ */
double thrustToThrotle(double a, double b, double thrust) {
  // --- The normalization is maded by quadratic curve that represent thrust force x throtle normalized
  return (a * sqrt(thrust)) + b;
}
//}

/* thrustToThrotle() //{ */
double thrustToThrotle(double a, double b, double thrust, double thrust_max, double thrust_min) {
  thrust = std::min(thrust, thrust_max);
  thrust = std::max(thrust, thrust_min);

  // --- The normalization is maded by quadratic curve that represent thrust force x throtle normalized
  return (a * sqrt(thrust)) + b;
}
//}
}  // namespace laser_uav_controllers
