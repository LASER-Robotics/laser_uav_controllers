#include <laser_uav_controllers/nmpc_controller.hpp>
#include <stdio.h>

namespace laser_uav_controllers
{

/* NmpcController::NmpcController() {} */

NmpcController::NmpcController() {

  /* nlmpc_solver_.setLoggerLevel(mpc::Logger::LogLevel::DEEP); */
  nlmpc_solver_.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);
  nlmpc_solver_.setDiscretizationSamplingTime(0.1);

  setCostFunction();
  setUavDynamics();

  Q << 120, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 100, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 100, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 7e-1, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 7e-1, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 7e-1, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-5, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-5, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-5;

}

void NmpcController::setCostFunction() {
  auto res = nlmpc_solver_.setObjectiveFunction(
      [&](const mpc::mat<_nxh_ + 1, _nx_> &x, const mpc::mat<_nxh_ + 1, _ny_> &y, const mpc::mat<_nxh_ + 1, _nu_> &u, const double &e) {
        double cost = 0;

        for (auto i = 0; i < _nxh_ - 1; i++) {
          cost += (Q * (x.row(i).transpose() - reference)).squaredNorm();
        }

        cost += ((Q * 2) * (x.row(_nxh_ - 1).transpose() - reference)).norm();

        return cost;
      });
}

void NmpcController::setUavDynamics() {
  auto res = nlmpc_solver_.setStateSpaceFunction([&](mpc::cvec<_nx_> &dx, const mpc::cvec<_nx_> &x, const mpc::cvec<_nu_> &u, const unsigned int &) {
    /* Eigen::Vector4d u_thrust; */
    /* u_thrust << ct_ * pow(u(0), 2), ct_ * pow(u(1), 2), ct_ * pow(u(2), 2), ct_ * pow(u(3), 2); */
    Eigen::Vector4d u_thrust;
    u_thrust  << u(0),u(1),u(2),u(3);

    Eigen::Matrix4d g1;
    g1 << 1, 1, 1, 1, 
          l_ * sin(beta_), -l_ * sin(beta_), -l_ * sin(beta_), l_ * sin(beta_), 
          -l_ * cos(beta_), -l_ * cos(beta_), l_ * cos(beta_), l_ * cos(beta_), 
          cq_ / ct_, -cq_ / ct_, cq_ / ct_, -cq_ / ct_;

    Eigen::Vector4d total_thrust_and_torque = g1 * u;

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.8;

    Eigen::Vector3d z_body;
    z_body << 0, 0, 1;

    Eigen::Vector3d linear_acceleration = ((total_thrust_and_torque(0) * z_body) / mass_) + gravity;

    dx(VX) = linear_acceleration(0);
    dx(VY) = linear_acceleration(1);
    dx(VZ) = linear_acceleration(2);

    Eigen::Quaterniond q1(x(QW), x(QX), x(QY), x(QZ));
    Eigen::Quaterniond q2(0, x(OX), x(OY), x(OZ));

    Eigen::Quaterniond rotational_speed = q1 * q2;

    dx(QW) = 0.5 * rotational_speed.w();
    dx(QX) = 0.5 * rotational_speed.x();
    dx(QY) = 0.5 * rotational_speed.y();
    dx(QZ) = 0.5 * rotational_speed.z();

    Eigen::Vector3d linear_speed;
    linear_speed << x(VX), x(VY), x(VZ);

    Eigen::Vector3d linear_speed_body = q1.toRotationMatrix() * linear_speed;

    dx(X) = linear_speed_body(0);
    dx(Y) = linear_speed_body(1);
    dx(Z) = linear_speed_body(2);

    Eigen::Matrix3d inertial;
    inertial.diagonal() << Ixx_, Iyy_, Izz_;

    Eigen::Vector3d angular_speed;
    angular_speed << x(OX), x(OY), x(OZ);

    Eigen::Vector3d torque;
    torque << total_thrust_and_torque(1), total_thrust_and_torque(2), total_thrust_and_torque(3);

    Eigen::Vector3d angular_acceleration = inertial.inverse() * (torque - angular_speed.cross(inertial * angular_speed));

    dx(OX) = angular_acceleration(0);
    dx(OY) = angular_acceleration(1);
    dx(OZ) = angular_acceleration(2);
  });
}

void NmpcController::getCorrection(geometry_msgs::msg::Pose ref, const nav_msgs::msg::Odometry msg) {
  mpc::cvec<_nx_> x0;

  // --- Update reference
  reference(X)  = ref.position.x;  // x
  reference(Y)  = ref.position.y;  // y
  reference(Z)  = ref.position.z;  // z
  reference(QW) = 1.00;            // qw
  reference(QX) = 0.00;            // qx
  reference(QY) = 0.00;            // qy
  reference(QZ) = 0.00;            // qz
  reference(VX) = 0.00;            // vbx
  reference(VY) = 0.00;            // vby
  reference(VZ) = 0.00;            // vbz
  reference(OX) = 0.00;            // wx
  reference(OY) = 0.00;            // wy
  reference(OZ) = 0.00;            // wz

  // --- Read Estimate
  x0(X) = msg.pose.pose.position.x;
  x0(Y) = msg.pose.pose.position.y;
  x0(Z) = msg.pose.pose.position.z;

  x0(QW) = msg.pose.pose.orientation.w;
  x0(QX) = msg.pose.pose.orientation.x;
  x0(QY) = msg.pose.pose.orientation.y;
  x0(QZ) = msg.pose.pose.orientation.z;

  x0(VX) = msg.twist.twist.linear.x;
  x0(VY) = msg.twist.twist.linear.y;
  x0(VZ) = msg.twist.twist.linear.z;

  x0(OX) = msg.twist.twist.angular.x;
  x0(OY) = msg.twist.twist.angular.y;
  x0(OZ) = msg.twist.twist.angular.z;

  mpc::NLParameters params;
  params.maximum_iteration = 100;
  nlmpc_solver_.setOptimizerParameters(params);

  auto res = nlmpc_solver_.getLastResult();
  res.cmd.setZero();
  res = nlmpc_solver_.optimize(x0, res.cmd);

  auto seq = nlmpc_solver_.getOptimalSequence();
  std::cout << "Optimal sequence (input): " << seq.input << std::endl;
  std::cout << "Optimal sequence (state): " << seq.state << std::endl;
  std::cout << "reference: " << reference.transpose() << std::endl;
  std::cout << nlmpc_solver_.getExecutionStats();
}
}  // namespace laser_uav_controllers
