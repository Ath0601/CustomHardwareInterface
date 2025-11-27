#include "fakecan/fakecan.hpp"
#include <algorithm>

namespace fakecan {

FakeCANInterface::FakeCANInterface() = default;
FakeCANInterface::~FakeCANInterface() = default;

bool FakeCANInterface::init(int dof, double dt_default) {
  if (dof <= 0) return false;
  dof_ = dof;
  dt_default_ = dt_default;
  positions_.assign(dof_, 0.0);
  velocities_.assign(dof_, 0.0);
  commands_.assign(dof_, 0.0);
  return true;
}

bool FakeCANInterface::read_position(int joint_id, double &pos) {
  std::lock_guard<std::mutex> l(mtx_);
  if (joint_id < 0 || joint_id >= dof_) return false;
  pos = positions_[joint_id];
  return true;
}

bool FakeCANInterface::read_velocity(int joint_id, double &vel) {
  std::lock_guard<std::mutex> l(mtx_);
  if (joint_id < 0 || joint_id >= dof_) return false;
  vel = velocities_[joint_id];
  return true;
}

bool FakeCANInterface::write_command(int joint_id, double cmd) {
  std::lock_guard<std::mutex> l(mtx_);
  if (joint_id < 0 || joint_id >= dof_) return false;
  // commands_ are treated as desired positions (position-control mode)
  commands_[joint_id] = cmd;
  return true;
}

void FakeCANInterface::update(double dt)
{
  std::lock_guard<std::mutex> l(mtx_);
  if (dt <= 0.0) dt = dt_default_;

  // simple position interpolation (no PID)
  const double alpha = 0.1;  // smaller = slower movement
  for (int i = 0; i < dof_; ++i)
  {
    double target = commands_[i];

    // smooth movement toward target
    positions_[i] += (target - positions_[i]) * alpha;

    // simple velocity estimation
    velocities_[i] = (target - positions_[i]);
  }
}


} // namespace fakecan
