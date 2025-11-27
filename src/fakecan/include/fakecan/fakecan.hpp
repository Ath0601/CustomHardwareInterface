#pragma once
#include <vector>
#include <mutex>

namespace fakecan {

class FakeCANInterface {
public:
  FakeCANInterface();
  ~FakeCANInterface();

  bool init(int dof, double dt_default = 0.001);

  // read simulated measurements
  bool read_position(int joint_id, double &pos);
  bool read_velocity(int joint_id, double &vel);

  // write command for joint
  bool write_command(int joint_id, double cmd);

  // perform simulation step (dt in seconds)
  void update(double dt);

  int get_dof() const { return dof_; }

private:
  int dof_{0};
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
  std::mutex mtx_;
  double dt_default_{0.001};
};

} // namespace fakecan
