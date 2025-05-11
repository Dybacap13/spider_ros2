#pragma once

#include <spider_client_interface.h>

#include <atomic>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace spider_client_library {

enum class ControlMode { DISABLE, EFFORT, POSITION, VELOCITY, TRAJECTORY };

class ActuatorState {
 public:
  ActuatorState() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

class Actuator {
 public:
  Actuator() : state() {}
  ~Actuator(){};

  ControlMode mode = ControlMode::DISABLE;
  ActuatorState state;
};

class SpiderClientImitation : public SpiderClientInterface {
 public:
  SpiderClientImitation(std::vector<double> initial_pose);
  ~SpiderClientImitation();

  void getJointData(std::vector<double>& data) override;
  void stop() override;
  void writeJointCommandPosition(std::vector<double> target_position) override;

  void writeTrajectory(std::vector<double> target_position) override;

 private:
  bool checkNan(std::vector<double> check_vector);
  void cycle();
  void update();
  void movePositionMode(size_t index_actuator);
  double getIntermediatePointsTrajectory(double first_pos, double next_point,
                                         double diff);

  bool comparetePosition(std::vector<double> current_position,
                         std::vector<double> target_position);

  void moveByTrajectory(size_t index_actuator);

  std::mutex mu;
  std::vector<std::string> joint_names;
  std::vector<double> joint_state_imit;
  std::vector<double> joint_position_target;
  std::vector<double> joint_velocity_target;
  std::vector<double> joint_effort_target;
  std::vector<std::shared_ptr<Actuator>> actuators;
  std::thread cycle_thread;
  std::vector<std::deque<double>> position_command;
  std::vector<double> last_position_traj;

  std::deque<int> index_trajectory;

  std::atomic<bool> terminated;
  uint32_t ctime;
  double rate = 5.0;
  double profile_velocity = 0.75;
};

}  // namespace spider_client_library