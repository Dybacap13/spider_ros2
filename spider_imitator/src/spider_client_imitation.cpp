#include <spider_client_imitation.h>

namespace spider_client_library {

SpiderClientImitation::SpiderClientImitation(std::vector<double> initial_pose)
    : terminated(false) {
  actuators.resize(initial_pose.size());

  for (auto& actuator : actuators) {
    actuator = std::make_shared<Actuator>();
  }
  for (size_t i = 0; i < initial_pose.size(); i++) {
    actuators[i]->state.position = initial_pose[i];
  }

  joint_position_target.resize(initial_pose.size(), 0.0);
  joint_velocity_target.resize(initial_pose.size(), 0.0);
  joint_effort_target.resize(initial_pose.size(), 0.0);

  ctime = 1.0 / rate * 1000 * 1000;  // microseconds
  cycle_thread = std::thread(&SpiderClientImitation::cycle, this);
}

SpiderClientImitation::~SpiderClientImitation() {
  terminated = true;
  cycle_thread.join();
}

void SpiderClientImitation::cycle() {
  auto cycletime = std::chrono::microseconds(this->ctime);
  while (!terminated.load()) {
    auto start = std::chrono::high_resolution_clock::now();
    update();
    auto end = std::chrono::high_resolution_clock::now();
    const auto delta = end - start;
    if (delta > cycletime) {
      // std::cout << "System too slow for cycle time " << cycletime.count()
      //           << "ms sending takes " << delta.count() << "ns" << std::endl;
    } else {
      std::this_thread::sleep_for(cycletime - delta);
    }
  }
}

void SpiderClientImitation::update() {
  for (size_t index_actuator = 0; index_actuator < actuators.size();
       index_actuator++)

  {
    if (actuators[index_actuator]->mode == ControlMode::POSITION) {
      movePositionMode(index_actuator);
      continue;
    }
    if (actuators[index_actuator]->mode == ControlMode::DISABLE) {
      continue;
    }

    throw std::runtime_error("Unknown joint mode.");
  }
}

void SpiderClientImitation::movePositionMode(size_t index_actuator) {
  const std::lock_guard<std::mutex> lock(mu);
  if (actuators[index_actuator]->state.position ==
      joint_position_target[index_actuator]) {
    actuators[index_actuator]->state.velocity = 0.0;
    return;
  }
  actuators[index_actuator]->state.position += getIntermediatePointsTrajectory(
      actuators[index_actuator]->state.position,
      joint_position_target[index_actuator], profile_velocity / rate);
}

void SpiderClientImitation::getJointData(std::vector<double>& data) {
  const std::lock_guard<std::mutex> lock(mu);
  std::vector<double> joint_states;
  data.resize(actuators.size());
  for (std::size_t i = 0; i < actuators.size(); i++) {
    data[i] = actuators[i]->state.position;
  }
}

double SpiderClientImitation::getIntermediatePointsTrajectory(double first_pos,
                                                              double next_point,
                                                              double diff) {
  return (first_pos < next_point) ? std::min(diff, next_point - first_pos)
                                  : std::max(-diff, next_point - first_pos);
}

void SpiderClientImitation::writeJointCommandPosition(
    std::vector<double> target_position) {
  const std::lock_guard<std::mutex> lock(mu);
  if (!checkNan(target_position)) {
    return;
  }
  for (std::size_t current = 0; current < target_position.size(); current++) {
    actuators[current]->mode = ControlMode::POSITION;
    joint_position_target[current] = target_position[current];
  }
}

bool SpiderClientImitation::checkNan(std::vector<double> check_vector) {
  for (auto value : check_vector) {
    if (value != value) return false;
  }
  return true;
}

void SpiderClientImitation::stop() {
  const std::lock_guard<std::mutex> lock(mu);
  for (std::size_t current = 0; current < actuators.size(); current++) {
    actuators[current]->mode = ControlMode::DISABLE;
  }
  std::cout << "STOP";
}

}  // namespace spider_client_library