#include <spider_gait_generator.h>

namespace spider_client_library {

SpiderGaitGenerator::SpiderGaitGenerator(GaitParametrs parametrs) {
  gait_parametrs = parametrs;
}

Coefficients SpiderGaitGenerator::calculationOfCoefficients(
    Position coordinate_foot) {
  Coefficients result;
  if (current_point == gait_parametrs.number_points / 5) {
    pointIncrement();
  }
  result.a = gait_parametrs.step_lenght / 2;
  result.c = gait_parametrs.step_height;

  result.x0 = gait_parametrs.step_lenght / 2 + coordinate_foot.y;
  result.y0 = coordinate_foot.z;

  auto angle_offset =
      M_PI - (M_PI * current_point) / gait_parametrs.number_points;

  if (angle_offset == M_PI / 2) {
    // AAAAA
    result.k = M_PI / 2;
  } else {
    result.k = tan(angle_offset);
  }

  result.b = coordinate_foot.z - result.k * (gait_parametrs.step_lenght / 2);
  std::cout << "----" << std::endl;
  std::cout << "Coefficient" << std::endl;
  std::cout << "k = " << result.k << std::endl;
  std::cout << "b = " << result.b << std::endl;
  std::cout << "a = " << result.a << std::endl;
  std::cout << "c = " << result.c << std::endl;
  std::cout << "x0 = " << result.x0 << std::endl;
  std::cout << "y0 = " << result.y0 << std::endl;
  std::cout << "----" << std::endl;
  return result;
}

std::vector<Position>
SpiderGaitGenerator::calculationCoordinatesTrajectoryPoint(
    Coefficients coefficients) {
  // a * x2 + b * x + c = 0
  auto a =
      pow(coefficients.c, 2) + pow(coefficients.a, 2) * pow(coefficients.k, 2);
  auto b = 2 * coefficients.k * (coefficients.b - coefficients.y0) *
               pow(coefficients.a, 2) -
           2 * coefficients.x0 * pow(coefficients.c, 2);
  auto c = pow(coefficients.a, 2) * pow((coefficients.b - coefficients.y0), 2) -
           pow(coefficients.a, 2) * pow(coefficients.c, 2) +
           pow(coefficients.c, 2) * pow(coefficients.x0, 2);

  // D = b2 - 4ac
  auto D = pow(b, 2) - 4 * a * c;

  std::vector<Position> result;

  // anser = (- b +- sqrt(D)) / 2a
  if (D < 0) return result;
  if (D == 0) {
    Position anser;
    anser.y = (-b + sqrt(D)) / 2 * a;
    anser.z = coefficients.k * anser.y + coefficients.b;
    result.emplace_back(anser);
    return result;
  }

  Position anser_one;
  Position anser_two;

  anser_one.y = (-b + sqrt(D)) / (2 * a);
  anser_one.z = coefficients.k * anser_one.y + coefficients.b;

  anser_two.y = (-b - sqrt(D)) / (2 * a);
  anser_two.z = coefficients.k * anser_two.y + coefficients.b;

  result.emplace_back(anser_one);
  result.emplace_back(anser_two);
  std::cout << "----" << std::endl;
  std::cout << "Solver" << std::endl;
  std::cout << "D = " << D << std::endl;
  std::cout << "anser_one.y = " << anser_one.y << std::endl;
  std::cout << "anser_one.z  = " << anser_one.z << std::endl;
  std::cout << "anser_two.y = " << anser_two.y << std::endl;
  std::cout << "anser_two.z = " << anser_two.z << std::endl;
  std::cout << "----" << std::endl;
  return result;
}

Position SpiderGaitGenerator::checkoordinatesTrajectoryPoint(
    std::vector<Position> check_coordinates, Position coordinate_foot) {
  Position result;
  if (current_point != gait_parametrs.number_points) {
    step_x = 0.04;
  } else {
    step_x = 0;
  }
  for (size_t index = 0; index < check_coordinates.size(); index++) {
    if (check_coordinates[index].z == coordinate_foot.z and
        check_coordinates[index].y > coordinate_foot.y) {
      result = check_coordinates[index];
      break;
    }
    if (check_coordinates[index].z < coordinate_foot.z) continue;
    result = check_coordinates[index];
  }

  std::cout << "----" << std::endl;
  std::cout << "Check" << std::endl;
  std::cout << "result.y = " << result.y << std::endl;
  std::cout << "result.z = " << result.z << std::endl;
  std::cout << "----" << std::endl;
  return result;
}
void SpiderGaitGenerator::pointIncrement() {
  if (current_point == gait_parametrs.number_points) {
    current_point = 1;
    for (size_t index = 0; index < cycle_gait_.size(); index++) {
      cycle_gait_[index] = !cycle_gait_[index];
    }
  } else {
    current_point++;
  }
}

std::vector<TransformStamped> SpiderGaitGenerator::getGaitPoints(
    std::vector<TransformStamped> current_coordinates) {
  std::vector<TransformStamped> result;

  for (size_t leg = 0; leg < current_coordinates.size(); leg++) {
    TransformStamped led_position;

    if (!cycle_gait_[leg]) {
      led_position = current_coordinates[leg];
    } else {
      auto coeff = calculationOfCoefficients(current_coordinates[leg].position);
      auto probable_solutions = calculationCoordinatesTrajectoryPoint(coeff);
      led_position.position = checkoordinatesTrajectoryPoint(
          probable_solutions, current_coordinates[leg].position);
      led_position.position.x = current_coordinates[leg].position.x + step_x;
    }

    result.emplace_back(led_position);
  }
  std::cout << "POINT =  " << current_point << std::endl;
  pointIncrement();
  return result;
}

void SpiderGaitGenerator::getTrajectory(
    std::vector<TransformStamped> current_coordinates,
    std::vector<TransformStamped> target_coordinates, int points_number,
    std::vector<std::vector<TransformStamped>>& trajectory) {
  for (size_t point = 1; point < points_number + 1; point++) {
    float alpha = static_cast<float>(point) / points_number;
    std::vector<TransformStamped> point_i;
    for (size_t index_leg = 0; index_leg < current_coordinates.size();
         index_leg++) {
      TransformStamped intermediate;
      // интерполяционный коэффициент [0, 1]

      // Интерполяция позиции
      intermediate.position.x =
          current_coordinates[index_leg].position.x +
          alpha * (target_coordinates[index_leg].position.x -
                   current_coordinates[index_leg].position.x);

      intermediate.position.y =
          current_coordinates[index_leg].position.y +
          alpha * (target_coordinates[index_leg].position.y -
                   current_coordinates[index_leg].position.y);

      intermediate.position.z =
          current_coordinates[index_leg].position.z +
          alpha * (target_coordinates[index_leg].position.z -
                   current_coordinates[index_leg].position.z);

      point_i.emplace_back(intermediate);
    }
    trajectory.emplace_back(point_i);
  }
}

void SpiderGaitGenerator::generationOneIterationStepTrajectory(
    std::vector<TransformStamped> start_joint, double lenght_step,
    double lifting_step, std::vector<std::vector<TransformStamped>>& traj) {
  std::vector<int> cycle_gait = {1, 0, 1, 0, 1, 0};
  auto vector_offset = generatorVectorOffsetLegs(lenght_step, lifting_step);
  // std::vector<std::vector<spider_client_library::TransformStamped>> traj;
  std::vector<TransformStamped> last_joint = start_joint;
  for (size_t offset_step = 0; offset_step < vector_offset.size();
       offset_step++) {
    std::cout << "offset.x = " << vector_offset[offset_step].position.x
              << std::endl;
    std::cout << "offset.y = " << vector_offset[offset_step].position.y
              << std::endl;
    std::cout << "offset.z = " << vector_offset[offset_step].position.z
              << std::endl;
    auto step = offsetLegs(start_joint, cycle_gait, vector_offset[offset_step]);
    getTrajectory(last_joint, step, 20, traj);
    last_joint = step;
  }

  inverseGaitCycle(cycle_gait);

  for (size_t offset_step = 0; offset_step < vector_offset.size();
       offset_step++) {
    auto step = offsetLegs(start_joint, cycle_gait, vector_offset[offset_step]);
    getTrajectory(last_joint, step, 20, traj);
    last_joint = step;
  }
}

std::vector<TransformStamped> SpiderGaitGenerator::offsetLegs(
    std::vector<TransformStamped> current_joint, std::vector<int> cycle_gait,
    TransformStamped offset) {
  std::vector<TransformStamped> result = current_joint;
  for (size_t index_leg = 0; index_leg < current_joint.size(); index_leg++) {
    if (cycle_gait[index_leg]) {
      result[index_leg].position.x =
          current_joint[index_leg].position.x + offset.position.x;
      result[index_leg].position.y =
          current_joint[index_leg].position.y + offset.position.y;
      result[index_leg].position.z =
          current_joint[index_leg].position.z + offset.position.z;
    }
  }

  return result;
}

// void SpiderGaitGenerator::offsetParametrsLegs(double& leg,
//   double offset) {
// leg =  leg +offset.position.x;
// leg.position.y = offset.position.y;
// leg.position.z = offset.position.z;

// leg.orientation.pitch = offset.orientation.pitch;
// leg.orientation.roll = offset.orientation.roll;
// leg.orientation.yaw = offset.orientation.yaw;
// }

std::vector<TransformStamped> SpiderGaitGenerator::generatorVectorOffsetLegs(
    double step_length, double step_higth) {
  std::vector<TransformStamped> result;
  TransformStamped step_1;
  step_1.position.x = 0.0;
  step_1.position.y = 0.0;
  step_1.position.z = step_higth;
  result.emplace_back(step_1);

  TransformStamped step_2;
  step_2.position.x = step_length;
  step_2.position.y = 0.0;
  step_2.position.z = 0.0;
  result.emplace_back(step_2);

  TransformStamped step_3;
  step_3.position.x = 0.0;
  step_3.position.y = 0.0;
  step_3.position.z = 0.0;
  result.emplace_back(step_3);

  TransformStamped step_4;
  step_4.position.x = 0.0;
  step_4.position.y = 0.0;
  step_4.position.z = 0.0;
  result.emplace_back(step_4);

  return result;
}

void SpiderGaitGenerator::inverseGaitCycle(std::vector<int>& vector) {
  for (size_t index = 0; index < vector.size(); index++) {
    if (vector[index] == 1) {
      vector[index] = 0;
    } else {
      vector[index] = 1;
    }
  }
}
}  // namespace spider_client_library