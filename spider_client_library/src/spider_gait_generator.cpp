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
    for (size_t index = 0; index < cycle_gait.size(); index++) {
      cycle_gait[index] = !cycle_gait[index];
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

    if (!cycle_gait[leg]) {
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

}  // namespace spider_client_library