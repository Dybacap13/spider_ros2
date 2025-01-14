#include <spider_gait_generator.h>

namespace spider_client_library {

SpiderGaitGenerator::SpiderGaitGenerator(GaitParametrs parametrs) {
  gait_parametrs = parametrs;
}

Coefficients SpiderGaitGenerator::calculationOfCoefficients(
    int point, Position coordinate_foot) {
  Coefficients result;

  result.a = gait_parametrs.step_lenght / 2;
  result.c = gait_parametrs.step_height;

  result.x0 = gait_parametrs.step_lenght / 2 + coordinate_foot.y;
  result.y0 = coordinate_foot.z;

  auto angle_offset = M_PI - (M_PI * point) / gait_parametrs.number_points;
  if (angle_offset == M_PI / 2) {
    // AAAAA
    result.k = M_PI / 2;
  } else {
    result.k = tan(angle_offset);
  }

  result.b = coordinate_foot.z - result.k * (gait_parametrs.step_lenght / 2);
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

  anser_one.y = (-b + sqrt(D)) / 2 * a;
  anser_one.z = coefficients.k * anser_one.y + coefficients.b;

  anser_two.y = (-b - sqrt(D)) / 2 * a;
  anser_two.z = coefficients.k * anser_two.y + coefficients.b;

  result.emplace_back(anser_one);
  result.emplace_back(anser_two);
  return result;
}

}  // namespace spider_client_library