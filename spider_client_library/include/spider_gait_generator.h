#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "spider_struct_data.h"

namespace spider_client_library {

struct Coefficients {
  // y = kx + b
  double k;
  double b;

  // (x-x0)2 / a2 + (y-y0)2/c2 = 1
  double a;
  double c;

  double x0;
  double y0;
};

class SpiderGaitGenerator {
 public:
  SpiderGaitGenerator(GaitParametrs parametrs);

 private:
  Coefficients calculationOfCoefficients(int point, Position coordinate_foot);

  std::vector<Position> calculationCoordinatesTrajectoryPoint(
      Coefficients coefficients);
  GaitParametrs gait_parametrs;
};
}  // namespace spider_client_library