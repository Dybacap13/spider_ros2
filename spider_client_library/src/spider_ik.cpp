
#include <spider_client_library/spider_ik.h>

namespace spider_client_library {

void SpiderIk::calculateIK(TransformStamped body_update) {
  // Считаем смещение между текущим и желаемым пололжением
  double offset_x = abs(body_update.position.x - body_current.position.x);
  double offset_y = abs(body_update.position.y - body_current.position.y);
  // в теории это только если нагнуться
  double offset_z = abs(body_update.position.z - body_current.position.z);
}

// в положении стоя!
std::vector<TransformStamped> SpiderIk::coordFeetFromBody(
    TransformStamped body) {
  // "rr", "rm", "rf", "lr", "lm", "lf"
  std::vector<TransformStamped> result;
}

Trig SpiderIk::getSinCos(double angle_rad) {
  Trig result;
  result.sine = sin(angle_rad);
  result.cosine = cos(angle_rad);
  return result;
}

std::vector<JointLeg> SpiderIk::IK(const std::vector<TransformStamped> feet,
                                   const TransformStamped body, bool state) {
  std::vector<JointLeg> result;
  double sign = -1.0;
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
    if (leg_index <= 2) {
      // sign = -1.0;
      sign = 1.0;
    } else {
      sign = 1.0;
    }

    // First calculate sinus and co-sinus for each angular axis
    Trig A = getSinCos(body.orientation.yaw + feet[leg_index].orientation.yaw);
    Trig B = getSinCos(body.orientation.pitch);
    Trig G = getSinCos(body.orientation.roll);

    // Calculating totals from the feet to center of the body
    double cpr_x = feet[leg_index].position.x + body.position.x -
                   INIT_FOOT_POS_X[leg_index] - COXA_TO_CENTER_X[leg_index];

    double cpr_y = feet[leg_index].position.y +
                   sign * (body.position.y + INIT_FOOT_POS_Y[leg_index] +
                           COXA_TO_CENTER_Y[leg_index]);

    double cpr_z = feet[leg_index].position.z + body.position.z +
                   TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index];

    // Calculation of angular matrix of body (Tait-Bryan angles Z, Y, X)
    // http://en.wikipedia.org/wiki/Euler_angles
    double body_pos_x =
        cpr_x -
        ((cpr_x * A.cosine * B.cosine) +
         (cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine) +
         (cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine));

    double body_pos_y =
        cpr_y -
        ((cpr_x * B.cosine * A.sine) +
         (cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine) +
         (cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine));

    double body_pos_z =
        cpr_z - ((-cpr_x * B.sine) + (cpr_y * B.cosine * G.sine) +
                 (cpr_z * B.cosine * G.cosine));

    double feet_pos_x = -INIT_FOOT_POS_X[leg_index] + body.position.x -
                        body_pos_x + feet[leg_index].position.x;
    double feet_pos_y =
        INIT_FOOT_POS_Y[leg_index] +
        sign * (body.position.y - body_pos_y + feet[leg_index].position.y);
    double feet_pos_z = INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH +
                        body.position.z - body_pos_z -
                        feet[leg_index].position.z;
    if (leg_index == 0) {
      // std::cout <<"END X = "<< feet_pos_x<< std::endl;
      // std::cout <<"END Y = "<< feet_pos_y<< std::endl;
    }

    // Length between the Root and Foot Position ...Pythagorean theorem
    double femur_to_tarsus =
        sqrt(pow(feet_pos_x, 2) + pow(feet_pos_y, 2)) - COXA_LENGTH;

    if (std::abs(femur_to_tarsus) > (FEMUR_LENGTH + TIBIA_LENGTH)) {
      std::cout << "IK Solver cannot solve a foot position that is not within "
                   "leg reach!!!"
                << std::endl;

      break;
    }

    // Length of the sides of the triangle formed by the femur, tibia and tarsus
    // joints.
    double side_a = FEMUR_LENGTH;
    double side_a_sqr = pow(FEMUR_LENGTH, 2);

    double side_b = TIBIA_LENGTH;
    double side_b_sqr = pow(TIBIA_LENGTH, 2);

    double side_c = sqrt(pow(femur_to_tarsus, 2) + pow(feet_pos_z, 2));
    double side_c_sqr = pow(side_c, 2);

    // We are using the law of cosines on the triangle formed by the femur,
    // tibia and tarsus joints.
    double angle_b =
        acos((side_a_sqr - side_b_sqr + side_c_sqr) / (2.0 * side_a * side_c));
    double angle_c =
        acos((side_a_sqr + side_b_sqr - side_c_sqr) / (2.0 * side_a * side_b));

    // Angle of line between the femur and Tarsus joints with respect to
    // feet_pos_z.
    double theta = atan2(femur_to_tarsus, feet_pos_z);

    // Resulting joint angles in radians.

    result[leg_index].coxa =
        atan2(feet_pos_x, feet_pos_y) + INIT_COXA_ANGLE[leg_index];
    result[leg_index].femur = (PI / 2) - (theta + angle_b);

    result[leg_index].tibia = (PI)-angle_c;

    return result;
  }
}

}  // namespace spider_client_library
