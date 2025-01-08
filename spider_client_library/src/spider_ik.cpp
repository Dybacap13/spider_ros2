
#include <spider_ik.h>

namespace spider_client_library {

Eigen::Matrix<double, 4, 4> transformationDenaviteHartenberg(double a,
                                                             double alpha,
                                                             double d,
                                                             double tetta) {
  Eigen::Matrix<double, 4, 4> result;
  result << cos(tetta), -sin(tetta) * cos(alpha), sin(tetta) * sin(alpha),
      a * cos(tetta), sin(tetta), cos(tetta) * cos(alpha),
      -cos(tetta) * sin(alpha), a * sin(tetta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;
  return result;
}

SpiderIk::SpiderIk(RosParametrs ros_parametrs_) {
  ros_parametrs = ros_parametrs_;
}

TransformStamped SpiderIk::calculateDistanseOffsetBody(
    TransformStamped body_update, TransformStamped body_current) {
  // Считаем смещение между текущим и желаемым пололжением
  TransformStamped offset;
  offset.position.x = abs(body_update.position.x - body_current.position.x);
  offset.position.y = abs(body_update.position.y - body_current.position.y);
  // в теории это только если нагнуться
  offset.position.z = abs(body_update.position.z - body_current.position.z);
  return offset;
}

TransformStamped SpiderIk::calculateRotaryBodyZ(TransformStamped body_target,
                                                TransformStamped body_current)

{
  Eigen::Matrix<double, 3, 3> matrix_rotary;
  Eigen::Matrix<double, 3, 1> new_vector;
  Eigen::Matrix<double, 3, 1> old_vector;

  double gamma = body_target.orientation.pitch - body_current.orientation.pitch;

  matrix_rotary << cos(gamma), -sin(gamma), 0, sin(gamma), cos(gamma), 0, 0, 0,
      1;
  old_vector << body_current.orientation.pitch, body_current.orientation.roll,
      body_current.orientation.yaw;
  new_vector = matrix_rotary * old_vector;

  TransformStamped offset;
  offset.orientation.pitch = new_vector[0];
  offset.orientation.roll = new_vector[1];
  offset.orientation.yaw = new_vector[2];

  return offset;
}

std::vector<TransformStamped> SpiderIk::coordFeetFromBody(
    TransformStamped body, std::vector<JointLeg> joints) {
  // "rr", "rm", "rf", "lr", "lm", "lf"
  std::vector<TransformStamped> coord_feet_target;

  for (size_t index_leg = 0; index_leg < ros_parametrs.number_of_legs;
       index_leg++) {
    TransformStamped foot;
    auto coxa_z = sqrt(pow(ros_parametrs.coxa_to_center_x[index_leg], 2) +
                       pow(ros_parametrs.coxa_to_center_y[index_leg], 2));
    auto tetta_one = atan2(ros_parametrs.coxa_to_center_x[index_leg],
                           ros_parametrs.coxa_to_center_y[index_leg]);
    auto body_coxa = transformationDenaviteHartenberg(coxa_z, 0, 0, tetta_one);

    auto coxa_femur = transformationDenaviteHartenberg(
        ros_parametrs.coxa_length, PI / 2, 0, joints[index_leg].coxa);

    auto femur_tibia = transformationDenaviteHartenberg(
        ros_parametrs.femur_length, 0, 0, joints[index_leg].femur);

    auto tibia_foot = transformationDenaviteHartenberg(
        ros_parametrs.tibia_length, 0, 0, joints[index_leg].tibia);

    auto body_foot = body_coxa * coxa_femur * femur_tibia * tibia_foot;

    foot.position = getCoordinateFromTDH(body_foot);

    foot.position.x += body.position.x;
    foot.position.y += body.position.y;
    foot.position.z += body.position.z;

    coord_feet_target.emplace_back(foot);
  }
  return coord_feet_target;
}

Position SpiderIk::getCoordinateFromTDH(Eigen::Matrix<double, 4, 4> matrix) {
  Position coordinate;
  coordinate.x = matrix(3);
  coordinate.y = matrix(7);
  coordinate.z = matrix(11);
  return coordinate;
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
  for (int leg_index = 0; leg_index < ros_parametrs.number_of_legs;
       leg_index++) {
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
                   feet[leg_index].position.x -
                   ros_parametrs.coxa_to_center_x[leg_index];

    double cpr_y = feet[leg_index].position.y +
                   sign * (body.position.y + feet[leg_index].position.y +
                           ros_parametrs.coxa_to_center_y[leg_index]);

    double cpr_z = feet[leg_index].position.z + body.position.z +
                   ros_parametrs.tarsus_length - feet[leg_index].position.z;

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

    double feet_pos_x = -feet[leg_index].position.x + body.position.x -
                        body_pos_x + feet[leg_index].position.x;
    double feet_pos_y =
        feet[leg_index].position.y +
        sign * (body.position.y - body_pos_y + feet[leg_index].position.y);
    double feet_pos_z = feet[leg_index].position.z -
                        ros_parametrs.tarsus_length + body.position.z -
                        body_pos_z - feet[leg_index].position.z;
    if (leg_index == 0) {
      // std::cout <<"END X = "<< feet_pos_x<< std::endl;
      // std::cout <<"END Y = "<< feet_pos_y<< std::endl;
    }

    // Length between the Root and Foot Position ...Pythagorean theorem
    double femur_to_tarsus = sqrt(pow(feet_pos_x, 2) + pow(feet_pos_y, 2)) -
                             ros_parametrs.coxa_length;

    if (std::abs(femur_to_tarsus) >
        (ros_parametrs.femur_length + ros_parametrs.tibia_length)) {
      std::cout << "IK Solver cannot solve a foot position that is not within "
                   "leg reach!!!"
                << std::endl;

      break;
    }

    // Length of the sides of the triangle formed by the femur, tibia and tarsus
    // joints.
    double side_a = ros_parametrs.femur_length;
    double side_a_sqr = pow(ros_parametrs.femur_length, 2);

    double side_b = ros_parametrs.tibia_length;
    double side_b_sqr = pow(ros_parametrs.tibia_length, 2);

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

    result[leg_index].coxa = atan2(feet_pos_x, feet_pos_y) +
                             ros_parametrs.init_coxa_angle[leg_index];
    result[leg_index].femur = (PI / 2) - (theta + angle_b);

    result[leg_index].tibia = (PI)-angle_c;

    return result;
  }
}

}  // namespace spider_client_library
