#pragma once
#include "spider_struct_data.h"

namespace spider_client_library {

class SpiderClientInterface {
 public:
  virtual void getJointData(std::vector<double> & data) = 0;
  virtual void writeJointCommandPosition(
      std::vector<double> target_position) = 0;
  virtual void stop() = 0;
};
}  // namespace spider_client_library
