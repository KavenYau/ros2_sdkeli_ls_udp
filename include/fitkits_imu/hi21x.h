// Copyright <2020> [Copyright rossihwang]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <serial/serial.h>

#include <sensor_msgs/msg/imu.hpp>

namespace fitkits_imu {

struct Accel {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct Gyro {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct Mag {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct EulerI {
  int16_t pitch;
  int16_t roll;
  int16_t yaw;
};

// struct EulerF {
//   float pitch;
//   float roll;
//   float yaw;
// };

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

struct Aggr {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
};

class Hi21x : public rclcpp::Node {

 public:
  Hi21x(const std::string &name, rclcpp::NodeOptions const &options);
  virtual ~Hi21x();

 protected:
  std::thread process_thread_;
  std::atomic<bool> canceled_;

  std::shared_ptr<serial::Serial> serial_ptr_;
  std::string device_;
  int rate_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  void Process();
  void CreateParameter();
  bool HandleParameter(rclcpp::Parameter const &param);
  void ComputeCrc16(uint16_t *current_crc, const uint8_t *data, size_t length);  // FIXME: static?
};

}  // namespace fitkit_imu