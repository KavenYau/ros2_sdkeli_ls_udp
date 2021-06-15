#ifndef SDKELI_LS2027DE_H__
#define SDKELI_LS2027DE_H__

#include <sdkeli_ls_udp/sdkeli_ls1207de_parser.h>
#include <sdkeli_ls_udp/sdkeli_ls_common_udp.h>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sdkeli_ls_udp {

class LS2027DE : public rclcpp::Node {
 public:
  LS2027DE(const std::string &name, rclcpp::NodeOptions const &options);
  virtual ~LS2027DE();
  bool init();

 protected:
  std::thread process_thread_;
  std::atomic<bool> canceled_;

  std::string hostname_;
  int32_t port_;
  int32_t time_limit_;
  bool data_subscribed_;
  int32_t device_number_;
  std::string frame_id_;
  double range_min_;
  double range_max_;
  double time_increment_;
  int32_t angle_min_;
  int32_t angle_max_;
  bool intensity_;
  bool inverse_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::shared_ptr<sdkeli_ls_udp::CSDKeliLs1207DEParser> parser_ptr_;
  std::shared_ptr<sdkeli_ls_udp::CSDKeliLsCommonUdp> sdkeli_ptr_;

 protected:
  void Process();
  void CreateParameter();
  bool HandleParameter(rclcpp::Parameter const &param);
};
}
#endif  // SDKELI_LS2027DE_H__