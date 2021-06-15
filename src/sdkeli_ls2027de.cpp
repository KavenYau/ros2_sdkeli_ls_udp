#include "sdkeli_ls_udp/sdkeli_ls2027de.h"
#include <cmath>

namespace sdkeli_ls_udp {

LS2027DE::LS2027DE(const std::string &name, rclcpp::NodeOptions const &options)
    : rclcpp::Node(name, options), canceled_(false) {}

LS2027DE::~LS2027DE() { canceled_.store(true); }

bool LS2027DE::init() {
  CreateParameter();
  parser_ptr_.reset(new sdkeli_ls_udp::CSDKeliLs1207DEParser());

  RCLCPP_INFO(this->get_logger(), "range_min: %f", range_min_);
  parser_ptr_->SetRangeMin(range_min_);
  RCLCPP_INFO(this->get_logger(), "range_max: %f", range_max_);
  parser_ptr_->SetRangeMax(range_max_);

  // RCLCPP_INFO(this->get_logger(), "time_increment: %f", time_increment_);
  // parser_ptr_->SetTimeIncrement(time_increment_);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  parser_ptr_->SetFrameId(frame_id_);

  SDKeliLsConfig config;
  config.debug_mode = false;
  config.intensity = intensity_;
  // config.min_ang = -0.75 * M_PI;
  // config.max_ang = 0.75 * M_PI;
  config.min_ang = angle_min_ * M_PI / 180.0;
  config.max_ang = angle_max_ * M_PI / 180.0;
  config.skip = 0;
  config.auto_reboot = false;
  config.time_offset = -0.001;
  config.inverse = inverse_;

  sdkeli_ptr_.reset(
      new sdkeli_ls_udp::CSDKeliLsCommonUdp(hostname_, port_, time_limit_, parser_ptr_, shared_from_this(), config));
  /*Device has been initliazed successfully*/
  int32_t result = sdkeli_ptr_->Init();
  if (result != sdkeli_ls_udp::ExitSuccess) {
    return false;
  }

  process_thread_ = std::thread([this]() -> void {
    int32_t ret = sdkeli_ls_udp::ExitSuccess;
    while (rclcpp::ok() && !canceled_.load() && ret == sdkeli_ls_udp::ExitSuccess) {
      // ret = sdkeli_ptr_->LoopOnce();
      sdkeli_ptr_->LoopOnce();
    }
  });

  return true;
}

void LS2027DE::Process() {}

void LS2027DE::CreateParameter() {
  hostname_ = declare_parameter<std::string>("hostname", "192.168.0.10");
  port_ = declare_parameter<int32_t>("port", 2112);

  time_limit_ = declare_parameter<int32_t>("time_limit", 5);
  data_subscribed_ = declare_parameter<bool>("data_subscribed", false);
  device_number_ = declare_parameter<int32_t>("device_number", 0);
  frame_id_ = declare_parameter<std::string>("frame_id", "base_scan");
  range_min_ = declare_parameter<double>("range_min", 0.05);
  range_max_ = declare_parameter<double>("range_max", 20);
  time_increment_ = declare_parameter<double>("time_increment", -1);
  angle_min_ = declare_parameter<int32_t>("angle_min", -135);
  angle_max_ = declare_parameter<int32_t>("angle_max", 135);
  intensity_ = declare_parameter<bool>("intensity", true);
  inverse_ = declare_parameter<bool>("inverse", false);

  set_on_parameters_set_callback(
      [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto const &p : parameters) {
          result.successful &= HandleParameter(p);
        }
        return result;
      });
}

bool LS2027DE::HandleParameter(rclcpp::Parameter const &param) {
  if (param.get_name() == "hostname") {
    hostname_ = param.as_string();
  } else if (param.get_name() == "port") {
    port_ = param.as_int();
  } else if (param.get_name() == "time_limit") {
    time_limit_ = param.as_int();
  } else if (param.get_name() == "data_subscribed") {
    data_subscribed_ = param.as_int();
  } else if (param.get_name() == "device_number") {
    device_number_ = param.as_int();
  } else if (param.get_name() == "frame_id") {
    frame_id_ = param.as_int();
  } else if (param.get_name() == "range_min") {
    range_min_ = param.as_int();
  } else if (param.get_name() == "range_max") {
    range_max_ = param.as_int();
  } else if (param.get_name() == "time_increment") {
    time_increment_ = param.as_int();
  } else if (param.get_name() == "angle_min") {
    angle_min_ = param.as_int();
  } else if (param.get_name() == "angle_max") {
    angle_max_ = param.as_int();
  } else if (param.get_name() == "intensity") {
    intensity_ = param.as_bool();
  } else if (param.get_name() == "inverse") {
    inverse_ = param.as_bool();
  } else {
    return false;
  }
  return true;
}
}
