// Copyright <2020> [Copyright rossihwang]

#include "fitkits_imu/hi21x.h"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/clock.hpp>

namespace fitkits_imu {

constexpr uint8_t kPackPre = 0x5A;
constexpr uint8_t kPackType = 0xA5;
constexpr double kG = 9.80665;
constexpr double kPi = 3.14;

enum class ParserState {
  PRE,
  TYPE,
  LEN,
  CRC,
  DATA,
};

enum class PackId {
  USER = 0x90,
  ACC = 0xA0,
  GYRO = 0xB0,
  MAG = 0xC0,
  EULER_I = 0xD0,
  EULER_F = 0xD9,
  QUAT = 0xD1,
  PRESS = 0xF0,
  AGGR = 0x70,
};

std::unordered_map<PackId, size_t> PackLength = {
  {PackId::USER, 1},
  {PackId::ACC, 6},
  {PackId::GYRO, 6},
  {PackId::MAG, 6},
  {PackId::EULER_I, 6},
  {PackId::EULER_F, 12},
  {PackId::QUAT, 16},
  {PackId::PRESS, 4},
  {PackId::AGGR, 36},
};

Hi21x::Hi21x(const std::string &name, rclcpp::NodeOptions const &options)
    : rclcpp::Node(name, options),
      canceled_(false),
      device_("/dev/ttyS5"),
      rate_(115200) {
  
  auto timeout = serial::Timeout(1, 100, 0, 100, 0);  // ms
  serial_ptr_ = std::make_shared<serial::Serial>(device_, rate_, timeout);

  // if (options.use_intra_process_comms()) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", 10);
  // }

  RCLCPP_INFO(this->get_logger(), "CreateParameter");
  CreateParameter();

  process_thread_ = std::thread{[this]() -> void {
    ParserState state = ParserState::PRE;
    uint16_t buffer_len = 0;
    uint16_t package_len = 0;
    auto ros_imu_data = sensor_msgs::msg::Imu();
    static uint8_t buffer[512] = {0};
    uint16_t crc;
    uint16_t pack_crc;
    rclcpp::Rate rate(std::chrono::milliseconds(16));  // default output frequency 60Hz
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    // serial_ptr_->flush();
    
    while (rclcpp::ok() && !canceled_.load()) {
      switch (state) {
        case ParserState::PRE:
          RCLCPP_DEBUG(this->get_logger(), "ParserState::PRE");
          buffer_len = serial_ptr_->read(buffer, 1);
          RCLCPP_DEBUG(this->get_logger(), "%x ", *buffer);
          if (buffer_len == 1 && buffer[0] == kPackPre) {
            crc = 0;  // init crc
            state = ParserState::TYPE;
          }
          break;
        case ParserState::TYPE:
          RCLCPP_DEBUG(this->get_logger(), "ParserState::TYPE");
          buffer_len = serial_ptr_->read(buffer, 1);
          RCLCPP_DEBUG(this->get_logger(), "%x ", *buffer);
          if (buffer_len == 1) {
            if (buffer[0] == kPackType) {
              state = ParserState::LEN;
            } else if (buffer[0] == kPackPre) {
              // skip
            } else {
              state = ParserState::PRE; // reset
            }
          } else {  // almost impossible to reach here
            state = ParserState::PRE;
          }
          break;
        case ParserState::LEN:
          RCLCPP_DEBUG(this->get_logger(), "ParserState::LEN");
          buffer_len = serial_ptr_->read(buffer, 2);
          for (int i = 0; i < 2; ++i) {
            RCLCPP_DEBUG(this->get_logger(), "%x", buffer[i]);
          }
          if (buffer_len == 2) {
            package_len = static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
            RCLCPP_DEBUG(this->get_logger(), "length: %d", package_len);
            uint8_t data[] = {kPackPre, kPackType, buffer[0], buffer[1]};
            ComputeCrc16(&crc, data, 4);
            state = ParserState::CRC;
          } else {  // almost impossible to reach here
            state = ParserState::PRE;
          }
          break;
        case ParserState::CRC:
          RCLCPP_DEBUG(this->get_logger(), "ParserState::CRC");
          buffer_len = serial_ptr_->read(buffer, 2);
          for (int i = 0; i < 2; ++i) {
            RCLCPP_DEBUG(this->get_logger(), "%x", buffer[i]);
          }
          if (buffer_len == 2) {
            // store crc
            pack_crc = static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
            state = ParserState::DATA;
          } else {
            state = ParserState::PRE;
          }
          break;
        case ParserState::DATA: {
          RCLCPP_DEBUG(this->get_logger(), "ParserState::DATA");
          serial_ptr_->read(buffer, 1);
          ComputeCrc16(&crc, buffer, 1);
          auto pack_id = static_cast<PackId>(buffer[0]);
          RCLCPP_DEBUG(this->get_logger(), "pack id: %x, lenght: %d", static_cast<uint8_t>(pack_id), PackLength[pack_id]);
          // FIXME: validate the pack_id
          try {
            serial_ptr_->read(buffer, PackLength[pack_id]);
            ComputeCrc16(&crc, buffer, PackLength[pack_id]);
          } catch (const std::out_of_range &e) {
            RCLCPP_WARN(this->get_logger(), "Invalid pack id");
            state = ParserState::PRE;
            break;
          }
          // Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
          switch (pack_id) {
            case PackId::ACC: {
              RCLCPP_DEBUG(this->get_logger(), "acc pack(%d)", PackLength[pack_id]);
              Accel *accel = reinterpret_cast<Accel*>(buffer);
              ros_imu_data.linear_acceleration.x = accel->x / 1000.0 * kG;  // m/s^2
              ros_imu_data.linear_acceleration.y = accel->y / 1000.0 * kG;
              ros_imu_data.linear_acceleration.z = accel->z / 1000.0 * kG;
              break;
            }
            case PackId::GYRO: {
              RCLCPP_DEBUG(this->get_logger(), "gryo pack(%d)", PackLength[pack_id]);
              Gyro *gyro = reinterpret_cast<Gyro*>(buffer);
              ros_imu_data.angular_velocity.x = gyro->x / 10.0 * (kPi / 180);  // degree / s
              ros_imu_data.angular_velocity.y = gyro->y / 10.0 * (kPi / 180);
              ros_imu_data.angular_velocity.z = gyro->z / 10.0 * (kPi / 180);
              break;
            }
            case PackId::QUAT: {
              RCLCPP_DEBUG(this->get_logger(), "quat pack(%d)", PackLength[pack_id]);
              Quat *quat = reinterpret_cast<Quat*>(buffer);
              ros_imu_data.orientation.x = quat->x;
              ros_imu_data.orientation.y = quat->y;
              ros_imu_data.orientation.z = quat->z;
              ros_imu_data.orientation.w = quat->w;
              break;
            }
            case PackId::AGGR:
            case PackId::USER:
            case PackId::MAG:
            case PackId::PRESS:
            case PackId::EULER_I:
            case PackId::EULER_F:
            default:
              RCLCPP_DEBUG(this->get_logger(), "invalid pack: %x", static_cast<uint8_t>(pack_id));
              // ignore these
              break;
          }
          package_len -= (1 + PackLength[pack_id]);  // subtract the length of package id and data
          if (package_len == 0) {
            RCLCPP_DEBUG(this->get_logger(), "pack_crc: %x, crc: %x", pack_crc, crc);
            if (pack_crc == crc) {
              // state = ParserState::PRE;  // reset
              
              ros_imu_data.header.stamp = clock->now();
              ros_imu_data.header.frame_id = "hi216_imu";
              
              imu_pub_->publish(ros_imu_data);
              RCLCPP_INFO(this->get_logger(), "hi21x publish imu data");
              rate.sleep();
            } else {
              RCLCPP_WARN(this->get_logger(), "hi21x crc check failed");
            }
            state = ParserState::PRE;  // reset
          }
          break;
        }
        default:
          RCLCPP_DEBUG(this->get_logger(), "ParserState::DEFAULT");
          break;
      }
    }
  }};
}

Hi21x::~Hi21x() {
  canceled_.store(true);
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
}

void Hi21x::CreateParameter() {
  device_ = declare_parameter<std::string>("device", "/dev/ttyS5");
  rate_ = declare_parameter<int>("rate", 115200);

  set_on_parameters_set_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const &p : parameters) {
        result.successful &= HandleParameter(p);
      }
      return result;
    }
  ); 
}

bool Hi21x::HandleParameter(rclcpp::Parameter const &param) {

  if (param.get_name() == "device") {
    device_ = param.as_string();
  } else if (param.get_name() == "rate") {
    rate_ = param.as_int();
  } else {
    return false;
  }
  return true;
}

void Hi21x::ComputeCrc16(uint16_t *current_crc, const uint8_t *data, size_t length) {
  uint32_t crc = *current_crc;
  uint32_t j;
  for (j = 0; j < length; ++j) {
    uint32_t i;
    uint32_t byte = data[j];
    crc ^= byte << 8;
    for (i = 0; i < 8; ++i) {
      uint32_t temp = crc << 1;
      if (crc & 0x8000) {
        temp ^= 0x1021;
      }
      crc = temp;
    }
  }
  *current_crc = crc;
}

}  // namespace fitkits_imu
