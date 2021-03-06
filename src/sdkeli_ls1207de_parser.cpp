#include <sdkeli_ls_udp/sdkeli_ls1207de_parser.h>
#include <sdkeli_ls_udp/sdkeli_ls_sensor_frame.h>
// #include <ros/ros.h>
#include <stdio.h>
#include <cmath>

namespace sdkeli_ls_udp {

rclcpp::Clock g_clock = rclcpp::Clock(RCL_ROS_TIME);

CSDKeliLs1207DEParser::CSDKeliLs1207DEParser(const std::string &frame_id)
    : CParserBase(),
      fRangeMin(0.05),
      fRangeMax(10.0),
      fTimeIncrement(-1.0),
      fFrame_id(frame_id),
      logger_(rclcpp::get_logger("LS1207DE_parser")) {
  // Do Nothing...
}

CSDKeliLs1207DEParser::~CSDKeliLs1207DEParser() {
  // Do Nothing...
}

int CSDKeliLs1207DEParser::Parse(char *data, size_t data_length, SDKeliLsConfig &config,
                                 sensor_msgs::msg::LaserScan &msg) {
  CSDKeliLsSensFrame *pSensFrame = new CSDKeliLsSensFrame();
  if (!pSensFrame->InitFromSensBuff(data, data_length)) {
    RCLCPP_INFO(logger_, "Invalid frame data!");
    return ExitSuccess;
  }

  int dataCount = pSensFrame->GetSensDataCount();

  /*Fill sensor message struct*/
  // msg.header.frame_id = config.frame_id;
  msg.header.frame_id = fFrame_id;
  // ROS_DEBUG("Publishing with frame id: %s", config.frame_id.c_str());
  RCLCPP_DEBUG(logger_, "Publishing with frame id: %s", fFrame_id.c_str());

  /*1: Scan time: The time for every frame.*/
  rclcpp::Time start_time = g_clock.now();

  // ros::Time start_time = ros::Time::now();
  unsigned short scanning_freq = 1000 / 43 * 100; /*For dev borad, the device will send data every 50ms*/
  msg.scan_time = 1.0 / (scanning_freq / 100.0);
  RCLCPP_DEBUG(logger_, "scanning freq: %d, scan_time: %f", scanning_freq, msg.scan_time);

  /*2: Time increment: Time interval for between each data.*/
  /*Time increment has been overriden*/
  fTimeIncrement = 0.000040;
  msg.time_increment = fTimeIncrement;
  RCLCPP_DEBUG(logger_, "time_increment: %f", msg.time_increment);

  /*3: Angle Min: Starting angle of current scanning data.*/
  int starting_angle = 0xFFF92230; /*This value is from Sick Tim*/
  msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
  RCLCPP_DEBUG(logger_, "starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

  /*4: Angle step width: anguler between each scanning data.*/
  unsigned short angular_step_width = 0xD05; /*3333: This value is from Sick Tim*/
  msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;

  /*5: Angle Max: Ending angle of current scanning data.*/
  msg.angle_max = msg.angle_min + (dataCount - 1) * msg.angle_increment;

  /* calculate statring data index and adjust angle_min to min_ang config param */
  int index_min = 0;
  while (msg.angle_min + msg.angle_increment < config.min_ang) {
    msg.angle_min += msg.angle_increment;
    index_min++;
  }
  RCLCPP_DEBUG(logger_, "index_min: %d, angle_min: %f", index_min, msg.angle_min);

  /* calculate ending data index and adjust angle_max to max_ang config param */
  int index_max = dataCount - 1;
  while (msg.angle_max - msg.angle_increment > config.max_ang) {
    msg.angle_max -= msg.angle_increment;
    index_max--;
  }
  RCLCPP_DEBUG(logger_, "index_max: %i, angle_max: %f", index_max, msg.angle_max);

  /*5: Fill data range*/
  msg.ranges.resize(index_max - index_min + 1);
  msg.ranges.assign(index_max - index_min + 1, std::numeric_limits<double>::infinity());
  RCLCPP_DEBUG(logger_, "Fill sensor data. index_min = %d, index_max = %d.", index_min, index_max);
  for (int j = index_min; j <= index_max; ++j) {
    if (config.debug_mode) {
      if ((j - index_min + 1) % 48 == 0) {
        printf("\n");
      }
    }

    unsigned short range = 0;
    if (!config.inverse) {
      range = pSensFrame->GetSensDataOfIndex(j);
    } else {
      range = pSensFrame->GetSensDataOfIndex(pSensFrame->GetRangeEnd() - j);
    }
    /*unsigned short range = 3000; */ /*For testing....*/
    //  ROS_WARN("fRangeMax=%f,fRangeMin=%f",fRangeMax,fRangeMin);
    float meter_value = range / 1000.0;
    if (meter_value > fRangeMin && meter_value < fRangeMax) {
      msg.ranges[j - index_min] = meter_value;
    }

    if (config.debug_mode) {
      printf("%.2f ", msg.ranges[j - index_min]);
    }
  }
  if (config.debug_mode) {
    printf("\n");
  }

  if (config.intensity && data_length == dataCount * 4) {
    msg.intensities.resize(index_max - index_min + 1);
    for (int j = index_min; j <= index_max; ++j) {
      unsigned short intensity = 0;
      if (!config.inverse) {
        intensity = pSensFrame->GetSensIntensityOfIndex(j);
      } else {
        intensity = pSensFrame->GetSensIntensityOfIndex(pSensFrame->GetRangeEnd() - j);
      }

      if (intensity > 55000) {
        intensity = 600;
      }

      if (intensity > 5000) {
        intensity = 200 + (intensity - 5000) / 1200;

      } else {
        intensity = intensity / 25;
      }

      msg.intensities[j - index_min] = intensity;
    }
  }

  /*Override range*/
  msg.range_min = fRangeMin;
  msg.range_max = fRangeMax;

  /*6: Setting starting time*/
  /* - last scan point = now ==> first scan point = now - data count * time increment*/
  // msg.header.stamp = start_time - ros::Duration().fromSec(dataCount * msg.time_increment);
  rclcpp::Time cal_time = start_time - rclcpp::Duration::from_seconds(dataCount * msg.time_increment);
  msg.header.stamp = cal_time;
  /* - shit forward to time of first published scan point*/
  // msg.header.stamp += ros::Duration().fromSec((double)index_min * msg.time_increment);
  cal_time = cal_time + rclcpp::Duration::from_seconds((double)index_min * msg.time_increment);
  msg.header.stamp = cal_time;

  /* - add time offset (to account for USB latency etc.)*/
  //   msg.header.stamp += ros::Duration().fromSec(config.time_offset);
  cal_time = cal_time + rclcpp::Duration::from_seconds(config.time_offset);
  msg.header.stamp = cal_time;

  /*Consistency Check*/
  float expected_time_increment = msg.scan_time * msg.angle_increment / (2.0 * M_PI);
  if (fabs(expected_time_increment - msg.time_increment) > 0.00001) {
    RCLCPP_DEBUG_THROTTLE(
        logger_, g_clock, 60,
        "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
        "Expected time_increment: %.9f, reported time_increment: %.9f. "
        "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 "
        "seconds.",
        expected_time_increment, msg.time_increment);
  }

  if (pSensFrame) {
    delete pSensFrame;
  }

  return ExitSuccess;
}

void CSDKeliLs1207DEParser::SetRangeMin(float minRange) { fRangeMin = minRange; }

void CSDKeliLs1207DEParser::SetRangeMax(float maxRange) { fRangeMax = maxRange; }

void CSDKeliLs1207DEParser::SetTimeIncrement(float time) { fTimeIncrement = time; }

void CSDKeliLs1207DEParser::SetFrameId(std::string frame_id) { fFrame_id = frame_id; }

} /*namespace sdkeli_ls_udp*/
