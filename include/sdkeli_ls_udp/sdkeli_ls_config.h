#ifndef SDKELI_LS_CONFIG_H__
#define SDKELI_LS_CONFIG_H__

namespace sdkeli_ls_udp {

struct SDKeliLsConfig {
  double min_ang;
  double max_ang;
  double time_offset;
  int skip;
  bool intensity;
  bool debug_mode;
  bool auto_reboot;
  bool inverse;
};
}

#endif  // SDKELI_LS_CONFIG_H__