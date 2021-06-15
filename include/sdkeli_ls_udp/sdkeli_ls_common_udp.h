#ifndef SDKELI_LS_COMMON_UDP__
#define SDKELI_LS_COMMON_UDP__

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rclcpp/logging.hpp>
#include "sdkeli_ls_common.h"

namespace sdkeli_ls_udp {
class CSDKeliLsCommonUdp : public CSDKeliLsCommon {
 public:
  CSDKeliLsCommonUdp(const std::string &hostname, int32_t port, int32_t timelimit, std::shared_ptr<CParserBase> parser,
                     rclcpp::Node::SharedPtr node_ptr, SDKeliLsConfig config);
  virtual ~CSDKeliLsCommonUdp();

 protected:
  /*Override functions*/
  virtual int32_t InitDevice();
  virtual int32_t CloseDevice();

  virtual int32_t SendDeviceReq(const uint8_t *req, std::vector<uint8_t> *resp);

  virtual int32_t GetDataGram(uint8_t *receiveBuffer, int32_t bufferSize, int32_t *length);

  int32_t SendUdpData2Device(uint8_t *buf, int32_t length);

 private:
  int32_t mSocket;
  size_t mBytesReceived;
  std::string mHostName;
  int32_t mPort;
  int32_t mTimeLimit;
  struct sockaddr_in remoteServAddr;

};

} /*namespace sdkeli_ls_udp*/

#endif /*SDKELI_LS_COMMON_TCP__*/
