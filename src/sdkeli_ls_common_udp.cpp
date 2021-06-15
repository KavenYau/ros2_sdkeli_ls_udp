#include <sdkeli_ls_udp/sdkeli_ls_common_udp.h>
#include <algorithm>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <iterator>

namespace sdkeli_ls_udp {

CSDKeliLsCommonUdp::CSDKeliLsCommonUdp(const std::string &hostname, int32_t port, int32_t timelimit,
                                       std::shared_ptr<CParserBase> parser, rclcpp::Node::SharedPtr node_ptr, SDKeliLsConfig config)
    : CSDKeliLsCommon(parser, node_ptr, config), mHostName(hostname), mPort(port), mTimeLimit(timelimit) {}

CSDKeliLsCommonUdp::~CSDKeliLsCommonUdp() {
  StopScanner();
  CloseDevice();
}

int CSDKeliLsCommonUdp::InitDevice() {
  int opt = 1;
  struct sockaddr_in cliAddr;
  struct hostent *h;
  mSocket = -1;
  mSocket = socket(AF_INET, SOCK_DGRAM, 0);
  if (mSocket < 0) {
    RCLCPP_ERROR(logger_, "create udp socket failed");
    return ExitError;
  }

  h = gethostbyname(mHostName.c_str());
  if (h == NULL) {
    RCLCPP_ERROR(logger_, "unknown host '%s' /n", mHostName.c_str());
    return ExitError;
  }
  RCLCPP_INFO(logger_, "sending data to '%s' (IP : %s) (PORT : %d)", h->h_name,
              inet_ntoa(*(struct in_addr *)h->h_addr_list[0]), mPort);
  remoteServAddr.sin_family = h->h_addrtype;
  memcpy((char *)&remoteServAddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
  remoteServAddr.sin_port = htons(mPort);
  /*
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr(mHostName.c_str());
  addr.sin_port = htons(std::stoi(mPort));
  */

  bzero(&cliAddr, sizeof(cliAddr));
  cliAddr.sin_family = AF_INET;
  cliAddr.sin_addr.s_addr = INADDR_ANY;
  // cliAddr.sin_port = htons(atoi(mPort.c_str()));
  cliAddr.sin_port = 0;
  setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
  if (bind(mSocket, (struct sockaddr *)&cliAddr, sizeof(cliAddr)) < 0) {
    RCLCPP_ERROR(logger_, "bind socket failed");
    close(mSocket);
    mSocket = -1;
    return ExitError;
  }

  ClearConnectFlag();

  return ExitSuccess;
}

int CSDKeliLsCommonUdp::CloseDevice() {
  if (mSocket != -1) {
    close(mSocket);
    mSocket = -1;
    RCLCPP_ERROR(logger_, "close socket and CloseDevice");
  }
}

int CSDKeliLsCommonUdp::SendUdpData2Device(uint8_t *buf, int32_t length) {
  int n = -1;
  if (mSocket > 0) {
    n = sendto(mSocket, buf, length, 0, (struct sockaddr *)&remoteServAddr, sizeof(remoteServAddr));
  }
  return n;
}

int CSDKeliLsCommonUdp::SendDeviceReq(const uint8_t *req, std::vector<uint8_t> *resp) {
  if (mSocket == -1) {
    RCLCPP_ERROR(logger_, "SendDeviceReq: Socket NOT open");
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SDKELI_LS - SendDeviceReq: socket NOT open!");

    return ExitError;
  }

  if (SendUdpData2Device((uint8_t *)req, SIZE_OF_CMD) != SIZE_OF_CMD) {
    RCLCPP_ERROR(logger_, "Write error for req command");
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "SDKELI_LS - SendDeviceReq: Write command failed!");

    return ExitError;
  }
  return ExitSuccess;
}

int CSDKeliLsCommonUdp::GetDataGram(uint8_t *receiveBuffer, int32_t bufferSize, int32_t *length) {
  int len;
  struct timeval tv;
  socklen_t addrlen;
  fd_set rfds;
  struct sockaddr_in recvAddr;
  if (mSocket == -1) {
    RCLCPP_ERROR(logger_, "GetDataGram: Socket NOT open");
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SDKELI_LS - GetDataGram: socket NOT open!");

    return ExitError;
  }
  addrlen = sizeof(recvAddr);

  if (1)  //(!stream_stopped_)
  {
    tv.tv_sec = mTimeLimit;
    tv.tv_usec = 0;
    FD_ZERO(&rfds);
    FD_SET(mSocket, &rfds);
    if (select(mSocket + 1, &rfds, NULL, NULL, &tv) > 0) {
      *length = recvfrom(mSocket, receiveBuffer, bufferSize, 0, (struct sockaddr *)&recvAddr, &addrlen);
      if (len > 0) {
        //    printf("echo from %s:UDP%u/n", inet_ntoa(recvAddr.sin_addr),ntohs(recvAddr.sin_port));
      }
    } else {
      std::string errorStr = "SDKELI_LS - GetDataGram: No full response for read after " + mTimeLimit;
      errorStr = errorStr + "S";
    //   mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, errorStr);
      RCLCPP_WARN(logger_, "GetDataGram timeout for %ds", mTimeLimit);
      return ExitError;
    }
  }

  return ExitSuccess;
}
} /*sdkeli_ls_udp*/
