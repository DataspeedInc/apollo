/**
 * @file ds_usb_can_client.h
 * @brief Defines the DsUsbCanClient class which inherits CanClient.
 */

#pragma once

#include <string>
#include <vector>
#include <queue>

#include "gflags/gflags.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "third_party/can_card_library/ds_usb_can/CanUsb.h"

/**
 * @namespace apollo::drivers::canbus::can
 * @brief apollo::drivers::canbus::can
 */
namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

/**
 * @class DsUsbCanClient
 * @brief The class which defines a Dataspeed USB CAN client which inherits CanClient.
 */
class DsUsbCanClient : public CanClient {
 public:
  /**
   * @brief Initialize the Dataspeed USB CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter &parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~DsUsbCanClient();

  /**
   * @brief Start the Dataspeed USB CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the Dataspeed USB CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame> &frames,
                                 int32_t *const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                    int32_t *const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
  void recvDevice(unsigned int channel, uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8]);

  CANCardParameter::CANChannelId port_;
  std::queue<CanFrame> recv_fifo_;

  // USB Device
  CanUsb *dev_;

  // Parameters
  bool error_topic_;
  std::string mac_addr_;
  struct Filter {
    uint32_t mask;
    uint32_t match;
  };
  struct Channel {
    Channel() : bitrate(0), mode(0) {}
    int bitrate;
    uint8_t mode;
    std::vector<Filter> filters;
  };
  std::vector<Channel> channels_;

};

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
