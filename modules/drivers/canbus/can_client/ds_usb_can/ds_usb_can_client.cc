/**
 * @file ds_usb_can_client.cc
 * @brief Interface to Dataspeed's USB CAN device
 **/

#include "modules/drivers/canbus/can_client/ds_usb_can/ds_usb_can_client.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/sensor_gflags.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool DsUsbCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();
  return true;
}

DsUsbCanClient::~DsUsbCanClient() {
  Stop();
}

ErrorCode DsUsbCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }

  // open device
  // return ErrorCode::CAN_CLIENT_ERROR_BASE;

  is_started_ = true;
  return ErrorCode::OK;
}

void DsUsbCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    // Close here!
  }
}

// Synchronous transmission of CAN messages
ErrorCode DsUsbCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "Dataspeed CAN client is not initialized";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }

  // Send CAN frames here
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode DsUsbCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Dataspeed CAN client is not initialized";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  // TODO: Read CAN frames here

  // TODO: Send CAN messages to Apollo here
  for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
