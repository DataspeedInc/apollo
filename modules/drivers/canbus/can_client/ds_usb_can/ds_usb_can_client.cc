/**
 * @file ds_usb_can_client.cc
 * @brief Interface to Dataspeed's USB CAN device
 **/

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
  if (dev_handler_) {
    Stop();
  }
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
    cf.id = recv_frames_[i].id;
    cf.len = recv_frames_[i].len;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].len);
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

/************************************************************************/
/************************************************************************/
/* Function: GetErrorString()                                            */
/* Return ASCII representation of NTCAN return code                     */
/************************************************************************/
/************************************************************************/
const int32_t ERROR_BUF_SIZE = 200;
std::string DsUsbCanClient::GetErrorString(const NTCAN_RESULT ntstatus) {
  struct ERR2STR {
    NTCAN_RESULT ntstatus;
    const char *str;
  };

  int8_t str_buf[ERROR_BUF_SIZE];

  static const struct ERR2STR err2str[] = {
      {NTCAN_SUCCESS, "NTCAN_SUCCESS"},
      {NTCAN_RX_TIMEOUT, "NTCAN_RX_TIMEOUT"},
      {NTCAN_TX_TIMEOUT, "NTCAN_TX_TIMEOUT"},
      {NTCAN_TX_ERROR, "NTCAN_TX_ERROR"},
      {NTCAN_CONTR_OFF_BUS, "NTCAN_CONTR_OFF_BUS"},
      {NTCAN_CONTR_BUSY, "NTCAN_CONTR_BUSY"},
      {NTCAN_CONTR_WARN, "NTCAN_CONTR_WARN"},
      {NTCAN_NO_ID_ENABLED, "NTCAN_NO_ID_ENABLED"},
      {NTCAN_ID_ALREADY_ENABLED, "NTCAN_ID_ALREADY_ENABLED"},
      {NTCAN_ID_NOT_ENABLED, "NTCAN_ID_NOT_ENABLED"},
      {NTCAN_INVALID_FIRMWARE, "NTCAN_INVALID_FIRMWARE"},
      {NTCAN_MESSAGE_LOST, "NTCAN_MESSAGE_LOST"},
      {NTCAN_INVALID_PARAMETER, "NTCAN_INVALID_PARAMETER"},
      {NTCAN_INVALID_HANDLE, "NTCAN_INVALID_HANDLE"},
      {NTCAN_NET_NOT_FOUND, "NTCAN_NET_NOT_FOUND"},
#ifdef NTCAN_IO_INCOMPLETE
      {NTCAN_IO_INCOMPLETE, "NTCAN_IO_INCOMPLETE"},
#endif
#ifdef NTCAN_IO_PENDING
      {NTCAN_IO_PENDING, "NTCAN_IO_PENDING"},
#endif
#ifdef NTCAN_INVALID_HARDWARE
      {NTCAN_INVALID_HARDWARE, "NTCAN_INVALID_HARDWARE"},
#endif
#ifdef NTCAN_PENDING_WRITE
      {NTCAN_PENDING_WRITE, "NTCAN_PENDING_WRITE"},
#endif
#ifdef NTCAN_PENDING_READ
      {NTCAN_PENDING_READ, "NTCAN_PENDING_READ"},
#endif
#ifdef NTCAN_INVALID_DRIVER
      {NTCAN_INVALID_DRIVER, "NTCAN_INVALID_DRIVER"},
#endif
#ifdef NTCAN_OPERATION_ABORTED
      {NTCAN_OPERATION_ABORTED, "NTCAN_OPERATION_ABORTED"},
#endif
#ifdef NTCAN_WRONG_DEVICE_STATE
      {NTCAN_WRONG_DEVICE_STATE, "NTCAN_WRONG_DEVICE_STATE"},
#endif
      {NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"},
#ifdef NTCAN_HANDLE_FORCED_CLOSE
      {NTCAN_HANDLE_FORCED_CLOSE, "NTCAN_HANDLE_FORCED_CLOSE"},
#endif
#ifdef NTCAN_NOT_IMPLEMENTED
      {NTCAN_NOT_IMPLEMENTED, "NTCAN_NOT_IMPLEMENTED"},
#endif
#ifdef NTCAN_NOT_SUPPORTED
      {NTCAN_NOT_SUPPORTED, "NTCAN_NOT_SUPPORTED"},
#endif
#ifdef NTCAN_SOCK_CONN_TIMEOUT
      {NTCAN_SOCK_CONN_TIMEOUT, "NTCAN_SOCK_CONN_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_CMD_TIMEOUT
      {NTCAN_SOCK_CMD_TIMEOUT, "NTCAN_SOCK_CMD_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_HOST_NOT_FOUND
      {NTCAN_SOCK_HOST_NOT_FOUND, "NTCAN_SOCK_HOST_NOT_FOUND"},
#endif
#ifdef NTCAN_CONTR_ERR_PASSIVE
      {NTCAN_CONTR_ERR_PASSIVE, "NTCAN_CONTR_ERR_PASSIVE"},
#endif
#ifdef NTCAN_ERROR_NO_BAUDRATE
      {NTCAN_ERROR_NO_BAUDRATE, "NTCAN_ERROR_NO_BAUDRATE"},
#endif
#ifdef NTCAN_ERROR_LOM
      {NTCAN_ERROR_LOM, "NTCAN_ERROR_LOM"},
#endif
      {(NTCAN_RESULT)0xffffffff, "NTCAN_UNKNOWN"} /* stop-mark */
  };

  const struct ERR2STR *es = err2str;

  do {
    if (es->ntstatus == ntstatus) {
      break;
    }
    es++;
  } while ((uint32_t)es->ntstatus != 0xffffffff);

#ifdef NTCAN_ERROR_FORMAT_LONG
  {
    NTCAN_RESULT res;
    char sz_error_text[60];

    res = canFormatError(ntstatus, NTCAN_ERROR_FORMAT_LONG, sz_error_text,
                         static_cast<uint32_t>(sizeof(sz_error_text) - 1));
    if (NTCAN_SUCCESS == res) {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s - %s",
               es->str, sz_error_text);
    } else {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
               es->str, ntstatus);
    }
  }
#else
  snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
           es->str, ntstatus);
#endif /* of NTCAN_ERROR_FORMAT_LONG */

  return std::string((const char *)(str_buf));
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
