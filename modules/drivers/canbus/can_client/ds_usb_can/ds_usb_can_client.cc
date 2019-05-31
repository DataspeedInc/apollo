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
  if (dev_) {
    Stop();
  }
}

ErrorCode DsUsbCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }

  // open device
  dev_ = new CanUsb();
  dev_->setRecvCallback(boost::bind(&DsUsbCanClient::recvDevice, this, _1, _2, _3, _4, _5));
  
  Channel channel;
  channel.mode = 0;
  channel.bitrate = 500000;
  channels_.resize(CanUsb::MAX_CHANNELS, channel);
  
  if (dev_->open(mac_addr_)) {
    if (dev_->reset()) {
      bool success = true;
      for (unsigned int i = 0; i < dev_->numChannels(); i++) {
        for (unsigned int j = 0; j < channels_[i].filters.size(); j++) {
          const uint32_t mask = channels_[i].filters[j].mask;
          const uint32_t match = channels_[i].filters[j].match;
          dev_->addFilter(i, mask, match);
        }
      }
      for (unsigned int i = 0; i < dev_->numChannels(); i++) {
        const int bitrate = i < channels_.size() ? channels_[i].bitrate : 0;
        const uint8_t mode = i < channels_.size() ? channels_[i].mode : 0;
        success &= dev_->setBitrate(i, bitrate, mode);
      }
      if (!success) {
        dev_->reset();
        dev_->closeDevice();
        return ErrorCode::CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED;
      }
    } else {
      dev_->closeDevice();
      return ErrorCode::CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED;
    }
  } else {
    return ErrorCode::CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED;
  }

  is_started_ = true;
  return ErrorCode::OK;
}

void DsUsbCanClient::recvDevice(unsigned int channel, uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8])
{
  // if (channel < pubs_.size()) {
  //   can_msgs::Frame msg;
  //   msg.header.stamp = ros::Time::now();
  //   msg.id = id;
  //   msg.is_rtr = false;
  //   msg.is_extended = extended;
  //   msg.is_error = (dlc == 0x0F);
  //   msg.dlc = dlc;
  //   memcpy(msg.data.elems, data, 8);
  //   if (msg.is_error && (channel < pubs_err_.size())) {
  //     pubs_err_[channel].publish(msg);
  //   } else {
  //     pubs_[channel].publish(msg);
  //   }
  // }
}

void DsUsbCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    dev_->reset();
    delete dev_;
    dev_ = NULL;
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
