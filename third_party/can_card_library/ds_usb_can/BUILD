package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ds_usb_can",
    srcs = [
        "UsbDevice.cpp",
        "CanUsb.cpp"
    ],
    linkopts = [
      "-lboost_thread",
      "-lusb-1.0"
    ],
    deps = [
      "//third_party/can_card_library/ds_usb_can:ds_usb_can_headers",
    ],
    include_prefix = "ds_usb_can",
)

cc_library(
    name = "ds_usb_can_headers",
    hdrs = [
      "UsbDevice.h",
      "CanUsb.h",
      "MacAddr.h",
      "UsbCanMessages.h"
    ]
)
