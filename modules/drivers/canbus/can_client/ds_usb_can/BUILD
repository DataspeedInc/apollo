load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ds_usb_can_client",
    srcs = [
        "ds_usb_can_client.cc"
    ],
    hdrs = [
        "ds_usb_can_client.h"
    ],
    deps = [
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client",
        "//third_party/can_card_library/ds_usb_can:ds_usb_can_headers",
    ],
)

cpplint()
