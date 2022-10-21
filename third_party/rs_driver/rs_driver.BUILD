load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# TODO(all): May use rules_boost.
cc_library(
    name = "rs_driver",
    includes = [".",],
    hdrs = glob([
        "rs_driver/*.hpp",
        "rs_driver/driver/*.hpp",
        "rs_driver/driver/decoder/*.hpp",
        "rs_driver/driver/input/*.hpp",
        "rs_driver/driver/input/win/*.hpp",
        "rs_driver/driver/input/unix/*.hpp",
        "rs_driver/msg/*.hpp",
        "rs_driver/common/*.hpp",
        "rs_driver/api/*.hpp",
        "rs_driver/utility/*.hpp",
        "rs_driver/macro/*.hpp",
    ]), 
)
