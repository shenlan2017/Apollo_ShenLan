load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "example_lib_test",
    srcs = [
        "example_test.cpp",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//third_party/example",
    ],
)

cc_library(
    name = "example_lib_ceres_test",
    srcs = [
        "ceres_test.cpp",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "@com_google_ceres//:ceres",
        "@opencv//:core",
    ],
)

cc_library(
    name = "example_lib_sophus_test",
    srcs = [
        "sophus_test.cpp",
    ],
    hdrs = glob(["*.hpp"]),
    deps = [
        "@com_github_sophus//:sophus",
    ],
)

cc_library(
    name = "example_lib_proto_test",
    srcs = [
        "proto_test.cpp",
    ],
    hdrs = glob(["*.hpp"]),
    deps = [
        "//modules/example/proto:times_utils_cc_proto",
        "//cyber",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
    ],
)



cc_binary(
    name = "example_test",
    linkstatic = 0,
    deps = [":example_lib_test"],
)

cc_binary(
    name = "example_ceres_test",
    linkstatic = 0,
    deps = [":example_lib_ceres_test"],
)

cc_binary(
    name = "example_sopuhs_test",
    linkstatic = 0,
    deps = [":example_lib_sophus_test"],
)

cc_binary(
    name = "example_proto_test",
    linkstatic = 0,
    deps = [":example_lib_proto_test"],
)

cpplint()
