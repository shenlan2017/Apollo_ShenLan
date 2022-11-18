load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "times_utils_cc_proto",
    deps = [
        ":times_utils_proto",
    ],
)

proto_library(
    name = "times_utils_proto",
    srcs = ["times_utils.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "times_utils_py_pb2",
    deps = [
        ":times_utils_proto",
        "//modules/common/proto:header_py_pb2",
    ],
)
