load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "example",
    srcs = [
        "example_lib/libexample.so",
    ],
    hdrs = [
        "include/example.hpp",
    ],
    include_prefix = "example_third_party",
    strip_include_prefix = "include",
)
