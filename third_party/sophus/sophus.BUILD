load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sophus",
    srcs = [],
    includes = ['.'],
    hdrs = glob([
        "sophus/*.hpp",
    ]),
    deps = [
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)
