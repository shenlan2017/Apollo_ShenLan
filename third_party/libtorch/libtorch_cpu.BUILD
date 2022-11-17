load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_cpu",
    includes = [
        ".",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-L/usr/local/libtorch_cpu/lib",
        "-lc10",
        "-ltorch",
        "-ltorch_cpu",
    ],
    linkstatic = False,
    deps = [
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)
