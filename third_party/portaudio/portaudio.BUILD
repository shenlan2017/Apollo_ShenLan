load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "portaudio",
    includes = ["."],
    linkopts = [
        "-lrt",
        "-lasound",
        "-ljack",
        "-lpthread",
        "-lportaudio",
    ],
)
