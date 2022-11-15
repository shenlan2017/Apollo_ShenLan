load("@rules_cc//cc:defs.bzl", "cc_library")
licenses(["notice"])
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geographiclib",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/GeographicLib/*.hpp",
        "include/GeographicLib/*.h",
        "src/*.hh",
    ]),
     includes = [
       "include",
    ],
    deps = [
        "@boost",
    ],
    visibility = ["//visibility:public"],
)

