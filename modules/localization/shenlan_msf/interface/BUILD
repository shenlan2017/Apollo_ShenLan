load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    srcs = [
        "front_end.cc",
        "back_end.cc",
        "post_processing.cc",
        "loop_closing.cc",
    ],
    hdrs = [
        "front_end.h",
        "back_end.h",
        "post_processing.h",
        "loop_closing.h",
    ],
    deps = [
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
        "//modules/localization/shenlan_msf/core:pre_integrator",
        "//modules/localization/shenlan_msf/core:registration",
        "//modules/localization/shenlan_msf/core:cloud_filter",
        "//modules/localization/shenlan_msf/core:sensor_data",
        "//modules/localization/shenlan_msf/core:sliding_window",
        "//modules/localization/shenlan_msf/core:graph_optimizer",
    ],
)

cpplint()
