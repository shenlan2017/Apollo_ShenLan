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
        "msg_transfer.cc",
    ],
    hdrs = [
        "front_end.h",
        "back_end.h",
        "post_processing.h",
        "loop_closing.h",
        "msg_transfer.h",
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
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:rtk_config_cc_proto",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//modules/transform:transform_broadcaster",
        "//modules/localization/msf/common/util",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/canbus/proto:chassis_cc_proto",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
        "//modules/common/util:time_util",
    ],
)



cpplint()
