load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pangolin_visualizer_component_lib",
    srcs = [
        "pangolin_visualizer_component.cc",
        "pangolin_visualizer.cc",
    ],
    hdrs = [
        "pangolin_visualizer_component.h",
        "pangolin_visualizer.h",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        # "//modules/common/util",
        # "//modules/common/util:eigen_defs",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/localization/proto:rtk_config_cc_proto",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@opengl",
        "//third_party/pangolin",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_integ",
        "//modules/localization/shenlan_msf/core:pre_integrator",
        "//modules/localization/shenlan_msf/core:registration",
        "//modules/localization/shenlan_msf/core:cloud_filter",
        "//modules/localization/shenlan_msf/core:sensor_data",
        "//modules/localization/shenlan_msf/interface",
    ],
    copts = [
        "-O2",
        "-DMODULE_NAME=\\\"localization_visualizer\\\"",
    ],
)

cc_binary(
    name = "pangolin_visualizer_compenont.so",
    linkshared = True,
    linkstatic = False,
    deps = [":pangolin_visualizer_component_lib"],
)

install(
    name = "install",
    targets = [
        ":pangolin_visualizer_compenont.so",
    ],
)

cpplint()
