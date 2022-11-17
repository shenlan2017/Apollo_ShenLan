load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libshenlan_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":shenlan_msf_front_end_component_lib",
        ":shenlan_msf_back_end_component_lib",
        ":shenlan_msf_loop_closing_component_lib",
    ],
)

cc_library(
    name = "shenlan_msf_front_end_component_lib",
    copts = [
        '-DMODULE_NAME=\\"localization\\"',
    ],
    deps = [
        ":shenlan_front_end_component",
    ],
    alwayslink = True,
)


cc_library(
    name = "shenlan_msf_back_end_component_lib",
    copts = [
        '-DMODULE_NAME=\\"localization\\"',
    ],
    deps = [
        ":shenlan_back_end_component",
    ],
    alwayslink = True,
)

cc_library(
    name = "shenlan_msf_loop_closing_component_lib",
    copts = [
        '-DMODULE_NAME=\\"localization\\"',
    ],
    deps = [
        ":shenlan_loop_closing_component",
    ],
    alwayslink = True,
)

install(
    name = "install",
    targets = [
        ":libshenlan_localization_component.so",
    ],
)


cc_library(
    name = "shenlan_front_end_component",
    srcs = [
        "shenlan_front_end_component.cc",
    ],
    hdrs = [
        "shenlan_front_end_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
        "//modules/common/util:time_util",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//modules/transform:transform_broadcaster",
        "//modules/localization/msf/common/util",
        # "//modules/localization/msf/local_integ",
        "//modules/localization/shenlan_msf/core:pre_integrator",
        "//modules/localization/shenlan_msf/core:registration",
        "//modules/localization/shenlan_msf/core:cloud_filter",
        "//modules/localization/shenlan_msf/core:sensor_data",
        "//modules/localization/shenlan_msf/core:sliding_window",
        "//modules/localization/shenlan_msf/core:graph_optimizer",
        "//modules/localization/shenlan_msf/interface",
        "@boost",
        "@geographiclib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@com_github_sophus//:sophus",
        "@com_google_ceres//:ceres",
        "//third_party/g2o",
    ],
)

cc_library(
    name = "shenlan_back_end_component",
    srcs = [
        "shenlan_back_end_component.cc",
    ],
    hdrs = [
        "shenlan_back_end_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
        "//modules/common/util:time_util",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/canbus/proto:chassis_cc_proto",
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
        "//modules/localization/shenlan_msf/core:pre_integrator",
        "//modules/localization/shenlan_msf/core:registration",
        "//modules/localization/shenlan_msf/core:cloud_filter",
        "//modules/localization/shenlan_msf/core:sensor_data",
        "//modules/localization/shenlan_msf/core:sliding_window",
        "//modules/localization/shenlan_msf/core:graph_optimizer",
        "//modules/localization/shenlan_msf/interface",
        "@boost",
        "@geographiclib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@com_github_sophus//:sophus",
        "@com_google_ceres//:ceres",
        "//third_party/g2o",
    ],
)

cc_library(
    name = "shenlan_loop_closing_component",
    srcs = [
        "shenlan_loop_closing_component.cc",
    ],
    hdrs = [
        "shenlan_loop_closing_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//cyber/common:time_conversion",
        "//modules/common/util:time_util",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/canbus/proto:chassis_cc_proto",
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
        "//modules/localization/shenlan_msf/core:pre_integrator",
        "//modules/localization/shenlan_msf/core:registration",
        "//modules/localization/shenlan_msf/core:cloud_filter",
        "//modules/localization/shenlan_msf/core:sensor_data",
        "//modules/localization/shenlan_msf/core:sliding_window",
        "//modules/localization/shenlan_msf/core:graph_optimizer",
        "//modules/localization/shenlan_msf/interface",
        "@boost",
        "@geographiclib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@com_github_sophus//:sophus",
        "@com_google_ceres//:ceres",
        "//third_party/g2o",
    ],
)
cpplint()
