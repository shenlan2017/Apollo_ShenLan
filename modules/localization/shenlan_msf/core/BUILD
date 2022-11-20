load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_data",
    srcs = glob([
        "src/sensor_data/**/*.cpp",
        "src/sensor_data/**/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/sensor_data/**/*.hpp",
        "include/lidar_localization/sensor_data/**/*.h",
    ]),
    deps = [
        "@eigen",
        "@local_config_pcl//:pcl",
        "@geographiclib",
        "@com_github_sophus//:sophus",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)


cc_library(
    name = "tools",
    srcs = glob([
        "src/tools/**/*.cpp",
        "src/tools/**/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/tools/*.hpp",
        "include/lidar_localization/tools/*.h",
    ]),
    deps = [
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@com_github_google_glog//:glog"
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)


cc_library(
    name = "graph_optimizer",
    srcs = glob([
        "src/models/graph_optimizer/**/*.cpp",
        "src/models/graph_optimizer/**/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/graph_optimizer/**/*.hpp",
        "include/lidar_localization/models/graph_optimizer/**/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)

cc_library(
    name = "pre_integrator",
    srcs = glob([
        "src/models/pre_integrator/*.cpp",
        "src/models/pre_integrator/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/pre_integrator/*.hpp",
        "include/lidar_localization/models/pre_integrator/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        ":graph_optimizer",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)

cc_library(
    name = "registration",
    srcs = glob([
        "src/models/registration/*.cpp",
        "src/models/registration/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/registration/*.hpp",
        "include/lidar_localization/models/registration/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    copts = [
        "-fopenmp",
        # "-march=native",
    ],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)

cc_library(
    name = "cloud_filter",
    srcs = glob([
        "src/models/cloud_filter/*.cpp",
        "src/models/cloud_filter/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/cloud_filter/*.hpp",
        "include/lidar_localization/models/cloud_filter/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)


cc_library(
    name = "scan_adjust",
    srcs = glob([
        "src/models/scan_adjust/**/*.cpp",
        "src/models/scan_adjust/**/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/scan_adjust/**/*.hpp",
        "include/lidar_localization/models/scan_adjust/**/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        ":graph_optimizer",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)

cc_library(
    name = "kalman_filter",
    srcs = glob([
        "src/models/kalman_filter/*.cpp",
        "src/models/kalman_filter/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/kalman_filter/*.hpp",
        "include/lidar_localization/models/kalman_filter/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)


cc_library(
    name = "sliding_window",
    srcs = glob([
        "src/models/sliding_window/**/*.cpp",
        "src/models/sliding_window/**/*.cc",
    ]),
    hdrs = glob([
        "include/lidar_localization/models/sliding_window/**/*.hpp",
        "include/lidar_localization/models/sliding_window/**/*.h",
    ]),
    deps = [
        ":sensor_data",
        ":tools",
        ":graph_optimizer",
        "@boost",
        "@eigen",
        "@geographiclib",
        "@com_google_ceres//:ceres",
        "@com_github_google_glog//:glog",
        "@com_github_sophus//:sophus",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "//third_party/g2o",
    ],
    includes = ["include/"],
    strip_include_prefix = "include/",
    include_prefix = "modules/localization/shenlan_msf/core/",
)

# cc_library(
#     name = "models",
#     srcs = glob([
#         "src/models/**/*.cpp",
#         "src/models/**/*.cc",
#     ]),
#     hdrs = glob([
#         "include/lidar_localization/models/**/*.hpp",
#         "include/lidar_localization/models/**/*.h",
#     ]),
#     deps = [
#         "//cyber",
#         "//cyber/time:clock",
#         "//cyber/common:time_conversion",
#         "//modules/common/adapters:adapter_gflags",
#         "//modules/common/math",
#         "//modules/common/monitor_log",
#         "//modules/common/proto:geometry_cc_proto",
#         "//modules/common/status",
#         "//modules/common/util",
#         ":sensor_data",
#         ":tools",
#         "@boost",
#         "@eigen",
#         "@geographiclib",
#         "@com_google_ceres//:ceres",
#         "@com_github_google_glog//:glog",
#         "@com_github_sophus//:sophus",
#         "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#         "@local_config_pcl//:pcl",
#         "//third_party/g2o",
#     ],
#     copts = [
#         "-fopenmp",
#         # "-march=native",
#     ],
#     includes = ["include/"],
#     strip_include_prefix = "include/",
#     include_prefix = "modules/localization/shenlan_msf/core/",
# )

cpplint()
