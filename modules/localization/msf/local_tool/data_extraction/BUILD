load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "modules/localization/msf/local_tool/data_extraction",
    targets = [
        ":compare_poses",
        ":cyber_record_parser",
    ],
)

cc_library(
    name = "data_extraction",
    srcs = [
        "cyber_record_reader.cc",
        "location_exporter.cc",
        "pcd_exporter.cc",
    ],
    hdrs = [
        "cyber_record_reader.h",
        "location_exporter.h",
        "pcd_exporter.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

cc_binary(
    name = "cyber_record_parser",
    srcs = ["cyber_record_parser.cc"],
    linkstatic = 0,
    deps = [
        ":data_extraction",
        "//cyber",
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

cc_binary(
    name = "compare_poses",
    srcs = ["compare_poses.cc"],
    deps = [
        ":data_extraction",
        "//cyber",
        "@boost",
        "@eigen",
    ],
)

cpplint()
