load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "dump_message",
    srcs = ["dump_message.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/drivers/proto:pointcloud_py_pb2",
        "//modules/drivers/proto:conti_radar_py_pb2",
        "//modules/drivers/proto:sensor_image_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
    ],
)


py_binary(
    name = "dump_message_all",
    srcs = ["dump_message_all.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/drivers/proto:pointcloud_py_pb2",
        "//modules/drivers/proto:conti_radar_py_pb2",
        "//modules/drivers/proto:sensor_image_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/drivers/gnss/proto:ins_py_pb2",
        "//modules/drivers/gnss/proto:imu_py_pb2",
        "//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
        "//modules/drivers/gnss/proto:heading_py_pb2",
    ],
)


py_binary(
    name = "perception_obstacle_sender",
    srcs = ["perception_obstacle_sender.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/perception/proto:perception_obstacle_py_pb2",
    ],
)
