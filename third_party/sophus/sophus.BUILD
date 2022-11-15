load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sophus",
    hdrs = [
        "sophus/average.hpp",
        "sophus/cartesian.hpp",
        "sophus/ceres_local_parameterization.hpp",
        "sophus/ceres_manifold.hpp",
        "sophus/ceres_typetraits.hpp",
        "sophus/common.hpp",
        "sophus/geometry.hpp",
        "sophus/interpolate.hpp",
        "sophus/interpolate_details.hpp",
        "sophus/num_diff.hpp",
        "sophus/rotation_matrix.hpp",
        "sophus/rxso2.hpp",
        "sophus/rxso3.hpp",
        "sophus/se2.hpp",
        "sophus/se3.hpp",
        "sophus/sim2.hpp",
        "sophus/sim3.hpp",
        "sophus/sim_details.hpp",
        "sophus/so2.hpp",
        "sophus/so3.hpp",
        "sophus/spline.hpp",
        "sophus/types.hpp",
        "sophus/velocities.hpp",
    ],
    deps = [
        "@eigen",
    ],
    includes = [".",],
    copts = [
        # "-march=native",
        "-DSOPHUS_USE_BASIC_LOGGING",
    ],
    visibility = ["//visibility:public"],
)
