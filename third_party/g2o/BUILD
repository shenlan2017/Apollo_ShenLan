load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])
licenses(["notice"])

cc_library(
    name = "g2o_csparse",
    srcs = glob([
        "lib/**/*.so",
        "g2o/EXTERNAL/csparse/*.cpp",
        "g2o/EXTERNAL/csparse/*.c",
    ]),
    includes = ["g2o/EXTERNAL/csparse",
    ],
    hdrs = glob([
        "g2o/g2o/config.h",
        "g2o/EXTERNAL/csparse/*.h",
        "g2o/EXTERNAL/csparse/*.hpp",
    ]),

    deps = [
        "@eigen",
        "@atlas//:blas",
        "@atlas//:lapack"
    ],
    # copts = [
    #     "-march=native",
    # ],
)


cc_library(
    name = "g2o_cholmod",
    srcs = glob([
        "lib/cholmod/*.so",
    ]),
    hdrs = glob([
        "lib/cholmod/include/*.h",
    ]),
    deps = [
        "@eigen",
        "@atlas//:blas",
        "@atlas//:lapack"
    ],
    includes = [
        "lib/cholmod/include",
    ],
    # copts = [
    #     "-march=native",
    # ],
    strip_include_prefix = "lib/cholmod/include",       # 去除冗余的前缀
)

cc_library(
    name = "g2o",
    srcs = glob([
        "lib/**/*.so",
        "g2o/g2o/core/*.cpp",
        "g2o/g2o/stuff/*.cpp",
        "g2o/g2o/stuff/*.cpp",
        "g2o/g2o/solvers/**/*.cpp",
        "g2o/g2o/types/**/*.cpp",
        ],
        exclude=[
            "g2o/g2o/stuff/opengl_primitives.cpp",
            "g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp",
            "g2o/g2o/types/slam3d/test_isometry3d_mappings.cpp",
            "g2o/g2o/types/slam3d/test_mat2quat_jacobian.cpp",
            "g2o/g2o/types/slam3d/test_slam3d_jacobian.cpp",
            "g2o/g2o/types/slam3d_addons/line3d_test.cpp",
        ],
    ),
    includes = ["g2o/EXTERNAL/EXTERNAL",
                "g2o/g2o",
    ],
    hdrs = glob([
        "g2o/g2o/**/*.h",
        "g2o/g2o/**/*.hpp",
        "g2o/EXTERNAL/**/*.h",
        "g2o/EXTERNAL/**/*.hpp",
        ],
        exclude=["g2o/g2o/stuff/opengl_primitives.h",],
    ),
    deps = [
        "@eigen",
        "@atlas//:blas",
        "@atlas//:lapack",
        ":g2o_csparse",
        ":g2o_cholmod",
    ],
    # copts = [
    #     "-march=native",
    # ],
    strip_include_prefix = "g2o",       # 去除冗余的前缀
)
