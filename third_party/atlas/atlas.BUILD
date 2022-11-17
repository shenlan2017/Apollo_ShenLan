load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "atlas",
    includes = [
        ".",
    ],
    linkopts = [
        "-latlas",
    ],
    linkstatic = False,
)

cc_library(
    name = "blas",
    includes = [
        ".",
    ],
    linkopts = [
        "-lblas",
    ],
    linkstatic = False,
)

cc_library(
    name = "cblas",
    includes = [
        ".",
    ],
    linkopts = [
        "-lcblas",
    ],
    linkstatic = False,
)

cc_library(
    name = "lapack",
    includes = [
        ".",
    ],
    linkopts = [
        "-llapack",
    ],
    linkstatic = False,
)
