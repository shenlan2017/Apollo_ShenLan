load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "**/*.yaml",
        "*.yaml",
    ]),
)
