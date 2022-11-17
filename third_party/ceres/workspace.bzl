"""Loads the ceres library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "com_google_ceres",
        build_file = clean_dep("//third_party/ceres:ceres.BUILD"),
        path = "/apollo/third_party/ceres/com_google_ceres/",
    )

