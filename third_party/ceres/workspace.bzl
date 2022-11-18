"""Loads the ceres library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    # ceres 1.14.0
    http_archive(
        name = "com_google_ceres",
        sha256 = "1296330fcf1e09e6c2f926301916f64d4a4c5c0ff12d460a9bc5d4c48411518f",
        strip_prefix = "ceres-solver-1.14.0",
        urls = [
            "https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz",
        ],
        build_file = clean_dep("//third_party/ceres:ceres.BUILD"),
    )


