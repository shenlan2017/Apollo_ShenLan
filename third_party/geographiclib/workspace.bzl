"""Loads the sophus library"""

# load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "geographiclib",
        build_file = clean_dep("//third_party/geographiclib:geographiclib.BUILD"),
        path = "/apollo/third_party/geographiclib/geographiclib/",
        # path = "/apollo/geographiclib/",
        # workspace_file_content = "",
    )
