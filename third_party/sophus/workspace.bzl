"""Loads the sophus library"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    # sophus 1.0
    new_git_repository(
        name = "com_github_sophus",
        commit = "8e1941677ef214aab0397608f42b18fbe1a19885",
        remote = "https://github.com/strasdat/Sophus.git",
        build_file = clean_dep("//third_party/sophus:sophus.BUILD"),
    )

