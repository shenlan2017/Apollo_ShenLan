"""Loads the sophus library"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    # sophus 1.0
    new_git_repository(
        name = "com_github_sophus",
        commit = "db218a249202fe63ac13248b5f565b0d385f6640",
        remote = "https://gitee.com/xlzhknight/Sophus.git",
        build_file = clean_dep("//third_party/sophus:sophus.BUILD"),
    )

