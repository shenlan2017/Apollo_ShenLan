"""Loads the sophus library"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "com_github_sophus",
<<<<<<< HEAD
=======
        commit = "8e1941677ef214aab0397608f42b18fbe1a19885",
        remote = "https://github.com/strasdat/Sophus.git",
>>>>>>> 4bafe17af7eb7de8cddaec4de8780d18732ee92c
        build_file = clean_dep("//third_party/sophus:sophus.BUILD"),
        path = "/apollo/third_party/sophus/com_github_sophus/",
    )

