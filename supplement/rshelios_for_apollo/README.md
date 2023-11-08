1.put "lidar" into "apollo/modules/drivers""

2.put "rs_driver" into "apollo/third_party"

3.add below into "apollo/tools/workspace.bzl"

load("//third_party/rs_driver:workspace.bzl", rs_driver = "repo")
rs_driver()
