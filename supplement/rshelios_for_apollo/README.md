put lidar/ into apollo/modules/drivers/

put third_party/ into apollo/third_party

add line in apollo/tools/workspace.bzl/ & function initialize_third_party()
load("//third_party/rs_driver:workspace.bzl", rs_driver = "repo")
rs_driver()
