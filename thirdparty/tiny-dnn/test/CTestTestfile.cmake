# CMake generated Testfile for 
# Source directory: /home/weiming/ubuntu-project/tiny-dnn/test
# Build directory: /home/weiming/ubuntu-project/tiny-dnn/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(all_tests "tiny_dnn_test")
subdirs(../googletest-build)
