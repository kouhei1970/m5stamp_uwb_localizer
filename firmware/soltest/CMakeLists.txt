cmake_minimum_required(VERSION 3.16)

# uwb_loc は repo-level components/ 配下にある（firmware/soltest 自身の
# components/ は無い）。firmware/probe 等と同じ流儀。
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uwb_soltest)
