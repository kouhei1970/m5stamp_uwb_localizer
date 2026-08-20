cmake_minimum_required(VERSION 3.16)

# uwb_ranging / uwb_qm33120 / uwb_port / uwb_loc / qm33120w_sdk はすべて
# repo-level components/ 配下にある（firmware/tag 自身の components/ は無い）。
# firmware/twr 等と同じ流儀。
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uwb_tag)
