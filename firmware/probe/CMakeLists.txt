cmake_minimum_required(VERSION 3.16)

# qm33120w_sdk and uwb_port live in the repo-level components/ directory,
# outside this project's own components/ (which doesn't exist yet).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uwb_probe)
