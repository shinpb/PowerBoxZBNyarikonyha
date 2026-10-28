cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/git_version.cmake)   # sets PROJECT_VER

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#set(EXTRA_COMPONENT_DIRS "C:/WS2/PowerBoxZBNyarikonyha/src")
project(PowerBoxZBNyarikonyha)
