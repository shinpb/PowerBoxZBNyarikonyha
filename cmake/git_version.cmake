# Sets PROJECT_VER = short git hash ("-d" suffix if tracked files differ from HEAD), max 16 chars.
# Usage (root CMakeLists.txt, BEFORE project.cmake / project()):
#   include(${CMAKE_CURRENT_LIST_DIR}/cmake/git_version.cmake)
# Plain include (not a function), so PROJECT_VER is set in the caller's scope.
# Note: CMAKE_CURRENT_LIST_DIR inside this file is its own directory, so the repo root is CMAKE_SOURCE_DIR.

find_package(Git QUIET)
set(_gv_ver "nogit")
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short=7 HEAD
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                  OUTPUT_VARIABLE _gv_hash OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE _gv_rc)
  if(_gv_rc EQUAL 0)
    execute_process(COMMAND ${GIT_EXECUTABLE} diff --quiet HEAD
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} RESULT_VARIABLE _gv_dirty ERROR_QUIET)
    set(_gv_ver ${_gv_hash})
    if(NOT _gv_dirty EQUAL 0)
      set(_gv_ver "${_gv_hash}-d")
    endif()

    # Re-run CMake (refreshing PROJECT_VER) when the git state changes, not only on a clean build.
    execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --absolute-git-dir
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                    OUTPUT_VARIABLE _gv_gitdir OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND ${GIT_EXECUTABLE} symbolic-ref -q HEAD
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                    OUTPUT_VARIABLE _gv_ref OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    foreach(_gv_f HEAD index packed-refs ${_gv_ref})
      if(EXISTS "${_gv_gitdir}/${_gv_f}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_gv_gitdir}/${_gv_f}")
      endif()
    endforeach()
  endif()
endif()
set(PROJECT_VER "${_gv_ver}")