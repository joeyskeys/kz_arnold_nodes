find_path(Arnold_INCLUDE_DIR ai.h
    HINTS
        $ENV{ARNOLD_PATH}/include/arnold)

find_library(AI_LIB ai
    HINTS
        $ENV{ARNOLD_PATH}
    PATH_SUFFIXES
        lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Arnold DEFAULT_MSG
    Arnold_INCLUDE_DIR
    AI_LIB)

if (Arnold_FOUND)
    set(Arnold_INCLUDE_DIRS ${Arnold_INCLUDE_DIR})
    set(Arnold_LIBRARIES ${AI_LIB})
endif()