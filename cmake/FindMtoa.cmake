find_path(Mtoa_INCLUDE_DIR platform/Platform.h
    HINTS
        $ENV{MTOA_PATH}/include/mtoa)

find_library(Mtoa_LIB mtoa_api
    HINTS
        $ENV{MTOA_PATH}
    PATH_SUFFIXES
        lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mtoa DEFAULT_MSG
    Mtoa_INCLUDE_DIR
    Mtoa_LIB)

if (Mtoa_FOUND)
    set(Mtoa_INCLUDE_DIRS ${Mtoa_INCLUDE_DIR})
    set(Mtoa_LIBRARIES ${Mtoa_LIB})
endif()