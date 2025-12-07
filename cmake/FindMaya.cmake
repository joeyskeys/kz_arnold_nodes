find_path(Maya_INCLUDE_DIR maya/MGlobal.h
    HINTS
        $ENV{DEVKIT_LOCATION}
    PATH_SUFFIXES
        include)

set(LIBRARIES Foundation OpenMaya OpenMayaAnim OpenMayaFX OpenMayaRender OpenMayaUI)

foreach(MAYA_LIB ${LIBRARIES})
    find_library(${MAYA_LIB}_PATH NAMES ${MAYA_LIB}
        HINTS
            $ENV{DEVKIT_LOCATION}
        PATH_SUFFIXES
            lib)
    set(LIBRARY_NAME ${LIBRARY_NAMES} ${MAYA_LIB}_PATH)
    set(LIBRARY_PATHS ${LIBRARY_PATHS} ${${MAYA_LIB}_PATH})
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Maya DEFAULT_MSG
    Maya_INCLUDE_DIR
    ${LIBRARY_NAMES})

if (Maya_FOUND)
    set(Maya_INCLUDE_DIRS ${Maya_INCLUDE_DIR})
    set(Maya_LIBRARIES
        ${LIBRARY_PATHS})
endif()
