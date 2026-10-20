# Prefer glog's own target: newer releases require its compile definitions.
find_package(glog CONFIG QUIET)
if(glog_FOUND)
    return()
endif()

# Older distributions ship glog headers/libraries without a CMake config.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PC_GLOG QUIET libglog)
endif()

find_path(GLOG_INCLUDE_DIRS glog/logging.h
        HINTS ${PC_GLOG_INCLUDE_DIRS} ${GLOG_ROOT} ENV GLOG_ROOT
        PATH_SUFFIXES include)
find_library(GLOG_LIBRARIES NAMES glog
        HINTS ${PC_GLOG_LIBRARY_DIRS} ${GLOG_ROOT} ENV GLOG_ROOT
        PATH_SUFFIXES lib lib64)
set(GLOG_VERSION "${PC_GLOG_VERSION}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog
        REQUIRED_VARS GLOG_LIBRARIES GLOG_INCLUDE_DIRS
        VERSION_VAR GLOG_VERSION)
mark_as_advanced(GLOG_INCLUDE_DIRS GLOG_LIBRARIES)

if(glog_FOUND AND NOT TARGET glog::glog)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_target_properties(glog::glog PROPERTIES
            IMPORTED_LOCATION "${GLOG_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
            INTERFACE_COMPILE_OPTIONS "${PC_GLOG_CFLAGS_OTHER}")
endif()
