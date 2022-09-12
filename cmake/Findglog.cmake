include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLOG DEFAULT_MSG GLOG_LIBRARIES)

if(GLOG_FOUND)
    message(STATUS "glog library found at ${GLOG_LIBRARIES}")
    if(PC_GLOG_VERSION)
        set(GLOG_VERSION ${PC_GLOG_VERSION})
    endif()
endif()