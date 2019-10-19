include(FindPackageHandleStandardArgs)

set(MVSDK_ROOT_DIR "${PROJECT_SOURCE_DIR}/third_party/mind_vision" CACHE PATH "Folder contains MVSDK")

find_path(MVSDK_INCLUDE_DIR CameraApi.h
        PATHS ${MVSDK_ROOT_DIR}/include)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" )
    find_library(MVSDK_LIBRARY MVSDK
        PATHS ${MVSDK_ROOT_DIR}/lib/x64
        PATH_SUFFIXES lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
    find_library(MVSDK_LIBRARY MVSDK
        PATHS ${MVSDK_ROOT_DIR}/lib/arm64
        PATH_SUFFIXES lib)
else()
	message(FATAL_ERROR "${CMAKE_SYSTEM_PROCESSOR} not support for CamLib MVSDK")
endif()

find_package_handle_standard_args(MVSDK DEFAULT_MSG MVSDK_INCLUDE_DIR MVSDK_LIBRARY)

if(MVSDK_FOUND)
  set(MVSDK_INCLUDE_DIRS ${MVSDK_INCLUDE_DIR})
  set(MVSDK_LIBRARIES ${MVSDK_LIBRARY})
  message(STATUS "Found MVSDK    (include: ${MVSDK_INCLUDE_DIR}, library: ${MVSDK_LIBRARY})")
  mark_as_advanced(MVSDK_ROOT_DIR MVSDK_LIBRARY MVSDK_INCLUDE_DIR)
endif()