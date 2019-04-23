#
# (C) Copyright 2011-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawClaronMicronTracker)

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message (FATAL_ERROR "sawClaronMicronTracker requires cisst to be built with JSON")
  endif ()

  # catkin/ROS paths
  cisst_is_catkin_build (sawClaronMicronTracker_IS_CATKIN_BUILT)
  if (sawClaronMicronTracker_IS_CATKIN_BUILT)
    set (sawClaronMicronTracker_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawClaronMicronTracker/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawClaronMicronTracker_BINARY_DIR}/lib")
  else ()
    set (sawClaronMicronTracker_CONFIG_FILE_DIR "${sawClaronMicronTracker_BINARY_DIR}")
  endif ()

  # MicronTracker library
  find_path (ClaronMicronTracker_INCLUDE_DIR
             NAMES MTC.h
             PATHS "C:/Program Files/Claron Technology/MicronTracker/Dist64"
                   "C:/Program Files/Claron Technology/MicronTracker/Dist"
                   "C:/Program Files (x86)/Claron Technology/MicronTracker/Dist")
  find_library (ClaronMicronTracker_MTC_LIBRARY
                NAMES MTC
                HINTS ${ClaronMicronTracker_INCLUDE_DIR})
  set (ClaronMicronTracker_LIBRARIES
       ${ClaronMicronTracker_MTC_LIBRARY})

  # Linux
  if (UNIX)
    find_library (ClaronMicronTracker_dc1394_LIBRARY
                  NAMES dc1394
                  HINTS ${ClaronMicronTracker_INCLUDE_DIR})
    find_library (raw1394_LIBRARY
                  NAMES raw1394)
    find_library (vnl_LIBRARY
                  NAMES vnl)
    find_library (vnl_algo_LIBRARY
                  NAMES vnl_algo)
    find_library (vcl_LIBRARY
                  NAMES vcl)
    list (APPEND ClaronMicronTracker_LIBRARIES
          ${ClaronMicronTracker_dc1394_LIBRARY}
          ${raw1394_LIBRARY}
          ${vnl_LIBRARY}
          ${vnl_algo_LIBRARY}
          ${vcl_LIBRARY})
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
      add_definitions (-D_LINUX64)
    endif (CMAKE_SIZEOF_VOID_P EQUAL 8)
  endif (UNIX)

  if (ClaronMicronTracker_MTC_LIBRARY)
    # create/configure file for find_package (sawClaronMicronTracker)
    set (sawClaronMicronTracker_INCLUDE_DIR
         "${sawClaronMicronTracker_SOURCE_DIR}/include"
         "${sawClaronMicronTracker_BINARY_DIR}"
         "${ClaronMicronTracker_INCLUDE_DIR}")
    set (sawClaronMicronTracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawClaronMicronTracker_LIBRARIES sawClaronMicronTracker "${ClaronMicronTracker_LIBRARIES}")
    set (sawClaronMicronTrackerQt_LIBRARIES sawClaronMicronTrackerQt)

    # Set the version number
    set (sawClaronMicronTracker_VERSION_MAJOR "1")
    set (sawClaronMicronTracker_VERSION_MINOR "1")
    set (sawClaronMicronTracker_VERSION_PATCH "0")
    set (sawClaronMicronTracker_VERSION "${sawClaronMicronTracker_VERSION_MAJOR}.${sawClaronMicronTracker_VERSION_MINOR}.${sawClaronMicronTracker_VERSION_PATCH}")

    # Generate sawClaronMicronTrackerRevision.h
    configure_file ("${sawClaronMicronTracker_SOURCE_DIR}/code/sawClaronMicronTrackerRevision.h.in"
                    "${sawClaronMicronTracker_BINARY_DIR}/include/sawClaronMicronTracker/sawClaronMicronTrackerRevision.h")

    include_directories (${sawClaronMicronTracker_INCLUDE_DIR} ${sawClaronMicronTracker_BINARY_DIR})
    set (sawClaronMicronTracker_HEADER_DIR "${sawClaronMicronTracker_SOURCE_DIR}/include/sawClaronMicronTracker")

    add_library (sawClaronMicronTracker
                 ${sawClaronMicronTracker_HEADER_DIR}/sawClaronMicronTrackerExport.h
                 code/mtsMicronTracker.cpp
                 ${sawClaronMicronTracker_HEADER_DIR}/mtsMicronTracker.h)
    set_property (TARGET sawClaronMicronTracker PROPERTY FOLDER "sawClaronMicronTracker")

    # on Linux, linking against libMTC.a fails because if wasn't compiled with -fPIC
    if (WIN32)
      target_link_libraries (sawClaronMicronTracker ${ClaronMicronTracker_LIBRARIES})
    endif(WIN32)
    cisst_target_link_libraries (sawClaronMicronTracker ${REQUIRED_CISST_LIBRARIES})

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawClaronMicronTracker_LIBRARIES ${sawClaronMicronTracker_LIBRARIES} ${sawClaronMicronTrackerQt_LIBRARIES})
    endif (CISST_HAS_QT)

    set (sawClaronMicronTracker_CMAKE_CONFIG_FILE
         "${sawClaronMicronTracker_CONFIG_FILE_DIR}/sawClaronMicronTrackerConfig.cmake")

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawClaronMicronTrackerConfig.cmake.in"
                    "${sawClaronMicronTracker_CMAKE_CONFIG_FILE}"
                     @ONLY)

    cisst_add_config_version ("${sawClaronMicronTracker_CONFIG_FILE_DIR}/sawClaronMicronTrackerConfigVersion.cmake"
                              VERSION ${sawClaronMicronTracker_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawClaronMicronTracker)

    configure_file ("${sawClaronMicronTracker_SOURCE_DIR}/code/sawClaronMicronTrackerConfig.h.in"
                    "${sawClaronMicronTracker_BINARY_DIR}/include/sawClaronMicronTracker/sawClaronMicronTrackerConfig.h")

    # Install target for headers and library
    install (DIRECTORY
             "${sawClaronMicronTracker_SOURCE_DIR}/include/sawClaronMicronTracker"
             "${sawClaronMicronTracker_BINARY_DIR}/include/sawClaronMicronTracker"
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawClaronMicronTracker
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawClaronMicronTracker_CMAKE_CONFIG_FILE}"
             DESTINATION "share/sawClaronMicronTracker")

  else (ClaronMicronTracker_MTC_LIBRARY)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the MicronTracker SDK (lib MTC)")
  endif  (ClaronMicronTracker_MTC_LIBRARY)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
