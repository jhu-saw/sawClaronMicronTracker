#
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
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

find_package (cisst 1.0.6 REQUIRED ${REQUIRED_CISST_LIBRARIES})

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
  if (WIN32)    
    # Windows
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
  elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_path (ClaronMicronTracker_INCLUDE_DIR
               NAMES MTC.h)
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
    find_library (v3p_netlib_LIBRARY
                  NAMES v3p_netlib)
    list (APPEND ClaronMicronTracker_LIBRARIES
          ${ClaronMicronTracker_dc1394_LIBRARY}
          ${raw1394_LIBRARY}
          ${vnl_LIBRARY}
          ${vnl_algo_LIBRARY}
          ${vcl_LIBRARY}
          ${v3p_netlib_LIBRARY})
  endif (UNIX)

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
    set (sawClaronMicronTracker_VERSION_MINOR "0")
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
  target_link_libraries (sawClaronMicronTracker ${ClaronMicronTracker_LIBRARIES})
  cisst_target_link_libraries (sawClaronMicronTracker ${REQUIRED_CISST_LIBRARIES})

  # # Qt widgets
  # if (CISST_HAS_QT)

  #   file (MAKE_DIRECTORY "${sawClaronMicronTracker_BINARY_DIR}/sawClaronMicronTracker")

  #   # initial find_package doesn't have cisstQt
  #   cisst_set_directories (cisstQt)

  #   # for Qt generated header files
  #   include_directories (${CMAKE_CURRENT_BINARY_DIR})
  #   qt4_wrap_cpp (SAW_MICRON_TRACKER_QT_COMPONENTS_GENERATED
  #                 ${sawClaronMicronTracker_HEADER_DIR}/mtsMicronTrackerControllerQtComponent.h
  #                 ${sawClaronMicronTracker_HEADER_DIR}/mtsMicronTrackerToolQtComponent.h)
  #   qt4_wrap_ui (SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED
  #                mtsMicronTrackerControllerQtWidget.ui
  #                mtsMicronTrackerToolQtWidget.ui)

  #   add_library (sawClaronMicronTrackerQt
  #                ${SAW_MICRON_TRACKER_QT_COMPONENTS_GENERATED}
  #                ${SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED}
  #                ${sawClaronMicronTracker_HEADER_DIR}/sawClaronMicronTrackerExportQt.h
  #                mtsMicronTrackerControllerQtComponent.cpp
  #                ${sawClaronMicronTracker_HEADER_DIR}/mtsMicronTrackerControllerQtComponent.h
  #                mtsMicronTrackerToolQtComponent.cpp
  #                ${sawClaronMicronTracker_HEADER_DIR}/mtsMicronTrackerToolQtComponent.h)
  #   set_property (TARGET sawClaronMicronTrackerQt PROPERTY FOLDER "sawClaronMicronTracker")

  #   # find all the header files generated by qt4_wrap_ui and copy to include dir
  #   foreach (headerFile ${SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED})
  #     # get the filename only
  #     get_filename_component (fileName "${headerFile}" NAME)
  #     # recent versions of CMake add a prefix ui_, strip it!
  #     string (REGEX REPLACE "^ui_" ""
  #             fileNameWoUI ${fileName})
  #     add_custom_command (TARGET sawClaronMicronTrackerQt
  #                         POST_BUILD
  #                         COMMAND ${CMAKE_COMMAND}
  #                         ARGS -E copy_if_different
  #                                 ${headerFile}
  #                                 ${sawClaronMicronTracker_BINARY_DIR}/sawClaronMicronTracker/${fileNameWoUI})
  #   endforeach (headerFile ${SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED})

  #   target_link_libraries (sawClaronMicronTrackerQt ${QT_LIBRARIES})
  #   cisst_target_link_libraries (sawClaronMicronTrackerQt ${REQUIRED_CISST_LIBRARIES} cisstQt)
  # endif (CISST_HAS_QT)

  # add Qt code
  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawClaronMicronTracker_LIBRARIES ${sawClaronMicronTracker_LIBRARIES} ${sawClaronMicronTrackerQt_LIBRARIES})
  endif (CISST_HAS_QT)

  set (sawClaronMicronTracker_CMAKE_CONFIG_FILE 
    "${sawClaronMicronTracker_CONFIG_FILE_DIR}/sawClaronMicronTrackerConfig.cmake")  

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawClaronMicronTrackerConfig.cmake.in"
                  "${sawClaronMicronTracker_CMAKE_CONFIG_FILE}"
                   @ONLY@)

  cisst_add_config_version ("${sawClaronMicronTracker_CONFIG_FILE_DIR}/sawClaronMicronTrackerConfigVersion.cmake"
                            VERSION ${sawClaronMicronTracker_VERSION}
                            DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                            COMPONENT sawClaronMicronTracker)
  # Install target for headers and library
  install (DIRECTORY
           "${sawClaronMicronTracker_SOURCE_DIR}/include/sawClaronMicronTracker"
           "${sawClaronMicronTracker_BINARY_DIR}/include/sawClaronMicronTracker"
           DESTINATION include)

  install (TARGETS sawClaronMicronTracker
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawClaronMicronTracker_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawClaronMicronTracker")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
