#
# $Id$
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---


set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes
                              cisstStereoVision)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} cisstQt)

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # MicronTracker library
  find_path (ClaronMicronTracker_INCLUDE_DIR
             NAMES MTC.h
             PATHS "C:/Program Files/Claron Technology/MicronTracker/Dist"
                   "C:/Program Files (x86)/Claron Technology/MicronTracker/Dist")
  find_library (ClaronMicronTracker_MTC_LIBRARY
                NAMES MTC
                HINTS ${ClaronMicronTracker_INCLUDE_DIR})
  set (ClaronMicronTracker_LIBRARIES
       ${ClaronMicronTracker_MTC_LIBRARY})
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

  include_directories (${ClaronMicronTracker_INCLUDE_DIR})

  add_library (sawClaronMicronTracker
               ../sawClaronMicronTrackerExport.h
               mtsMicronTracker.cpp
               ../mtsMicronTracker.h)

  target_link_libraries (sawClaronMicronTracker ${ClaronMicronTracker_LIBRARIES})
  cisst_target_link_libraries (sawClaronMicronTracker ${REQUIRED_CISST_LIBRARIES})

  # Qt widgets
  if (CISST_HAS_QT)

  # this needs to be moved to cisstMacros.cmake
  MACRO (CISST_QT4_WRAP_UI outfiles)
    QT4_EXTRACT_OPTIONS (ui_files ui_options ${ARGN})

    FOREACH (it ${ui_files})
      GET_FILENAME_COMPONENT (outfile ${it} NAME_WE)
      GET_FILENAME_COMPONENT (infile ${it} ABSOLUTE)
      # the only thing we modified ...
      SET (outfile ${CMAKE_CURRENT_BINARY_DIR}/sawClaronMicronTracker/${outfile}.h)
      ADD_CUSTOM_COMMAND (OUTPUT ${outfile}
                          COMMAND ${QT_UIC_EXECUTABLE}
                          ARGS ${ui_options} -o ${outfile} ${infile}
                          MAIN_DEPENDENCY ${infile})
      SET (${outfiles} ${${outfiles}} ${outfile})
    ENDFOREACH (it)
  ENDMACRO (CISST_QT4_WRAP_UI)

    include_directories ("${CMAKE_CURRENT_BINARY_DIR}")

    qt4_wrap_cpp (SAW_MICRON_TRACKER_QT_COMPONENTS_GENERATED
                  ../mtsMicronTrackerControllerQtComponent.h
                  ../mtsMicronTrackerToolQtComponent.h
                  )
    cisst_qt4_wrap_ui (SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED
                       mtsMicronTrackerControllerQtWidget.ui
                       mtsMicronTrackerToolQtWidget.ui)

    add_library (sawClaronMicronTrackerQt
                 ${SAW_MICRON_TRACKER_QT_COMPONENTS_GENERATED}
                 ${SAW_MICRON_TRACKER_QT_WIDGETS_GENERATED}
                 ../sawClaronMicronTrackerExportQt.h
                 mtsMicronTrackerControllerQtComponent.cpp
                 ../mtsMicronTrackerControllerQtComponent.h
                 mtsMicronTrackerToolQtComponent.cpp
                 ../mtsMicronTrackerToolQtComponent.h)

    cisst_target_link_libraries (sawClaronMicronTrackerQt ${REQUIRED_CISST_LIBRARIES} cisstQt)

  endif (CISST_HAS_QT)

  # create/configure file for find_package (sawClaronMicronTracker)
  set (sawClaronMicronTracker_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${ClaronMicronTracker_INCLUDE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
  set (sawClaronMicronTracker_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawClaronMicronTracker_LIBRARIES sawClaronMicronTracker "${ClaronMicronTracker_LIBRARIES}")
  set (sawClaronMicronTrackerQt_LIBRARIES sawClaronMicronTrackerQt)

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawClaronMicronTrackerConfig.cmake.in"
                  "${sawComponents_BINARY_DIR}/sawClaronMicronTrackerConfig.cmake"
                   @ONLY@)

endif (cisst_FOUND)