#
# $Id$
#
# (C) Copyright 2009-2011 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# create a list of libraries needed for this project
set (REQUIRED_CISST_LIBRARIES cisstCommon cisstCommonQt cisstCommonXML
                              cisstVector cisstOSAbstraction cisstNumerical
                              cisstMultiTask cisstParameterTypes
                              cisstStereoVision)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawClaronMicronTracker has been compiled within cisst, we should find it automatically
  find_package (sawClaronMicronTracker REQUIRED HINTS "${cisst_saw_BINARY_DIR}/saw/components")

  if (sawClaronMicronTracker_FOUND)

    # sawClaronMicronTracker configuration
    include_directories (${sawClaronMicronTracker_INCLUDE_DIR})
    link_directories (${sawClaronMicronTracker_LIBRARY_DIR})

    add_executable (sawClaronMicronTrackerQtExample mainQt.cpp)

    # link against non cisst libraries and cisst components
    target_link_libraries (sawClaronMicronTrackerQtExample
                           ${sawClaronMicronTracker_LIBRARIES}
                           ${sawClaronMicronTrackerQt_LIBRARIES}
                           ${QT_LIBRARIES}
                         )
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawClaronMicronTrackerQtExample ${REQUIRED_CISST_LIBRARIES})

  endif (sawClaronMicronTracker_FOUND)

endif (cisst_FOUND)
