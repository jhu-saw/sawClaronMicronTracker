/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Ali Uneri
  Created on: 2009-10-27

  (C) Copyright 2009-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawClaronMicronTracker/mtsMicronTrackerToolQtComponent.h>

#include <QDir>
#include <QString>

CMN_IMPLEMENT_SERVICES(mtsMicronTrackerToolQtComponent);


mtsMicronTrackerToolQtComponent::mtsMicronTrackerToolQtComponent(const std::string & taskName) :
    mtsComponent(taskName)
{
    ToolWidget = new prmPositionCartesianGetQtWidget();
    ToolWidget->setWindowTitle(QString::fromStdString(taskName));

    MTC.MarkerProjectionLeft.SetSize(2);
    MTC.MarkerProjectionRight.SetSize(2);

    mtsInterfaceRequired * required = AddInterfaceRequired(taskName);
    if (required) {
        required->AddFunction("GetPositionCartesian", MTC.GetPositionCartesian);
        required->AddFunction("GetMarkerProjectionLeft", MTC.GetMarkerProjectionLeft);
        required->AddFunction("GetMarkerProjectionRight", MTC.GetMarkerProjectionRight);
    }
}


void mtsMicronTrackerToolQtComponent::UpdatePositionCartesian()
{
    MTC.GetPositionCartesian(MTC.PositionCartesian);
    MTC.GetMarkerProjectionLeft(MTC.MarkerProjectionLeft);
    MTC.GetMarkerProjectionRight(MTC.MarkerProjectionRight);

    ToolWidget->SetValue(MTC.PositionCartesian);

    if (MTC.PositionCartesian.Valid()) {
        MarkerProjectionLeft.setX(MTC.MarkerProjectionLeft.X());
        MarkerProjectionLeft.setY(MTC.MarkerProjectionLeft.Y());
        MarkerProjectionRight.setX(MTC.MarkerProjectionRight.X());
        MarkerProjectionRight.setY(MTC.MarkerProjectionRight.Y());
    } else {
        MarkerProjectionLeft.setX(0.0);
        MarkerProjectionLeft.setY(0.0);
        MarkerProjectionRight.setX(0.0);
        MarkerProjectionRight.setY(0.0);;
    }
}
