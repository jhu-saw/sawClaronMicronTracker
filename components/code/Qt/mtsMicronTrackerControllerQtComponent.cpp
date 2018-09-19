/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-10-29

  (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <QDir>
#include <QString>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawClaronMicronTracker/mtsMicronTrackerControllerQtComponent.h>
#include <ui_mtsMicronTrackerControllerQtWidget.h>

CMN_IMPLEMENT_SERVICES(mtsMicronTrackerControllerQtComponent);


mtsMicronTrackerControllerQtComponent::mtsMicronTrackerControllerQtComponent(const std::string & taskName) :
    mtsComponent(taskName)
{
    ControllerWidget = new Ui::mtsMicronTrackerControllerQtWidget();
    ControllerWidget->setupUi(&CentralWidget);

    MTC.FrameLeft.SetSize(FrameSize);
    MTC.FrameRight.SetSize(FrameSize);
    FrameIndexed8 = QImage(FrameWidth, FrameHeight, QImage::Format_Indexed8);
    Timer = new QTimer(this);
    
    MTC.XPoints.resize(50);
    MTC.XPointsProjectionLeft.resize(50);
    MTC.XPointsProjectionRight.resize(50);

    mtsInterfaceRequired * required = AddInterfaceRequired("Controller");
    if (required) {
        required->AddFunction("ToggleCapturing", MTC.Capture);
        required->AddFunction("ToggleTracking", MTC.Track);
        required->AddFunction("GetCameraFrameLeft", MTC.GetFrameLeft);
        required->AddFunction("GetCameraFrameRight", MTC.GetFrameRight);        
        required->AddFunction("GetXPointsMaxNum", MTC.GetXPointsMaxNum);
        required->AddFunction("GetXPoints", MTC.GetXPoints);
        required->AddFunction("GetXPointsProjectionLeft", MTC.GetXPointsProjectionLeft);
        required->AddFunction("GetXPointsProjectionRight", MTC.GetXPointsProjectionRight);
    }

    // connect Qt signals to slots    
    QObject::connect(ControllerWidget->ButtonTrack, SIGNAL(toggled(bool)),
                     this, SLOT(MTCTrackQSlot(bool)));    
    QObject::connect(ControllerWidget->ButtonScreenshot, SIGNAL(clicked()),
                     this, SLOT(ScreenshotQSlot()));
    QObject::connect(this->Timer, SIGNAL(timeout()),
                     this, SLOT(UpdateFrames()));
}


void mtsMicronTrackerControllerQtComponent::AddTool(QObject * toolQtComponent, QWidget * toolQtWidget, QPoint * markerLeft, QPoint * markerRight)
{
    ControllerWidget->LayoutTools->addWidget(toolQtWidget);    

    MarkerNames.append(toolQtWidget->windowTitle());
    MarkersLeft.append(markerLeft);
    MarkersRight.append(markerRight);

    QObject::connect(this->Timer, SIGNAL(timeout()),
                     toolQtComponent, SLOT(UpdatePositionCartesian()));
}


void mtsMicronTrackerControllerQtComponent::UpdateFrames()
{
    MTC.GetXPointsMaxNum(MTC.XPointsMaxNum);
    
    if (MTC.XPointsMaxNum > 0)
        MTC.XPoints.resize(MTC.XPointsMaxNum);
    
    if (ControllerWidget->ButtonCaptureFrameLeft->isChecked()) {
        MTC.GetFrameLeft(MTC.FrameLeft);
        memcpy(FrameIndexed8.bits(), MTC.FrameLeft.Pointer(), FrameSize);
        
        if (MTC.XPointsMaxNum > 0) {
            MTC.XPointsProjectionLeft.resize(MTC.XPointsMaxNum);
            MTC.GetXPointsProjectionLeft(MTC.XPointsProjectionLeft);
            PaintImageWithXpoints(FrameIndexed8, MarkersLeft, MTC.XPointsProjectionLeft);
        }
        else
            PaintImage(FrameIndexed8, MarkersLeft);
        
        ControllerWidget->FrameLeft->setPixmap(QPixmap::fromImage(FrameRGB));
    } else {
        ControllerWidget->FrameLeft->clear();
        CentralWidget.parentWidget()->resize(0,0);
    }

    if (ControllerWidget->ButtonCaptureFrameRight->isChecked()) {
        MTC.GetFrameRight(MTC.FrameRight);
        memcpy(FrameIndexed8.bits(), MTC.FrameRight.Pointer(), FrameSize);
        
        if (MTC.XPointsMaxNum > 0) {
            MTC.XPointsProjectionRight.resize(MTC.XPointsMaxNum);
            MTC.GetXPointsProjectionRight(MTC.XPointsProjectionRight);
            PaintImageWithXpoints(FrameIndexed8, MarkersRight, MTC.XPointsProjectionRight);
        }
        else
            PaintImage(FrameIndexed8, MarkersRight);
        
        ControllerWidget->FrameRight->setPixmap(QPixmap::fromImage(FrameRGB));
    } else {
        ControllerWidget->FrameRight->clear();
        CentralWidget.parentWidget()->resize(0,0);
    }
}


void mtsMicronTrackerControllerQtComponent::PaintImage(QImage & frameIndexed8, QList<QPoint *> & markers)
{
    FrameRGB = frameIndexed8.convertToFormat(QImage::Format_ARGB32_Premultiplied);
    const size_t numTools = ControllerWidget->LayoutTools->count();
    for (unsigned int i = 0; i < numTools; i++) {
        MarkerPainter.begin(&FrameRGB);
        MarkerPainter.setPen(Qt::red);
        MarkerPainter.setBrush(Qt::red);  // paint inside the ellipse
        MarkerPosition = QPoint(markers[i]->x(), markers[i]->y());
        MarkerPainter.drawEllipse(MarkerPosition, 2, 2);
        MarkerPosition += QPoint(3, -3);  // label offset
        MarkerPainter.setFont(QFont(MarkerPainter.font().family(), 10, QFont::DemiBold));
        MarkerPainter.drawText(MarkerPosition, MarkerNames[i]);
        MarkerPainter.end();
    }
}

void mtsMicronTrackerControllerQtComponent::PaintImageWithXpoints(QImage & frameIndexed8, QList<QPoint *> & markers, std::vector<vct3> & xpoints)
{
    FrameRGB = frameIndexed8.convertToFormat(QImage::Format_ARGB32_Premultiplied);
    const size_t numTools = ControllerWidget->LayoutTools->count();
    for (unsigned int i = 0; i < numTools; i++) {
        MarkerPainter.begin(&FrameRGB);
        MarkerPainter.setPen(Qt::red);
        MarkerPainter.setBrush(Qt::red);  // paint inside the ellipse
        MarkerPosition = QPoint(markers[i]->x(), markers[i]->y());
        MarkerPainter.drawEllipse(MarkerPosition, 2, 2);
        MarkerPosition += QPoint(3, -3);  // label offset
        MarkerPainter.setFont(QFont(MarkerPainter.font().family(), 10, QFont::DemiBold));
        MarkerPainter.drawText(MarkerPosition, MarkerNames[i]);
        
        if (MTC.XPointsMaxNum > 0) {
            for (unsigned int j = 0; j < (unsigned int)MTC.XPointsMaxNum; j++) {
                MarkerPainter.setPen(Qt::green);
                MarkerPainter.setBrush(Qt::green);
                MarkerPosition = QPoint(xpoints[j].X(), xpoints[j].Y());
                MarkerPainter.drawEllipse(MarkerPosition, 2, 2);
            }
         }
        MarkerPainter.end();
    }
}

void mtsMicronTrackerControllerQtComponent::MTCTrackQSlot(bool toggled)
{
    MTC.Capture(mtsBool(toggled));
    MTC.Track(mtsBool(toggled));
    if (toggled) {
        Timer->start(20);
    } else {
        Timer->stop();
    }
}


void mtsMicronTrackerControllerQtComponent::ScreenshotQSlot(void)
{
    QPixmap leftCamera(ControllerWidget->FrameLeft->size());
    QPixmap rightCamera(ControllerWidget->FrameRight->size());
    leftCamera = CentralWidget.grab(ControllerWidget->FrameLeft->geometry());
    rightCamera = CentralWidget.grab(ControllerWidget->FrameRight->geometry());

    std::string dateTime;
    osaGetDateTimeString(dateTime);
    std::cerr << QDir::currentPath().toStdString() << std::endl;	
    QString leftPath = QDir::currentPath() + "/LeftCamera-" + dateTime.c_str() + ".tif";
    if (!leftPath.isEmpty()) {
        leftCamera.save(leftPath, "tif");
    }
    QString rightPath = QDir::currentPath() + "/RightCamera-" + dateTime.c_str() + ".tif";
    if (!rightPath.isEmpty()) {
        rightCamera.save(rightPath, "tif");
    }
}
