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

#ifndef _mtsMicronTrackerControllerQtComponent_h
#define _mtsMicronTrackerControllerQtComponent_h

#include <QImage>
#include <QList>
#include <QPainter>
#include <QTimer>
#include <QWidget>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionVoid.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsVector.h>
#include <sawClaronMicronTracker/sawClaronMicronTrackerExportQt.h>  // always include last

namespace Ui {
    class mtsMicronTrackerControllerQtWidget;
}

class CISST_EXPORT mtsMicronTrackerControllerQtComponent : public QObject, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsMicronTrackerControllerQtComponent(const std::string & taskName);
    ~mtsMicronTrackerControllerQtComponent(void) {};

    void Configure(const std::string & CMN_UNUSED(filename) = "") {};

    void AddTool(QObject * toolQtComponent, QWidget * toolQtWidget, QPoint * markerLeft, QPoint * markerRight);

    QWidget * GetWidget(void) {
        return &CentralWidget;
    }

 protected:
    static const unsigned int FrameWidth = 1024;
    static const unsigned int FrameHeight = 768;
    static const unsigned int FrameSize = FrameWidth * FrameHeight;

    Ui::mtsMicronTrackerControllerQtWidget * ControllerWidget;
    QWidget CentralWidget;
    QTimer * Timer;

    struct {        
        mtsFunctionWrite Capture;
        mtsFunctionWrite Track;
        mtsFunctionRead GetFrameLeft;
        mtsFunctionRead GetFrameRight;
        
        mtsFunctionRead GetXPointsMaxNum;
        mtsFunctionRead GetXPoints;
        mtsFunctionRead GetXPointsProjectionLeft;
        mtsFunctionRead GetXPointsProjectionRight;        

        mtsUCharVec FrameLeft;
        mtsUCharVec FrameRight;
        
        int XPointsMaxNum;
        std::vector<vct3> XPoints;
        std::vector<vct3> XPointsProjectionLeft;
        std::vector<vct3> XPointsProjectionRight;
    } MTC;
    
    QImage FrameIndexed8;
    QImage FrameRGB;
    QPainter MarkerPainter;
    QPoint MarkerPosition;
    QList<QString> MarkerNames;
    QList<QPoint *> MarkersLeft;
    QList<QPoint *> MarkersRight;
    
 public slots:
    void UpdateFrames();
    void PaintImage(QImage & frameIndexed8, QList<QPoint *> & markers);
    void PaintImageWithXpoints(QImage & frameIndexed8, QList<QPoint *> & markers,
                               std::vector<vct3> & xpoints);    
    void MTCTrackQSlot(bool toggled);    
    void ScreenshotQSlot(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsMicronTrackerControllerQtComponent);

#endif  // _mtsMicronTrackerControllerQDevice_h
