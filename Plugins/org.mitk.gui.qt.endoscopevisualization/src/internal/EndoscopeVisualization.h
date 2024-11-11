/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/


#ifndef EndoscopeVisualization_h
#define EndoscopeVisualization_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_EndoscopeVisualizationControls.h"


//TEST
#include <mitkNavigationDataToPointSetFilter.h>
#include <mitkNavigationDataLandmarkTransformFilter.h>
#include <mitkNavigationDataReferenceTransformFilter.h>
#include <mitkNavigationDataObjectVisualizationFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <mitkTrackingDeviceSource.h>
#include <mitkSurface.h>
#include <mitkCameraVisualization.h>

#include <QToolBox>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QTimer>

#include <vtkLandmarkTransform.h>
#include <vtkSmartPointer.h>
//

/**
  \brief EndoscopeVisualization

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class EndoscopeVisualization : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  
    static const std::string VIEW_ID;

  EndoscopeVisualization(); //constructor

  ~EndoscopeVisualization(); //destructor


protected:
  
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  Ui::EndoscopeVisualizationControls m_Controls;

  void CalculationSelected();
  void PerformCalculation(int calculationType);

  void InterpolationSelected();
  void PerformInterpolation(int interpolationType);

  void TubeDiameterChanged(int tubediameter);
  void PerformTube();

  void VisualizeEndoscope();

  mitk::TrackingDeviceSource::Pointer m_Source;                       // connected Tracking Device
  mitk::DataStorage *datastorage;                                     // data storage that contains the navigation data
  std::vector<mitk::NavigationData::Pointer> m_SensorDataList;        // list containing the navigation data of all 6 sensors
  mitk::NavigationData::Pointer m_NavigationDataSensor;

  int m_selectedCalculationType=1;
  int m_selectedInterpolationType=1;
  int m_selectedtubediameter;

  QTimer *m_Timer;                                                    // Timer to update the tracking data


  mitk::PointSet::Pointer splinepointset = mitk::PointSet::New();

  void SetupNavigation(); 

  void UpdateTrackingData();

  void PerformCalculation1();
  void PerformCalculation2();
  void PerformCalculation3();

  void PerformInterpolation1();
  void PerformInterpolation2();
  void PerformInterpolation3();
  void PerformInterpolation4();
  void PerformInterpolation5();

};

#endif // EndoscopeVisualization_h
