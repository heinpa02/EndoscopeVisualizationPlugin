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


#include <vtkParametricFunctionSource.h>
#include <vtkParametricSpline.h>
#include <vtkQuaternion.h>


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

  Ui::EndoscopeVisualizationControls m_Controls;                            // Ui controls
  mitk::TrackingDeviceSource::Pointer m_Source;                             // connected Tracking Device
    
  QTimer *m_Timer;                                                          // Timer to update the tracking data

  mitk::DataStorage *datastorage;                                           // data storage that contains the navigation data

  // CALCULATION
  std::vector<mitk::NavigationData::Pointer> m_NavigationDataList;          // list containing the navigation data of the 6 sensors (INPUT CALCULATION)
  std::vector<mitk::NavigationData::Pointer> m_NodeList;                    // list containing the nodes for spline interpolation
  vtkSmartPointer<vtkPoints> pointseven;
  vtkSmartPointer<vtkPoints> pointsuneven;
  vtkSmartPointer<vtkPoints> points;                                        // nodes as vtkpoints (OUTPUT CALCULATION)
  vtkSmartPointer<vtkParametricSpline> splineEven;
  vtkSmartPointer<vtkParametricSpline> splineUneven;

  vtkSmartPointer<vtkParametricSpline> spline;
  vtkSmartPointer<vtkParametricFunctionSource> functionSource;
  vtkSmartPointer<vtkParametricFunctionSource> averagedfunctionSource;

  vtkSmartPointer<vtkActor> actorSpline;
  vtkSmartPointer<vtkActor> actorPoints;
  vtkSmartPointer<vtkActor> actorTube;

  void SetupNavigation();

  void UpdateTrackingData();

  void CalculationSelected();
  int m_selectedCalculationType = 1;
  void PerformCalculation(int calculationType);

  void PerformCalculation1();
  mitk::NavigationData::Pointer CalculateMidpointAndOrientation(mitk::NavigationData::Pointer sensor1Data, mitk::NavigationData::Pointer sensor2Data);
  
 
  void PerformCalculation2();
  
  void QuaternionToEuler(const mitk::Quaternion &quat, double &pitch, double &yaw, double &roll);
  mitk::Quaternion EulerToQuaternion(double pitch, double yaw, double roll);

  void PerformCalculation3();


  void InterpolationSelected();
  void PerformInterpolation(int interpolationType); 
  void PerformInterpolation_Parametric();
  void PerformInterpolation_Kochanek();
  void PerformInterpolation_Cardinal();
  void PerformInterpolation_SCurve();
  void PerformInterpolation5();
  
  int m_selectedInterpolationType = 1;

  void TubeDiameterChanged(int tubeDiameter);
  int m_selectedTubeDiameter = 10;

  void VisualizePoints();
  void VisualizeSpline();
  void VisualizeTube();

  void VisualizeEndoscope();

};

#endif // EndoscopeVisualization_h
