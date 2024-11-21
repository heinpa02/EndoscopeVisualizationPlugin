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


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "EndoscopeVisualization.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

// GUI
#include "ui_EndoscopeVisualizationControls.h"

//DataSourceSelectionWidget
#include "QmitkNavigationDataSourceSelectionWidget.h"

//TEST
#include <QmitkRenderWindow.h>
#include <QmitkNDIConfigurationWidget.h>
#include <QmitkFiducialRegistrationWidget.h>
#include <QmitkUpdateTimerWidget.h>
#include <QmitkToolSelectionWidget.h>
#include <QmitkToolTrackingStatusWidget.h>
#include <mitkStaticIGTHelperFunctions.h>
#include <mitkIGTException.h>

//TEST 
#include <mitkDataNode.h>
#include <mitkRenderingManager.h>
#include <mitkPoint.h>
#include <mitkDataNode.h>
#include <mitkQuaternionAveraging.h>
#include <mitkQuaternion.h>
#include <vector>

#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkSmartPointer.h>
#include "vtkSpline.h"
#include <vtkParametricSpline.h>
#include <vtkKochanekSpline.h>
#include <vtkCardinalSpline.h>
#include <vtkSCurveSpline.h>
#include <vtkTubeFilter.h>

#include <mitkMatrix.h>
#include <mitkVector.h>
#include <vtkQuaternion.h>
#include <vtkMatrix4x4.h>

#include <vtkParametricFunctionSource.h>
#include <vtkNamedColors.h>
#include <vtkVertexGlyphFilter.h>




const std::string EndoscopeVisualization::VIEW_ID = "org.mitk.views.endoscopevisualization";


EndoscopeVisualization::EndoscopeVisualization()
  : QmitkAbstractView(),
    m_Source(nullptr)
{}


EndoscopeVisualization::~EndoscopeVisualization() {}


void EndoscopeVisualization::SetFocus()
{
  m_Controls.buttonPerformEndoscopeVisualization->setFocus();
}


void EndoscopeVisualization::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  //Initialize and connect Timer
  m_Timer = new QTimer(this);
  connect(m_Timer, &QTimer::timeout, this, &EndoscopeVisualization::UpdateTrackingData);

  // VisualizeEndoscope button
  connect(m_Controls.buttonPerformEndoscopeVisualization, &QPushButton::clicked, this, &EndoscopeVisualization::VisualizeEndoscope);

  // NavigationDataSourceSelectionWidget
  connect(m_Controls.widget, SIGNAL(NavigationDataSourceSelected(mitk::NavigationDataSource::Pointer)), this, SLOT(SetupNavigation()));

  // CalculationSelection radiobuttons
  connect(m_Controls.Cal1, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  connect(m_Controls.Cal2, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  connect(m_Controls.Cal3, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  m_Controls.Cal1->setChecked(true);

  // InterpolationSelection radiobuttons
  connect(m_Controls.Interpol1, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  connect(m_Controls.Interpol2, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  connect(m_Controls.Interpol3, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  connect(m_Controls.Interpol4, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  m_Controls.Interpol1->setChecked(true);

  // TubeDiameterSelection spinbox
  connect(m_Controls.spinBoxTubeDiameter, QOverload<int>::of(&QSpinBox::valueChanged), this, &EndoscopeVisualization::TubeDiameterChanged);
}



void EndoscopeVisualization::VisualizeEndoscope()
{
  MITK_INFO << "Erreicht VisualizeEndoscope.";

  datastorage = this->GetDataStorage();
  if (datastorage == nullptr)
  {
    MITK_ERROR << "DataStorage is null!";
    return;
  }

  m_Timer->start(10);  
}


void EndoscopeVisualization::CalculationSelected() 
{
  if (m_Controls.Cal1->isChecked())
  {
    MITK_INFO << "Calculation 1";
    m_selectedCalculationType = 1;
  }
  else if (m_Controls.Cal2->isChecked())
  {
    MITK_INFO << "Calculation 2 ";
    m_selectedCalculationType = 2;
  }
  else if (m_Controls.Cal3->isChecked())
  {
    MITK_INFO << "Calculation 3 ";
    m_selectedCalculationType = 3;
  }
}


void EndoscopeVisualization::InterpolationSelected()
{
  if (m_Controls.Interpol1->isChecked())
  {
    MITK_INFO << "Interpolation 1";
    m_selectedInterpolationType = 1;
  }
  else if (m_Controls.Interpol2->isChecked())
  {
    MITK_INFO << "Interpolation 2 ";
    m_selectedInterpolationType = 2;
  }
  else if (m_Controls.Interpol3->isChecked())
  {
    MITK_INFO << "Interpolation 3 ";
    m_selectedInterpolationType = 3;
  }
  else if (m_Controls.Interpol4->isChecked())
  {
    MITK_INFO << "Interpolation 4 ";
    m_selectedInterpolationType = 4;
  }
}



void EndoscopeVisualization::TubeDiameterChanged(int tubediameter) {
  MITK_INFO << "Durchmesser= " << tubediameter;
  m_selectedTubeDiameter = tubediameter;
}


void EndoscopeVisualization::SetupNavigation()
{
  if (m_Source.IsNotNull())
    if (m_Source->IsTracking())
      if (m_Controls.widget->GetSelectedToolID() != -1)
        return;
      MITK_ERROR << "Please select the last detected tool.";
    MITK_INFO << "Please start tracking.";
  MITK_INFO << "Please select a tracking device.";
}


void EndoscopeVisualization::UpdateTrackingData()
{
  MITK_INFO << "Update trackingdata";
  
  // extract tracking data from the navigation data source

  m_NavigationDataList.clear();
  if (m_Controls.widget->GetSelectedToolID() <= 6)
  {
    for (size_t i = 0; i <= m_Controls.widget->GetSelectedToolID(); ++i)
    {
      mitk::NavigationData::Pointer m_NavigationDataSensor;
      m_NavigationDataSensor = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(i);
      m_NavigationDataList.push_back(m_NavigationDataSensor);
    }
  }
  else{
    MITK_INFO<< "Additional sensor detected. Make sure the EndoscopeNavigation is plugged into the first three inputs.";
     mitk::NavigationData::Pointer m_NavigationDataSensor0 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(0);
     m_NavigationDataList.push_back(m_NavigationDataSensor0);
     mitk::NavigationData::Pointer m_NavigationDataSensor1 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(1);
     m_NavigationDataList.push_back(m_NavigationDataSensor1);
     mitk::NavigationData::Pointer m_NavigationDataSensor2 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(2);
     m_NavigationDataList.push_back(m_NavigationDataSensor2);
     mitk::NavigationData::Pointer m_NavigationDataSensor3 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(4);
     m_NavigationDataList.push_back(m_NavigationDataSensor3);
     mitk::NavigationData::Pointer m_NavigationDataSensor4 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(5);
     m_NavigationDataList.push_back(m_NavigationDataSensor4);
     mitk::NavigationData::Pointer m_NavigationDataSensor5 = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(6);
     m_NavigationDataList.push_back(m_NavigationDataSensor5);
  }
 

  // processing tracking data and visualizing endoscope

  PerformCalculation(m_selectedCalculationType);
  PerformInterpolation(m_selectedInterpolationType);
  VisualizePoints();
  VisualizeSpline();
  VisualizeTube();

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->RequestUpdate();

}


void EndoscopeVisualization::PerformCalculation(int calculationType) 
{
  // initialize new for next iteration
  m_NodeList.clear();
  points = vtkSmartPointer<vtkPoints>::New();
  functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();

  switch (calculationType)
  {
    case 1:
      PerformCalculation1();
      break;
    case 2:
      PerformCalculation2();
      break;
    case 3:
      PerformCalculation3();
      break;
  }
}


void EndoscopeVisualization::PerformInterpolation(int interpolationType)
{
  MITK_INFO << "Erreicht PerformInterpolation.";

    vtkRenderer *vtkRenderer = this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer();

  if (actorPoints)
  {
    vtkRenderer->RemoveActor(actorPoints);
    actorPoints = nullptr;
  }
  if (actorSpline)
  {
    vtkRenderer->RemoveActor(actorSpline);
    actorSpline = nullptr;
  }
  if (actorTube)
  {
    vtkRenderer->RemoveActor(actorTube);
    actorTube = nullptr;
  }

  

  switch (interpolationType)
  {
    case 1:
      PerformInterpolation_Parametric();
      break;
    case 2:
      PerformInterpolation_Kochanek();
      break;
    case 3:
      PerformInterpolation_Cardinal();
      break;
    case 4:
      PerformInterpolation_SCurve();
      break;
    case 5:
      PerformInterpolation5();
      break;
  }
}


void EndoscopeVisualization::PerformCalculation1()
{
  MITK_INFO << "Averaging Positions";

  mitk::NavigationData::Pointer node1 = mitk::NavigationData::New();
  mitk::NavigationData::Pointer node2 = mitk::NavigationData::New();
  mitk::NavigationData::Pointer node3 = mitk::NavigationData::New();
  node1 = CalculateMidpointAndOrientation(m_NavigationDataList[0], m_NavigationDataList[1]);
  node2 = CalculateMidpointAndOrientation(m_NavigationDataList[2], m_NavigationDataList[3]);
  node3 = CalculateMidpointAndOrientation(m_NavigationDataList[4], m_NavigationDataList[5]);
  m_NodeList.push_back(node1);
  m_NodeList.push_back(node2);
  m_NodeList.push_back(node3);

  points->SetNumberOfPoints(m_NodeList.size());
  for (size_t i = 0; i < m_NodeList.size(); ++i)
  {
    mitk::Point3D position = m_NodeList[i]->GetPosition();
    points->SetPoint(i, position[0], position[1], position[2]);
  }
}


mitk::NavigationData::Pointer EndoscopeVisualization::CalculateMidpointAndOrientation(mitk::NavigationData::Pointer sensor1Data, mitk::NavigationData::Pointer sensor2Data)
{

  mitk::Point3D position1 = sensor1Data->GetPosition();
  mitk::Point3D position2 = sensor2Data->GetPosition();

  mitk::Quaternion orientation1 = sensor1Data->GetOrientation();
  mitk::Quaternion orientation2 = sensor2Data->GetOrientation();

  MITK_INFO << orientation1.rotation_euler_angles();
  MITK_INFO << orientation2.rotation_euler_angles();

  vtkQuaternion<double> vtkQuat1(orientation1.r(), orientation1.x(), orientation1.y(), orientation1.z());
  vtkQuaternion<double> vtkQuat2(orientation2.r(), orientation2.x(), orientation2.y(), orientation2.z());


  mitk::NavigationData::Pointer newsensor = mitk::NavigationData::New();

  // Position imaginary 6DOF sensor 

  mitk::Point3D midpoint;
  midpoint[0] = (position1[0] + position2[0]) / 2.0;
  midpoint[1] = (position1[1] + position2[1]) / 2.0;
  midpoint[2] = (position1[2] + position2[2]) / 2.0;
  
  newsensor->SetPosition(midpoint);

  //Orientnation imaginary 6DOF sensor 

  vtkQuaternion<double> vtkMidOri = vtkQuat1.Slerp(0.5, vtkQuat2);

  double dx = position2[0] - position1[0];
  double dy = position2[1] - position1[1];
  double yaw_m = std::atan2(dy, dx);

  vtkQuaternion<double> vtkZ(yaw_m, 0, 0, 1);

  vtkQuaternion<double> combined = vtkZ * vtkMidOri;
  
  double angleini = vtkMidOri.GetW();
  double xini = vtkMidOri.GetX();
  double yini = vtkMidOri.GetY();
  double zini = vtkMidOri.GetZ();

  mitk::Quaternion MidOri(xini, yini, zini, angleini);

  double r = combined.GetW();
  double x = combined.GetX();
  double yin = combined.GetY();
  double zin = combined.GetZ();
 
  mitk::Quaternion finalOrientation_commbined(x, yin, zin, r);

  newsensor->SetOrientation(MidOri);

  return newsensor;
}



void EndoscopeVisualization::PerformCalculation2()
{
  MITK_INFO << "Averaging Positions and adding point offset the x-axis ";

  mitk::NavigationData::Pointer node1 = mitk::NavigationData::New();
  mitk::NavigationData::Pointer node2 = mitk::NavigationData::New();
  mitk::NavigationData::Pointer node3 = mitk::NavigationData::New();
  node1 = CalculateMidpointAndOrientation(m_NavigationDataList[0], m_NavigationDataList[1]);
  node2 = CalculateMidpointAndOrientation(m_NavigationDataList[2], m_NavigationDataList[3]);
  node3 = CalculateMidpointAndOrientation(m_NavigationDataList[4], m_NavigationDataList[5]);
  m_NodeList.push_back(node1);
  m_NodeList.push_back(node2);
  m_NodeList.push_back(node3);


  for (size_t i = 0; i < m_NodeList.size(); ++i)
  {
    mitk::Point3D position = m_NodeList[i]->GetPosition();
    if (position[0] != 0)
    { 
      points->InsertNextPoint(position[0], position[1], position[2]);

      mitk::Quaternion orientation = m_NodeList[i]->GetOrientation();
      
      orientation.normalize();
     
      itk::Matrix<double, 3, 3> rotationMatrix;
      rotationMatrix = orientation.rotation_matrix_transpose().transpose();

      itk::Vector<double, 3> z;
      z[0] = 0.0;
      z[1] = 0.0;
      z[2] = -1.0;

      itk::Vector<double, 3> zAxisLocal = rotationMatrix * z;
      itk::Vector<double, 3> displacement = zAxisLocal * 3;

      mitk::Point3D newPosition;
      newPosition[0] = position[0] + displacement[0];
      newPosition[1] = position[1] + displacement[1];
      newPosition[2] = position[2] + displacement[2];

      MITK_INFO << newPosition;
      points->InsertNextPoint(newPosition[0], newPosition[1], newPosition[2]);
    }
  }
}


void EndoscopeVisualization::PerformCalculation3()
{
  MITK_INFO << "Averaging 2 Splines";

  pointseven = vtkSmartPointer<vtkPoints>::New();
  pointsuneven = vtkSmartPointer<vtkPoints>::New();

  for (size_t i = 0; i < m_NavigationDataList.size(); ++i)
  {
    mitk::Point3D position = m_NavigationDataList[i]->GetPosition();
    if (i % 2 == 0)
    {
      pointseven->InsertNextPoint(position[0], position[1], position[2]);
    }
    else
    {
      pointsuneven->InsertNextPoint(position[0], position[1], position[2]);
    }
  }
  
    MITK_INFO << "EVEN";

  points->DeepCopy(pointseven);
  PerformInterpolation(m_selectedInterpolationType);

  vtkPoints *pointscopy = spline->GetPoints();
  splineEven->SetPoints(pointscopy);

  MITK_INFO << "UNEVEN";
  
  points->DeepCopy(pointsuneven);
  PerformInterpolation(m_selectedInterpolationType);

  vtkPoints *pointscopyun = spline->GetPoints();
  splineEven->SetPoints(pointscopyun);

  MITK_INFO << "AVERAGE";
  
  vtkSmartPointer<vtkPoints> averagedPoints = vtkSmartPointer<vtkPoints>::New();

  int numSteps = 10;

  points = vtkSmartPointer<vtkPoints>::New();


  for (int i = 0; i < numSteps; ++i)
  {
    double t_value = i / double(numSteps - 1);
    double* t = &t_value;
    double point1[3];
    double Du[9];

    splineEven->Evaluate(t, point1, Du);
    double point2[3];
    splineUneven->Evaluate(t, point2, Du);

    double midpoint[3];
    for (int j = 0; j < 3; ++j)
    {
      midpoint[j] = point1[j] + 0.5 * (point2[j] - point1[j]);
    }
    MITK_INFO << "Point:" << midpoint;
    points->InsertNextPoint(midpoint);
  }
  
}


void EndoscopeVisualization::PerformInterpolation_Parametric()
{
  MITK_INFO << "ParametricSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkParametricSpline> spline;
  spline->SetPoints(points);

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->SetWResolution(50 * numberOfPoints);
  functionSource->Update();
}


void EndoscopeVisualization::PerformInterpolation_Kochanek()
{
  MITK_INFO << "KochanekSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkKochanekSpline> xSpline;
  xSpline->SetDefaultBias(0);
  xSpline->SetDefaultTension(0);
  xSpline->SetDefaultContinuity(0);
  vtkNew<vtkKochanekSpline> ySpline;
  ySpline->SetDefaultBias(0);
  ySpline->SetDefaultTension(0);
  ySpline->SetDefaultContinuity(0);
  vtkNew<vtkKochanekSpline> zSpline;
  zSpline->SetDefaultBias(0);
  zSpline->SetDefaultTension(0);
  zSpline->SetDefaultContinuity(0);
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(points);

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->SetWResolution(50 * numberOfPoints);
  functionSource->Update();
} 
 

void EndoscopeVisualization::PerformInterpolation_Cardinal()
{
  MITK_INFO << "CardinalSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkCardinalSpline> xSpline;
  vtkNew<vtkCardinalSpline> ySpline;
  vtkNew<vtkCardinalSpline> zSpline;
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(points);

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->SetWResolution(50 * numberOfPoints);
  functionSource->Update();
}


void EndoscopeVisualization::PerformInterpolation_SCurve()
{
  MITK_INFO << "SCurveSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkSCurveSpline> xSpline;
  vtkNew<vtkSCurveSpline> ySpline;
  vtkNew<vtkSCurveSpline> zSpline;
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(points);

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->SetWResolution(50 * numberOfPoints);
  functionSource->Update();
}


void EndoscopeVisualization::PerformInterpolation5()
{

}


void EndoscopeVisualization::VisualizeSpline() 
{
  MITK_INFO << "VisualizeSpline";

  vtkNew<vtkNamedColors> colors;
  vtkNew<vtkPolyDataMapper> splineMapper;
  splineMapper->SetInputConnection(functionSource->GetOutputPort());

  actorSpline = vtkNew<vtkActor>();
  actorSpline->SetMapper(splineMapper);
  actorSpline->GetProperty()->SetColor(colors->GetColor3d("DarkSlateGrey").GetData());
  actorSpline->GetProperty()->SetLineWidth(3.0);

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer()->AddActor(actorSpline);
}

void EndoscopeVisualization::VisualizePoints()
{
  MITK_INFO << "VisualizePoints";

  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);

  vtkNew<vtkVertexGlyphFilter> glyphFilter;
  glyphFilter->SetInputData(polyData);
  glyphFilter->Update();

  vtkNew<vtkPolyDataMapper> pointMapper;
  pointMapper->SetInputConnection(glyphFilter->GetOutputPort());

  actorPoints = vtkNew<vtkActor>();
  actorPoints->SetMapper(pointMapper);
  actorPoints->GetProperty()->SetPointSize(4);
  actorPoints->GetProperty()->SetColor(colors->GetColor3d("Peacock").GetData());

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)
    ->GetQmitkRenderWindow("3d")
    ->GetRenderer()
    ->GetVtkRenderer()
    ->AddActor(actorPoints);
}

void EndoscopeVisualization::VisualizeTube() 
{

  MITK_INFO << "VisualizeTube";

  vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
  tubeFilter->SetInputConnection(functionSource->GetOutputPort());
  tubeFilter->SetRadius(m_selectedTubeDiameter / 2);
  tubeFilter->SetNumberOfSides(20);
  tubeFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(tubeFilter->GetOutputPort());

  actorTube = vtkNew<vtkActor>();
  actorTube->SetMapper(mapper);

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer()->AddActor(actorTube);
}