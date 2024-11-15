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
#include <mitkPoint.h>

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
#include <vtkParametricSpline.h>
#include <vtkKochanekSpline.h>
#include <vtkCardinalSpline.h>
#include <vtkSCurveSpline.h>
#include <vtkTubeFilter.h>

#include <vtkParametricFunctionSource.h>
#include <vtkNamedColors.h>
#include <vtkVertexGlyphFilter.h>
#


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
  connect(m_Controls.Interpol5, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
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

  m_Timer->start(100);  
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
  else if (m_Controls.Interpol5->isChecked())
  {
    MITK_INFO << "Interpolation 5 ";
    m_selectedInterpolationType = 5;
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
  
  m_NavigationDataList.clear();
  for (size_t i = 0; i <= m_Controls.widget->GetSelectedToolID(); ++i)
  {
    mitk::NavigationData::Pointer m_NavigationDataSensor;
    m_NavigationDataSensor = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(i);
    mitk::Point3D position = m_NavigationDataSensor->GetPosition();
    mitk::Quaternion orientation = m_NavigationDataSensor->GetOrientation();
    MITK_INFO << "Sensor: " << m_NavigationDataSensor->GetName() << " Position: " << position << " Orientation: " << orientation;

    m_NavigationDataList.push_back(m_NavigationDataSensor);
  }

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


  PerformCalculation(m_selectedCalculationType);

  points->SetNumberOfPoints(m_NavigationDataList.size());
  for (size_t i = 0; i < m_NavigationDataList.size(); ++i)
  {
    mitk::Point3D position = m_NavigationDataList[i]->GetPosition();
    points->SetPoint(i, position[0], position[1], position[2]);
  }

  PerformInterpolation(m_selectedInterpolationType);
  VisualizePoints();
  VisualizeSpline();
  VisualizeTube();

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->RequestUpdate();

}


void EndoscopeVisualization::PerformCalculation(int calculationType) 
{
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
      PerformInterpolation4();
      break;
    case 5:
      PerformInterpolation5();
      break;
  }
}


void EndoscopeVisualization::PerformCalculation1()
{

}


void EndoscopeVisualization::PerformCalculation2()
{
  MITK_INFO << "Executing Calculation 2...";
  //
}


void EndoscopeVisualization::PerformCalculation3()
{
  MITK_INFO << "Executing Calculation 3...";
  //
}


void EndoscopeVisualization::PerformInterpolation_Parametric()
{
  MITK_INFO << "ParametricSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkParametricSpline> spline;
  spline->SetPoints(points);

  functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->Update();
}


void EndoscopeVisualization::PerformInterpolation_Kochanek()
{
  MITK_INFO << "KochanekSpline";

  vtkNew<vtkNamedColors> colors;
  int numberOfPoints = points->GetNumberOfPoints();

  vtkNew<vtkKochanekSpline> xSpline;
  vtkNew<vtkKochanekSpline> ySpline;
  vtkNew<vtkKochanekSpline> zSpline;
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(points);

  functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
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

  functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * numberOfPoints);
  functionSource->SetVResolution(50 * numberOfPoints);
  functionSource->Update();
}


void EndoscopeVisualization::PerformInterpolation4()
{
  
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

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer()->AddActor(actorPoints);
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