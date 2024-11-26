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

#include <vtkCellLocator.h>
#include <vtkPolyData.h>
#include <vtkMath.h>


const std::string EndoscopeVisualization::VIEW_ID = "org.mitk.views.endoscopevisualization";


EndoscopeVisualization::EndoscopeVisualization()
  : QmitkAbstractView(), m_Source(nullptr)
{}


EndoscopeVisualization::~EndoscopeVisualization() {}


void EndoscopeVisualization::SetFocus()
{
  m_Controls.button_performEndoscopeVisualization->setFocus();
}


void EndoscopeVisualization::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  //Initialize and connect Timer
  m_Timer = new QTimer(this);
  connect(m_Timer, &QTimer::timeout, this, &EndoscopeVisualization::UpdateTrackingData);

  // VisualizeEndoscope button
  connect(m_Controls.button_performEndoscopeVisualization, &QPushButton::clicked, this, &EndoscopeVisualization::VisualizeEndoscope);

  // NavigationDataSourceSelectionWidget
  connect(m_Controls.widget, SIGNAL(NavigationDataSourceSelected(mitk::NavigationDataSource::Pointer)), this, SLOT(SetupNavigation()));

  // CalculationSelection radiobuttons
  connect(m_Controls.button_Calculation1, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  connect(m_Controls.button_Calculation2, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  connect(m_Controls.button_Calculation3, &QRadioButton::clicked, this, &EndoscopeVisualization::CalculationSelected);
  m_Controls.button_Calculation1->setChecked(true);

  // InterpolationSelection radiobuttons
  connect(m_Controls.button_InterpolationCardinal, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected); 
  connect(m_Controls.button_InterpolationKochanek, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  connect(m_Controls.button_Interpolation_SCurve, &QRadioButton::clicked, this, &EndoscopeVisualization::InterpolationSelected);
  m_Controls.button_InterpolationCardinal->setChecked(true);

  // TubeDiameterSelection spinbox
  connect(m_Controls.spinBox_tubeDiameter, QOverload<int>::of(&QSpinBox::valueChanged), this, &EndoscopeVisualization::TubeDiameterChanged);
  connect(m_Controls.checkBox_tube, &QCheckBox::toggled, this, &EndoscopeVisualization::TubeCheckboxToggled);

  //PointSetRecording
  connect(m_Controls.checkBox_pointSetRecording, &QCheckBox::toggled, this, &EndoscopeVisualization::RecordPointSet);
  connect(m_Controls.button_performEvaluation, &QPushButton::clicked, this, &EndoscopeVisualization::PerformEvaluation);
  m_Controls.radioButton_spline->setChecked(true);
  
}



void EndoscopeVisualization::VisualizeEndoscope()
{
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
  if (m_Controls.button_Calculation1->isChecked())
  {
    MITK_INFO << "Calculation 1";
    m_selectedCalculationType = 1;
  }
  else if (m_Controls.button_Calculation2->isChecked())
  {
    MITK_INFO << "Calculation 2 ";
    m_selectedCalculationType = 2;
  }
  else if (m_Controls.button_Calculation3->isChecked())
  {
    MITK_INFO << "Calculation 3 ";
    m_selectedCalculationType = 3;
  }
}


void EndoscopeVisualization::InterpolationSelected()
{
  if (m_Controls.button_InterpolationCardinal->isChecked())
  {
    MITK_INFO << "Interpolation 1 ";
    m_selectedInterpolationType = 1;
  }
  else if (m_Controls.button_InterpolationKochanek->isChecked())
  {
    MITK_INFO << "Interpolation 2 ";
    m_selectedInterpolationType = 2;
  }
  else if (m_Controls.button_Interpolation_SCurve->isChecked())
  {
    MITK_INFO << "Interpolation 3 ";
    m_selectedInterpolationType = 3;
  }
}



void EndoscopeVisualization::TubeDiameterChanged(int tubediameter) {
  m_selectedTubeDiameter = tubediameter;
}


void EndoscopeVisualization::TubeCheckboxToggled(bool checked)
{
  if (checked)
  {
    m_TubeActivated = true;
  }
  else
  {
    m_TubeActivated = false;
  }
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
  m_NavigationDataList.clear();
  if (m_Controls.widget->GetSelectedToolID() < 6)
  {
    for (size_t i = 0; i <= m_Controls.widget->GetSelectedToolID(); ++i)
    {
      mitk::NavigationData::Pointer m_NavigationDataSensor;
      m_NavigationDataSensor = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(i);
      m_NavigationDataList.push_back(m_NavigationDataSensor);
    }
  }
  else
  {
    mitk::NavigationData::Pointer m_NavigationDataSensor0 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(0);
    m_NavigationDataList.push_back(m_NavigationDataSensor0);
    mitk::NavigationData::Pointer m_NavigationDataSensor1 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(1);
    m_NavigationDataList.push_back(m_NavigationDataSensor1);
    mitk::NavigationData::Pointer m_NavigationDataSensor2 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(2);
    m_NavigationDataList.push_back(m_NavigationDataSensor2);
    mitk::NavigationData::Pointer m_NavigationDataSensor3 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(4);
    m_NavigationDataList.push_back(m_NavigationDataSensor3);
    mitk::NavigationData::Pointer m_NavigationDataSensor4 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(5);
    m_NavigationDataList.push_back(m_NavigationDataSensor4);
    mitk::NavigationData::Pointer m_NavigationDataSensor5 =
      m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(6);
    m_NavigationDataList.push_back(m_NavigationDataSensor5);
  }

  PerformCalculation(m_selectedCalculationType);

  spline = PerformInterpolation(points, m_selectedInterpolationType);

  VisualizePoints();

  VisualizeSpline();

  if (m_TubeActivated)
  {
    VisualizeTube();
  }

  this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->RequestUpdate();

  if (m_RecordingActive)
  {
    int size = m_RecordedPointSet->GetSize();
    mitk::NavigationData::Pointer nd = m_RecordedNavigationData;

    if (size > 0)
    {
      mitk::Point3D p = m_RecordedPointSet->GetPoint(size - 1);
      if (p.EuclideanDistanceTo(nd->GetPosition()) > (double)m_Controls.spinBox_pointSetRecording->value())
        m_RecordedPointSet->InsertPoint(size, nd->GetPosition());
    }
    else
      m_RecordedPointSet->InsertPoint(size, nd->GetPosition());
  }

}




void EndoscopeVisualization::PerformCalculation(int calculationType) 
{
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
    case 4:
      PerformCalculation4();
      break;
  }
}


vtkSmartPointer<vtkParametricSpline> EndoscopeVisualization::PerformInterpolation(vtkSmartPointer<vtkPoints> punkte, int interpolationType)
{
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
    vtkRenderer *vtkRenderer = this->GetRenderWindowPart(mitk::WorkbenchUtil::OPEN)->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer();

      if (pointSetNode.IsNotNull() && datastorage)
    {
      datastorage->Remove(pointSetNode);
      pointSetNode = nullptr; 
    }

    if (splineNode.IsNotNull() && datastorage)
    {
      datastorage->Remove(splineNode);
      splineNode = nullptr; 
    }

    if (tubeNode.IsNotNull() && datastorage)
    {
      datastorage->Remove(tubeNode);
      tubeNode = nullptr;
    }

  
  switch (interpolationType)
  {
    case 1:
      spline = PerformInterpolation_Kochanek(punkte);
      break;
    case 2:
      spline = PerformInterpolation_Cardinal(punkte);
      break;
    case 3:
      spline = PerformInterpolation_SCurve(punkte);
      break;
  }

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(m_resolution);
  functionSource->SetVResolution(m_resolution);
  functionSource->SetWResolution(m_resolution);
  functionSource->Update();

  return spline;
}


void EndoscopeVisualization::PerformCalculation1()
{

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

  double halfYaw = yaw_m / 2.0;
  vtkQuaternion<double> vtkYaw(cos(halfYaw), 0, 0, sin(halfYaw));

  vtkQuaternion<double> combined = vtkMidOri * vtkYaw;
  
  double angleini = vtkMidOri.GetW();
  double xini = vtkMidOri.GetX();
  double yini = vtkMidOri.GetY();
  double zini = vtkMidOri.GetZ();

  mitk::Quaternion MidOri(xini, yini, zini, angleini);

  double r = combined.GetW();
  double x = combined.GetX();
  double yin = combined.GetY();
  double zin = combined.GetZ();
 
  mitk::Quaternion finalOrientation(x, yin, zin, r);

  newsensor->SetOrientation(finalOrientation);

  return newsensor;
}



void EndoscopeVisualization::PerformCalculation2()
{

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

      double offset = m_Controls.spinBox_offset->value();

      itk::Vector<double, 3> z;
      z[0] = 0.0;
      z[1] = 0.0;
      z[2] = -1.0;

      itk::Vector<double, 3> zAxisLocal = rotationMatrix * z;

      zAxisLocal = -zAxisLocal;

      itk::Vector<double, 3> displacement = zAxisLocal * offset;

      mitk::Point3D newPosition;
      newPosition[0] = position[0] - displacement[0];
      newPosition[1] = position[1] - displacement[1];
      newPosition[2] = position[2] - displacement[2];

      points->InsertNextPoint(newPosition[0], newPosition[1], newPosition[2]);
    }
  }
}


void EndoscopeVisualization::PerformCalculation3()
{
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

  splineEven = PerformInterpolation(pointseven, m_selectedInterpolationType);
  splineUneven = PerformInterpolation(pointsuneven, m_selectedInterpolationType);

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

    points->InsertNextPoint(midpoint);

  }
}

void EndoscopeVisualization::PerformCalculation4() {

}


vtkSmartPointer<vtkParametricSpline> EndoscopeVisualization::PerformInterpolation_Kochanek(vtkSmartPointer<vtkPoints> punkte)
{
  int numberOfPoints = punkte->GetNumberOfPoints();

  double bias = m_Controls.doubleSpinBox_bias->value();
  double tension = m_Controls.doubleSpinBox_tension->value();
  double continuity = m_Controls.doubleSpinBox_continuity->value();

  vtkNew<vtkKochanekSpline> xSpline;
  xSpline->SetDefaultBias(bias);
  xSpline->SetDefaultTension(tension);
  xSpline->SetDefaultContinuity(continuity);
  vtkNew<vtkKochanekSpline> ySpline;
  ySpline->SetDefaultBias(bias);
  ySpline->SetDefaultTension(tension);
  ySpline->SetDefaultContinuity(continuity);
  vtkNew<vtkKochanekSpline> zSpline;
  zSpline->SetDefaultBias(bias);
  zSpline->SetDefaultTension(tension);
  zSpline->SetDefaultContinuity(continuity);

  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(punkte);

  m_resolution = 50 * numberOfPoints;

  return spline;
} 
 

vtkSmartPointer<vtkParametricSpline> EndoscopeVisualization::PerformInterpolation_Cardinal(vtkSmartPointer<vtkPoints> punkte)
{
  int numberOfPoints = punkte->GetNumberOfPoints();

  vtkNew<vtkCardinalSpline> xSpline;
  vtkNew<vtkCardinalSpline> ySpline;
  vtkNew<vtkCardinalSpline> zSpline;
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);

  spline->SetPoints(punkte);

  m_resolution = 50 * numberOfPoints;

  return spline;
}


vtkSmartPointer<vtkParametricSpline> EndoscopeVisualization::PerformInterpolation_SCurve(vtkSmartPointer<vtkPoints> punkte)
{
  int numberOfPoints = punkte->GetNumberOfPoints();

  vtkNew<vtkSCurveSpline> xSpline;
  vtkNew<vtkSCurveSpline> ySpline;
  vtkNew<vtkSCurveSpline> zSpline;
  vtkNew<vtkParametricSpline> spline;

  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(punkte);

  m_resolution = 50 * numberOfPoints;

  return spline;
}


void EndoscopeVisualization::VisualizePoints()
{
  mitk::PointSet::Pointer pointSet = mitk::PointSet::New();

  for (size_t i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    mitk::Point3D point;
    point[0] = points->GetPoint(i)[0];
    point[1] = points->GetPoint(i)[1];
    point[2] = points->GetPoint(i)[2];

    pointSet->InsertPoint(i, point);
  }

  if (!pointSetNode)
  {
    pointSetNode = mitk::DataNode::New();
    datastorage->Add(pointSetNode);
    pointSetNode->SetName("Nodes");
    pointSetNode->GetPropertyList()->SetProperty("color", mitk::ColorProperty::New(0.0, 1.0, 0.0));
    pointSetNode->GetPropertyList()->SetProperty("point size", mitk::FloatProperty::New(10.0));
  }

  pointSetNode->SetData(pointSet);

}

void EndoscopeVisualization::VisualizeSpline() 
{
  vtkSmartPointer<vtkPolyData> polyData = functionSource->GetOutput();

  mitk::Surface::Pointer splineSurface = mitk::Surface::New();
  splineSurface->SetVtkPolyData(polyData);

  
  if (!splineNode)
  {
    splineNode = mitk::DataNode::New();
    datastorage->Add(splineNode);
    splineNode->SetName("Spline");
    splineNode->GetPropertyList()->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 0.0));
    splineNode->GetPropertyList()->SetProperty("line width", mitk::FloatProperty::New(10.0));
  }

   splineNode->SetData(splineSurface);

}


void EndoscopeVisualization::VisualizeTube() 
{
  vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
  tubeFilter->SetInputConnection(functionSource->GetOutputPort());
  tubeFilter->SetRadius(m_selectedTubeDiameter / 2.0);
  tubeFilter->SetNumberOfSides(20);
  tubeFilter->Update();

  mitk::Surface::Pointer tubeSurface = mitk::Surface::New();
  tubeSurface->SetVtkPolyData(tubeFilter->GetOutput());

  if (!tubeNode)
  {
    tubeNode = mitk::DataNode::New();
    tubeNode->SetName("Tube");
    tubeNode->GetPropertyList()->SetProperty("color", mitk::ColorProperty::New(1, 1, 1));
    tubeNode->GetPropertyList()->SetProperty("opacity", mitk::FloatProperty::New(0.5));
    datastorage->Add(tubeNode);
  }

  tubeNode->SetData(tubeSurface);
 
}

void EndoscopeVisualization::RecordPointSet()
{
  if (m_Controls.checkBox_pointSetRecording->isChecked())
  {
    if (m_Controls.widget_2->GetSelectedToolID() == -1)
    {
      QMessageBox::warning(nullptr, "Error", "No tool selected for point set recording!");
      m_Controls.checkBox_pointSetRecording->setChecked(false);
      return;
    }
    m_RecordedNavigationData = m_Controls.widget_2->GetSelectedNavigationDataSource()->GetOutput(m_Controls.widget_2->GetSelectedToolID());

    mitk::DataNode::Pointer dn = datastorage->GetNamedNode("Evaluation Points");
    if (m_RecordedPointSet.IsNull() || dn.IsNull())
    {
      m_RecordedPointSet = nullptr;
      m_RecordedPointSet = mitk::PointSet::New();
      mitk::DataNode::Pointer EvaluationDataNode = mitk::DataNode::New();
      EvaluationDataNode->SetName("Evaluation Points");
      EvaluationDataNode->GetPropertyList()->SetProperty("color", mitk::ColorProperty::New(1.0, 0.0, 0.0));
      EvaluationDataNode->GetPropertyList()->SetProperty("point size", mitk::FloatProperty::New(100.0f));
      EvaluationDataNode->SetData(m_RecordedPointSet);
      datastorage->Add(EvaluationDataNode);
    }
    else
    {
      m_RecordedPointSet->Clear();
    }
    m_RecordingActive = true;
  }
  else
  {
    m_RecordingActive = false;
  }
}

void EndoscopeVisualization::PerformEvaluation()
{
  mitk::DataNode::Pointer evaPointSetNode = datastorage->GetNamedNode("Evaluation Points");
  mitk::PointSet::Pointer evaPointSet = dynamic_cast<mitk::PointSet *>(evaPointSetNode->GetData());

  std::string node;
  if (m_Controls.radioButton_spline->isChecked() == true)
  {
    node = "Spline";
  }
  else if (m_Controls.radioButton_tube->isChecked() == true)
  {
    node = "Tube";
  } 
 
  mitk::DataNode::Pointer evaSplineNode = datastorage->GetNamedNode(node);
  mitk::Surface::Pointer splineSurface = dynamic_cast<mitk::Surface *>(evaSplineNode->GetData());
  vtkSmartPointer<vtkPolyData> polyDataEval = splineSurface->GetVtkPolyData();

  vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
  cellLocator->SetDataSet(polyDataEval);
  cellLocator->BuildLocator();

  int numPoints = evaPointSet->GetSize();
  double totalDistance = 0.0;
  double totalSquaredDistance = 0.0;
  std::vector<double> distances;

  for (int i = 0; i < numPoints; ++i)
  {
    mitk::Point3D point3D = evaPointSet->GetPoint(i);
    const double point[3] = {point3D[0], point3D[1], point3D[2]};

    double closestPoint[3];
    double closestDistance2 = VTK_DOUBLE_MAX;

    vtkGenericCell *cell = vtkGenericCell::New();
    vtkIdType cellId;
    int subId;

    cellLocator->FindClosestPoint(point, closestPoint, cell, cellId, subId, closestDistance2);

    double distance = std::sqrt(closestDistance2);
    
    totalDistance += distance;
    totalSquaredDistance += closestDistance2;

    distances.push_back(distance);

    cell->Delete();
  }

  double meanDistance = totalDistance / numPoints;

  double mse = totalSquaredDistance / numPoints;

  double rmse = std::sqrt(mse);

  double sumSquaredDifferences = 0.0;
  for (double dist : distances)
  {
    sumSquaredDifferences += std::pow(dist - meanDistance, 2);
  }
  double standardDeviation = std::sqrt(sumSquaredDifferences / numPoints);

  m_Controls.output->setText("Mean Distance: " + QString::number(meanDistance) + +"<br>" +
                             " MSE: " + QString::number(mse) + "<br>" +
                            " RMSE: " + QString::number(rmse) + "<br>" +
                            " Standard Deviation: " + QString::number(standardDeviation));

}



