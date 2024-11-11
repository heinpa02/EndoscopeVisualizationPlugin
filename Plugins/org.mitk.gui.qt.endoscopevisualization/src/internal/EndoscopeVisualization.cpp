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

  m_Timer->start(1000);
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
  m_selectedtubediameter = tubediameter;
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
  
  for (size_t i = 0; i <= m_Controls.widget->GetSelectedToolID(); ++i)
  {
    m_NavigationDataSensor = m_Controls.widget->GetSelectedNavigationDataSource()->GetOutput(i);
    mitk::Point3D position = m_NavigationDataSensor->GetPosition();
    mitk::Quaternion orientation = m_NavigationDataSensor->GetOrientation();
    MITK_INFO << "Sensor: " << m_NavigationDataSensor->GetName() << " Position: " << position << " Orientation: " << orientation;

    m_SensorDataList.push_back(m_NavigationDataSensor);
  }

  PerformCalculation(m_selectedCalculationType);
  PerformInterpolation(m_selectedInterpolationType);
  PerformTube();

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
  switch (interpolationType)
  {
    case 1:
      PerformInterpolation1();
      break;
    case 2:
      PerformInterpolation2();
      break;
    case 3:
      PerformInterpolation3();
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
  MITK_INFO << "Executing Calculation 1...";
  //
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


void EndoscopeVisualization::PerformInterpolation1()
{
  MITK_INFO << "Executing Interpolation 1...";
  for (size_t i = 0; i <= m_Controls.widget->GetSelectedToolID(); ++i)
  {
    mitk::Point3D listposition = m_SensorDataList[i]->GetPosition();
    mitk::Quaternion listorientation = m_SensorDataList[i]->GetOrientation();
    MITK_INFO << " Position: " << listposition << " Orientation: " << listorientation;
  }
}

void EndoscopeVisualization::PerformInterpolation2()
{
  MITK_INFO << "Executing Interpolation 2...";
  // 
}

void EndoscopeVisualization::PerformInterpolation3()
{
  MITK_INFO << "Executing Interpolation 3...";
  // 
}

void EndoscopeVisualization::PerformInterpolation4()
{
  MITK_INFO << "Executing Interpolation 4...";
  // 
}

void EndoscopeVisualization::PerformInterpolation5()
{
  MITK_INFO << "Executing Interpolation 5...";
  // 
}
  

void EndoscopeVisualization::PerformTube() 
{
  MITK_INFO << "Tube in progress.";
}