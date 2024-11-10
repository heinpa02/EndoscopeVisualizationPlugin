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



void EndoscopeVisualization::InterpolationSelected()
{
  if (m_Controls.Interpol1->isChecked())
  {
    MITK_INFO << "Interpolation 1";
  }
  else if (m_Controls.Interpol2->isChecked())
  {
    MITK_INFO << "Interpolation 2 ";
  }
  else if (m_Controls.Interpol3->isChecked())
  {
    MITK_INFO << "Interpolation 3 ";
  }
  else if (m_Controls.Interpol4->isChecked())
  {
    MITK_INFO << "Interpolation 4 ";
  }
  else if (m_Controls.Interpol5->isChecked())
  {
    MITK_INFO << "Interpolation 5 ";
  }
}



void EndoscopeVisualization::TubeDiameterChanged(int tubediameter) {
  MITK_INFO << "Durchmesser= " << tubediameter;
}



void EndoscopeVisualization::SetupNavigation()
{
  if (m_Source.IsNotNull())
    if (m_Source->IsTracking())
      return;
    MITK_INFO << "Please start Tracking first.";
  MITK_INFO << "Please select a Tracking Device first.";
}



void EndoscopeVisualization::UpdateTrackingData()
{
  MITK_INFO << "UpdateTrackingData wird erreicht.";

  // Get all data nodes in the data storage
  mitk::DataStorage::SetOfObjects::ConstPointer allNodes = datastorage->GetAll();

  // Iterate through all data nodes and check for navigation data
  for (auto node : *allNodes)
  {
    if (node.IsNotNull())
    {
      // Attempt to cast the node data to NavigationData
      mitk::NavigationData::Pointer navigationData = dynamic_cast<mitk::NavigationData *>(node->GetData());

      // If the node contains valid navigation data, process it
      if (navigationData.IsNotNull())
      {
        // Extract position and orientation from the navigation data
        mitk::Point3D position = navigationData->GetPosition();
        mitk::Quaternion orientation = navigationData->GetOrientation();

        // Example: Print out the extracted data for each sensor
        MITK_INFO << "Sensor: " << node->GetName() << " Position: " << position << " Orientation: " << orientation;

        // m_SensorDataList.push_back(navigationData);
      }
    }
  }
}