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

const std::string EndoscopeVisualization::VIEW_ID = "org.mitk.views.endoscopevisualization";

void EndoscopeVisualization::SetFocus()
{
  m_Controls.buttonPerformImageProcessing->setFocus();
}

void EndoscopeVisualization::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  
  // Visualize button
  connect(m_Controls.buttonPerformImageProcessing, &QPushButton::clicked, this, &EndoscopeVisualization::DoImageProcessing);

  // NavigationDataSourceSelectionWidget
  connect(m_Controls.widget, SIGNAL(NavigationDataSourceSelected(mitk::NavigationDataSource::Pointer)), this, SLOT(OnSetupNavigation()));

  // InterpolationSelection radiobuttons
  connect(m_Controls.Interpol1, &QRadioButton::clicked, this, &EndoscopeVisualization::OnInterpolationSelected);
  connect(m_Controls.Interpol2, &QRadioButton::clicked, this, &EndoscopeVisualization::OnInterpolationSelected);
  connect(m_Controls.Interpol3, &QRadioButton::clicked, this, &EndoscopeVisualization::OnInterpolationSelected);
  connect(m_Controls.Interpol4, &QRadioButton::clicked, this, &EndoscopeVisualization::OnInterpolationSelected);
  connect(m_Controls.Interpol5, &QRadioButton::clicked, this, &EndoscopeVisualization::OnInterpolationSelected);

  // TubeDiameterSelection spinbox
  connect(m_Controls.spinBoxTubeDiameter, QOverload<int>::of(&QSpinBox::valueChanged), this, &EndoscopeVisualization::OnTubeDiameterChanged);

}

void EndoscopeVisualization::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  // iterate all selected objects, adjust warning visibility
  foreach (mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
    {
      m_Controls.labelWarning->setVisible(false);
      m_Controls.buttonPerformImageProcessing->setEnabled(true);
      return;
    }
  }

  m_Controls.labelWarning->setVisible(true);
  m_Controls.buttonPerformImageProcessing->setEnabled(false);
}

void EndoscopeVisualization::DoImageProcessing()
{
  QList<mitk::DataNode::Pointer> nodes = this->GetDataManagerSelection();
  if (nodes.empty())
    return;

  mitk::DataNode *node = nodes.front();

  if (!node)
  {
    // Nothing selected. Inform the user and return
    QMessageBox::information(nullptr, "Template", "Please load and select an image before starting image processing.");
    return;
  }

  // here we have a valid mitk::DataNode

  // a node itself is not very useful, we need its data item (the image)
  mitk::BaseData *data = node->GetData();
  if (data)
  {
    // test if this data item is an image or not (could also be a surface or something totally different)
    mitk::Image *image = dynamic_cast<mitk::Image *>(data);
    if (image)
    {
      std::stringstream message;
      std::string name;
      message << "Performing image processing for image ";
      if (node->GetName(name))
      {
        // a property called "name" was found for this DataNode
        message << "'" << name << "'";
      }
      message << ".";
      MITK_INFO << message.str();

      // actually do something here...
    }
  }
}


void EndoscopeVisualization::OnInterpolationSelected() {
  MITK_INFO << "Interpolation";
}

void EndoscopeVisualization::OnTubeDiameterChanged(int tubediameter) {
  MITK_INFO << "Durchmesser";
}
