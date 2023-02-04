/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/
#ifndef MITKMONAILABELTOOL_H
#define MITKMONAILABELTOOL_H

#include "mitkSegWithPreviewTool.h"
#include <MitkSegmentationExports.h>
#include <memory>
#include <unordered_map>
#include <set>
#include <httplib.h>
#include <nlohmann/json.hpp>


namespace us
{
  class ModuleResource;
}

namespace mitk
{
  struct MonaiModelInfo
  {
    std::string name;
    std::string type;
    std::unordered_map<std::string, int> labels;
    int dimension;
    std::string description;
    std::unordered_map<bool, std::string> config; //TODO: find the full extent
  };

  struct MonaiAppMetadata
  {
    std::string hostName;
    int port;
    std::string origin;
    std::string name;
    std::string description;
    std::string version;
    std::vector<std::string> labels;
    std::vector<MonaiModelInfo> models;
  };

  struct MonaiLabelRequest
  {
    /*Parameters that goes into infer POST*/
    MonaiModelInfo model;
    std::string hostName;
    int port;
  };


  class MITKSEGMENTATION_EXPORT MonaiLabelTool : public SegWithPreviewTool
  {
  public:
    mitkClassMacro(MonaiLabelTool, SegWithPreviewTool);
    itkFactorylessNewMacro(Self);
    itkCloneMacro(Self);
    bool m_TEST = false; // cleanup later

    const char *GetName() const override;
    const char **GetXPM() const override;
    us::ModuleResource GetIconResource() const override;

    void Activated() override;
    void GetOverallInfo(std::string&, int&);
    std::unique_ptr<MonaiAppMetadata> m_InfoParameters; //contains all parameters from Server to serve the GUI
    std::unique_ptr<MonaiLabelRequest> m_RequestParameters;
    std::vector<MonaiModelInfo> GetAutoSegmentationModels();
    std::vector<MonaiModelInfo> GetInteractiveSegmentationModels();
    std::vector<MonaiModelInfo> GetScribbleSegmentationModels();
    void PostInferRequest(std::string &, int &, std::string &, std::string &);

    itkSetMacro(ModelName, std::string);
    itkGetConstMacro(ModelName, std::string);
    itkSetMacro(URL, std::string);
    itkGetConstMacro(URL, std::string);
    itkSetMacro(MitkTempDir, std::string);
    itkGetConstMacro(MitkTempDir, std::string);

  protected:
    MonaiLabelTool();
    ~MonaiLabelTool();
    void DoUpdatePreview(const Image* inputAtTimeStep, const Image* oldSegAtTimeStep, LabelSetImage* previewImage, TimeStepType timeStep) override;
   
  private:
    std::string m_MitkTempDir;
    std::vector<std::string> getPartsBetweenBoundary(const std::string &, const std::string &);
    std::unique_ptr<MonaiAppMetadata> mitk::MonaiLabelTool::DataMapper(nlohmann::json&);
    bool IsMonaiServerOn(std::string &, int &);
    std::string m_ModelName;
    std::string m_URL;
    nlohmann::json m_ResultMetadata;
    const std::set<std::string> m_AUTO_SEG_TYPE_NAME = {"segmentation"};
    const std::set<std::string> m_SCRIBBLE_SEG_TYPE_NAME = {"scribbles"};
    const std::set<std::string> m_INTERACTIVE_SEG_TYPE_NAME = {"deepedit", "deepgrow"};
    const std::string m_TEMPLATE_FILENAME = "XXXXXX_000_0000.nii.gz";
    const std::string m_SERVER_503_ERROR_TEXT = "A connection to MonaiLabel server cannot be established.";
  }; // class
} // namespace
#endif
