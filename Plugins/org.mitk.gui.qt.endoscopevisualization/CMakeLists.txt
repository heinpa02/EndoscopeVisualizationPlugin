project(org_mitk_gui_qt_endoscopevisualization)

mitk_create_plugin(
  EXPORT_DIRECTIVE ENDOSCOPEVISUALIZATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkIGT MitkIGTUI MitkOpenIGTLink MitkOpenIGTLinkUI
)
