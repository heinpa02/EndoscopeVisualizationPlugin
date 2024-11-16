project(org_mitk_gui_qt_endoscopevisualization)

mitk_create_plugin(
  EXPORT_DIRECTIVE ENDOSCOPEVISUALIZATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  PACKAGE_DEPENDS ITK VTK VTK|CommonColor VTK|CommonComputationalGeometry VTK|ViewsInfovis VTK|CommonCore VTK|CommonDataModel VTK|FiltersSources VTK|InteractionStyle VTK|RenderingContextOpenGL2 VTK|RenderingCore VTK|RenderingFreeType VTK|RenderingGL2PSOpenGL2 VTK|RenderingOpenGL2 
  MODULE_DEPENDS MitkCore MitkQtWidgetsExt MitkIGT MitkIGTUI MitkOpenIGTLink MitkOpenIGTLinkUI MitkMapperExt 
)
