# DLL
set(sources  
  imconfig.h
  imgui.cpp
  imgui.h
  imgui_demo.cpp
  imgui_draw.cpp
  imgui_impl_glfw.cpp
  imgui_impl_glfw.h
  imgui_impl_opengl3.cpp
  imgui_impl_opengl3.h
  imgui_internal.h
  imgui_tables.cpp
  imgui_widgets.cpp )

add_library(imgui STATIC ${sources})
target_link_libraries(imgui PRIVATE glfw)
target_include_directories(
  imgui
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)
  