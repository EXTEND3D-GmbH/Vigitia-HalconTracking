# DLL
set(sources  ImGuiFileDialog.cpp
             ImGuiFileDialog.h
             ImGuiFileDialogConfig.h
             dirent/dirent.h )

add_library(imgui_filedialog STATIC ${sources})

target_include_directories(
  imgui_filedialog
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)
  
target_link_libraries(imgui_filedialog PRIVATE imgui)
target_compile_definitions(imgui_filedialog PUBLIC MSVC)
target_compile_features(imgui_filedialog PRIVATE cxx_std_17)