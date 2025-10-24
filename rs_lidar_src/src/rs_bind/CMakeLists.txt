find_package(pybind11 REQUIRED)
if(NOT pybind11_FOUND)
    message(FATAL_ERROR "pybind11 not found. 请安装pybind11: pip install pybind11")
endif()

file(GLOB RS_BINDINGS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/rs_bindings.cpp)

pybind11_add_module(
    rs_lidar  
    SHARED
    ${RS_BINDINGS_SRC}
  )

  target_include_directories(
    rs_lidar PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(
    rs_lidar PRIVATE
    pybind11::module
    rs_reader
    rs_viewer
    pthread
  )
  
  set_target_properties(
    rs_lidar 
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    PREFIX ""
  )