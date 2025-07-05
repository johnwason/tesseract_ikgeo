cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include here
find_package(Eigen3 REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_common REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          #-Werror=conversion
          #-Werror=sign-conversion
          -Wno-sign-compare
          -Werror=non-virtual-dtor)

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# ikgeo
option(TESSERACT_BUILD_IKGEO "Build ikgeo components" ON)
if(TESSERACT_BUILD_IKGEO)
  message("Building ikgeo components")
  list(APPEND SUPPORTED_COMPONENTS ikgeo)
  add_subdirectory(tesseract_kinematics/ikgeo)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_KINEMATICS_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

configure_package()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_KINEMATICS_ENABLE_TESTING)
  foreach(dir urdf)
    install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
  endforeach()

endif()
