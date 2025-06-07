# When integrating into tesseract_kinematics remove next line
find_package(tesseract_kinematics REQUIRED)

add_library(${PROJECT_NAME}_ikgeo src/ikgeo_inv_kin_chain.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ikgeo
  # When integrating into tesseract_kinematics restore lines below and remove tesseract_kinematics_core and
  # tesseract_kinematics_kdl PUBLIC ${PROJECT_NAME}_core ${PROJECT_NAME}_kdl
  PUBLIC boost_plugin_loader::boost_plugin_loader
         tesseract::tesseract_kinematics_core
         tesseract::tesseract_kinematics_kdl
         Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_ikgeo PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ikgeo PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikgeo PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ikgeo ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ikgeo PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikgeo
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ikgeo PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")
if(0)
# Add ikgeo kinematics factory
add_library(${PROJECT_NAME}_ikgeo_factory src/ikgeo_factory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ikgeo_factory
  PUBLIC ${PROJECT_NAME}_ikgeo
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_ikgeo_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ikgeo_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikgeo_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ikgeo_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ikgeo_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikgeo_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_ikgeo_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                         "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find this factory by default
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "${PROJECT_NAME}_ikgeo_factory" PARENT_SCOPE)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT ikgeo
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT ikgeo
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_ikgeo ${PROJECT_NAME}_ikgeo_factory
  DEPENDENCIES ikgeo_lib "tesseract_kinematics COMPONENTS core")
endif()