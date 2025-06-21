# When integrating into tesseract_kinematics remove next line
find_package(tesseract_kinematics REQUIRED)


set(ikgeo_src
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/utils.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/subproblems/quartic.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/subproblems/sp.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_2_parallel.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_3_parallel_2_intersecting.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_3_parallel.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_spherical_2_intersecting.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_spherical_2_parallel.cpp
    ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK/IK_spherical.cpp
)

add_library(${PROJECT_NAME} src/ikgeo_inv_kin_chain.cpp ${ikgeo_src})
target_link_libraries(
  ${PROJECT_NAME}
  # When integrating into tesseract_kinematics restore lines below and remove tesseract_kinematics_core and
  # tesseract_kinematics_kdl PUBLIC ${PROJECT_NAME}_core ${PROJECT_NAME}_kdl
  PUBLIC boost_plugin_loader::boost_plugin_loader
         tesseract::tesseract_kinematics_core
         tesseract::tesseract_kinematics_kdl
         Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>"
                                                          PRIVATE ${CMAKE_SOURCE_DIR}/ikgeo/cpp/IK
                                                          ${CMAKE_SOURCE_DIR}/ikgeo/cpp/subproblems)
# Add ikgeo kinematics factory
add_library(${PROJECT_NAME}_factory src/ikgeo_factory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_factory
  PUBLIC ${PROJECT_NAME}
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                         "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find this factory by default
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "${PROJECT_NAME}_factory" PARENT_SCOPE)

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
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_factory
  DEPENDENCIES ikgeo_lib "tesseract_kinematics COMPONENTS core")
