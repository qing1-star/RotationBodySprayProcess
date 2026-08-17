cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RotationBodyTrajectoryPlanningEditor)
set(CMAKE_AUTOMOC ON)

set(SOURCES
    Adapters/RotationBodyMeshAdapter.cpp
    Adapters/RotationBodyMeshAdapter.h
    Controllers/RotationBodyPlanningController.cpp
    Controllers/RotationBodyPlanningController.h
    Controllers/RotationBodyPlanningWorkflowCoordinator.cpp
    Controllers/RotationBodyPlanningWorkflowCoordinator.h
    Models/RotationBodyPlanningSession.cpp
    Models/RotationBodyPlanningSession.h
    Models/RotationBodyPlanningTranslations.cpp
    Models/RotationBodyPlanningTranslations.h
    Models/RotationBodyPlanningViewModel.h
    Persistence/RotationBodyPlanningDraftStore.cpp
    Persistence/RotationBodyPlanningDraftStore.h
    Persistence/RotationBodyPlanningDocumentOperations.cpp
    Persistence/RotationBodyPlanningDocumentOperations.h
    Persistence/RotationBodyTrajectoryProjectStore.cpp
    Persistence/RotationBodyTrajectoryProjectStore.h
    Widgets/ModelTransformPanel.cpp
    Widgets/ModelTransformPanel.h
    Widgets/ABBTranslationPanel.cpp
    Widgets/ABBTranslationPanel.h
    Widgets/RotationBodyPlanningLeftPanel.cpp
    Widgets/RotationBodyPlanningLeftPanel.h
    Widgets/RotationBodyPlanningRightPanel.cpp
    Widgets/RotationBodyPlanningRightPanel.h
    Widgets/RotationBodyWorkflowNavigation.cpp
    Widgets/RotationBodyWorkflowNavigation.h
    Widgets/SectionRegionPanel.cpp
    Widgets/SectionRegionPanel.h
    Widgets/SectionView.cpp
    Widgets/SectionView.h
    Widgets/TrajectoryPlanningPanel.cpp
    Widgets/TrajectoryPlanningPanel.h
    Widgets/WorkpieceCalibrationPanel.cpp
    Widgets/WorkpieceCalibrationPanel.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(SMRobotWorkbenchSprayProcess::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        SMRobotWorkbenchCommon::WorkbenchCommon
        SMRobotPlatform::AssetCore
        SMRobotPlatform::SimulationProject
        SMRobotSpray::RotationBodyTrajectoryPlanning
        Qt5::Widgets
    PRIVATE
        nlohmann_json::nlohmann_json
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchSprayProcess/SprayProcessWorkbench
    AUTOMOC ON
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(BuildTests)
    add_subdirectory(
        regression/RotationBodyPlanningSessionTest
        ${PROJECT_BINARY_DIR}/tests/rotation_body_planning_editor)
    add_subdirectory(
        regression/RotationBodyRealModelSmokeTest
        ${PROJECT_BINARY_DIR}/tests/rotation_body_real_model_smoke)
endif()

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        SMRobotWorkbenchCommon::WorkbenchCommon
        SMRobotPlatform::AssetCore
        SMRobotPlatform::SimulationProject
        SMRobotSpray::RotationBodyTrajectoryPlanning
        Qt5::Widgets
        nlohmann_json::nlohmann_json
    )
    set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(REMOVE_RECURSE ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})
    foreach(header_ IN LISTS SOURCES)
        if(header_ MATCHES "\\.h$")
            get_filename_component(header_dir_ ${header_} DIRECTORY)
            file(MAKE_DIRECTORY
                ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME}/${header_dir_})
            configure_file(
                ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
                ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME}/${header_}
                COPYONLY)
        endif()
    endforeach()
    unset(header_)
    unset(header_dir_)
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )
endif()
