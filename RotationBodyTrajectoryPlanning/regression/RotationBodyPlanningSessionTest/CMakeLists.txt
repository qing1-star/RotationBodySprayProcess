cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RotationBodyTrajectoryPlanningEditorTest)

add_executable(${TARGET_NAME}
    FakeViewportServices.h
    main.cpp
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
        RotationBodyTrajectoryPlanningEditor
        SMRobotPlatform::RenderCore
        SMRobotPlatform::RobotRenderBridge
        SMRobotPlatform::SceneCore
)

target_compile_definitions(${TARGET_NAME}
    PRIVATE
        ROTATION_BODY_TEST_SOURCE_ROOT="${CMAKE_SOURCE_DIR}"
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchSprayProcess/Tests
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:${TARGET_NAME}>/platforms"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:Qt5::QOffscreenIntegrationPlugin>"
        "$<TARGET_FILE_DIR:${TARGET_NAME}>/platforms/"
)

add_test(NAME RotationBodyTrajectoryPlanningEditor COMMAND ${TARGET_NAME})

set_tests_properties(RotationBodyTrajectoryPlanningEditor PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)
