cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RotationBodyRealModelSmokeTest)

add_executable(${TARGET_NAME}
    main.cpp
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
        RotationBodyTrajectoryPlanningEditor
)

target_compile_definitions(${TARGET_NAME}
    PRIVATE
        ROTATION_BODY_SOURCE_ROOT="${CMAKE_SOURCE_DIR}"
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchSprayProcess/Tests
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

add_test(NAME RotationBodyRealModelSmoke COMMAND ${TARGET_NAME})

set_tests_properties(RotationBodyRealModelSmoke PROPERTIES
    TIMEOUT 600
    LABELS "real-model"
)
