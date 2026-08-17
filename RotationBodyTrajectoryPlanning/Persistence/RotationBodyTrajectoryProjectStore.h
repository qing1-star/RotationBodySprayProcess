#pragma once

#include "../Models/RotationBodyPlanningSession.h"

#include <SimulationProject/ProjectDocument.h>

#include <optional>
#include <string>

namespace smrobot::workbench::spray::rotationbody
{
    enum class TrajectoryProjectReadStatus
    {
        NotFound,
        Loaded,
        UnsupportedVersion,
        InvalidPayload
    };

    struct TrajectoryWorkspaceReadResult
    {
        TrajectoryProjectReadStatus status{ TrajectoryProjectReadStatus::NotFound };
        std::optional<RotationBodyTrajectoryDraft> draft;
        std::string message;

        bool usable() const noexcept
        {
            return status == TrajectoryProjectReadStatus::Loaded;
        }
    };

    struct PublishedTrajectoryReadResult
    {
        TrajectoryProjectReadStatus status{ TrajectoryProjectReadStatus::NotFound };
        std::optional<domain::PublishedTrajectoryPlan> plan;
        std::string message;

        bool usable() const noexcept
        {
            return status == TrajectoryProjectReadStatus::Loaded;
        }
    };

    class RotationBodyTrajectoryProjectStore
    {
    public:
        static constexpr int currentVersion = 1;
        static constexpr const char* workspaceExtensionKey =
            "spray.rotation_body_trajectory_planning.trajectory_draft";
        static constexpr const char* publishedExtensionKey =
            "spray.rotation_body_trajectory_planning.trajectory_plan";

        static bool writeWorkspace(
            simulation_project::ProjectDocument& document,
            const RotationBodyTrajectoryDraft& draft);
        static TrajectoryWorkspaceReadResult readWorkspace(
            const simulation_project::ProjectDocument& document);
        static bool eraseWorkspace(simulation_project::ProjectDocument& document);

        static bool writePublishedPlan(
            simulation_project::ProjectDocument& document,
            const domain::PublishedTrajectoryPlan& plan);
        static PublishedTrajectoryReadResult readPublishedPlan(
            const simulation_project::ProjectDocument& document);
        static bool erasePublishedPlan(simulation_project::ProjectDocument& document);
        static bool eraseAll(simulation_project::ProjectDocument& document);
    };
}
