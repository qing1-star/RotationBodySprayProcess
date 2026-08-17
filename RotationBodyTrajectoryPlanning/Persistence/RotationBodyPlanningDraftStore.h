#pragma once

#include "../Models/RotationBodyPlanningSession.h"

#include <SimulationProject/ProjectDocument.h>

#include <cstdint>
#include <optional>
#include <string>

namespace smrobot::workbench::spray::rotationbody
{
    enum class DraftReadStatus
    {
        NotFound,
        Loaded,
        SourceChanged,
        UnsupportedVersion,
        InvalidPayload
    };

    struct DraftReadResult
    {
        DraftReadStatus status{ DraftReadStatus::NotFound };
        std::optional<RotationBodyPlanningDraft> draft;
        std::string message;

        bool usable() const noexcept
        {
            return status == DraftReadStatus::Loaded || status == DraftReadStatus::SourceChanged;
        }
    };

    class RotationBodyPlanningDraftStore
    {
    public:
        static constexpr const char* extensionKey =
            "spray.rotation_body_trajectory_planning.draft";
        static constexpr int currentVersion = 1;

        static bool write(
            simulation_project::ProjectDocument& document,
            const RotationBodyPlanningDraft& draft);
        static DraftReadResult read(
            const simulation_project::ProjectDocument& document,
            const std::string& currentSourceFingerprint = {},
            std::uint64_t currentMeshFingerprint = 0);
        static bool erase(simulation_project::ProjectDocument& document);
    };
}
