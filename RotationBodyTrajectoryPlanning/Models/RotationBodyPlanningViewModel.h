#pragma once

#include "RotationBodyPlanningSession.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace smrobot::workbench::spray::rotationbody
{
    struct PostProcessingRobotOption
    {
        std::string id;
        std::string name;
    };

    struct RotationBodyImportOptions
    {
        domain::PlanningObjectType objectType{ domain::PlanningObjectType::CompletePart };
        bool automaticAlignment{ true };
        domain::SignedAxis originalRotationAxis{ domain::SignedAxis::PositiveZ };
        domain::SignedAxis toothOutwardAxis{ domain::SignedAxis::PositiveY };
        double motherMaximumDiameterMeters{ 0.0 };
    };

    struct RotationBodySectionViewSnapshot
    {
        std::optional<domain::SectionContour> section;
        std::optional<domain::RegionAssignment> regions;
        std::optional<domain::SprayBoundary> boundary;
        std::optional<Eigen::Vector2d> trajectoryStartYz;
        std::optional<Eigen::Vector2d> trajectoryEndYz;
    };

    struct RotationBodyPlanningViewModel
    {
        domain::PlanningStage stage{ domain::PlanningStage::NoModel };
        std::uint64_t generation{ 0 };
        std::string objectId;
        std::string sourcePath;
        domain::PlanningErrorCode errorCode{ domain::PlanningErrorCode::None };
        domain::PlanningObjectType objectType{ domain::PlanningObjectType::CompletePart };
        domain::SignedAxis originalRotationAxis{ domain::SignedAxis::PositiveZ };
        domain::SignedAxis toothOutwardAxis{ domain::SignedAxis::PositiveY };
        double motherMaximumDiameterMeters{ 0.0 };
        domain::ModelStatistics statistics;
        domain::TransformComponents planningDelta;
        domain::TransformComponents baseFromPlanning;
        PublishFrame publishFrame{ PublishFrame::BaseFrame };
        domain::BoundaryMode boundaryMode{ domain::BoundaryMode::MaximumToothTopY };
        RotationBodySectionViewSnapshot sectionView;
        domain::TrajectoryWorkspace trajectoryWorkspace;
        domain::RapidExportSettings rapidSettings;
        std::vector<domain::RapidSequenceEntry> rapidSequence;
        WorkpieceCalibrationWorkspace calibrationWorkspace;
        std::string calibrationInputDirectory;
        std::string trajectoryParameterInputDirectory;
        RotationBodyUiState uiState;
        std::optional<domain::RapidModule> rapidModulePreview;
        std::string rapidOutputFile;
        std::string rapidExportStatus;
        bool rapidExportSucceeded{ false };
        std::vector<PostProcessingRobotOption> postProcessingRobots;
        std::vector<std::string> postProcessingTemplates;
        std::string postProcessingRobotId;
        std::string postProcessingPreview;
        std::vector<std::string> postProcessingOutputFiles;
        std::string postProcessingStatus;
        bool postProcessingTrajectoryLoaded{ false };
        bool postProcessingExportSucceeded{ false };
        std::array<std::size_t, 5> regionSegmentCounts{};
        bool hasModel{ false };
        bool canEditModelTransform{ false };
        bool canConfirmFrame{ false };
        bool canCreateSection{ false };
        bool canRecognizeRegions{ false };
        bool canEditRegions{ false };
        bool canUndoRegionEdit{ false };
        bool canRedoRegionEdit{ false };
        bool canRestoreAutomaticRegions{ false };
        bool canConfirmBoundary{ false };
        bool canReopenBoundary{ false };
        bool canGenerateTrajectory{ false };
        bool canEditCurrentTrajectory{ false };
        bool canSaveCurrentTrajectory{ false };
        bool canEditTrajectoryGroup{ false };
        bool canExportRapid{ false };
        bool canLoadPostProcessingTrajectory{ false };
        bool canExportPostProcessedProgram{ false };
        bool canSaveProgress{ false };
        bool hasPendingChanges{ false };
        bool isBusy{ false };
    };
}
