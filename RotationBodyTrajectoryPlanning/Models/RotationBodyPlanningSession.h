#pragma once

#include <RotationBodyTrajectoryPlanning/Core/PlanningTypes.h>
#include <RotationBodyTrajectoryPlanning/Core/TriangleMesh.h>
#include <CalibrationInstructionTranslation/Calibration/WorkpieceCalibration.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/RegionEditHistory.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/ToothRegionRecognizer.h>
#include <RotationBodyTrajectoryPlanning/Sectioning/YzSectionExtractor.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryTypes.h>
#include <CalibrationInstructionTranslation/ABBTranslation/RapidModuleGenerator.h>

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace smrobot::workbench::spray::rotationbody
{
    namespace domain = smrobot::spray::rotationbody;

    enum class PublishFrame
    {
        BaseFrame,
        PlanningLocalFrame
    };

    enum class RotationBodyWorkflow
    {
        ModelTransform,
        SectionRegionPlanning
    };

    enum class RotationBodyMainViewMode
    {
        Scene3d,
        Section
    };

    enum class RotationBodyRightWorkflow
    {
        TrajectoryPlanning,
        ABBTranslation,
        WorkpieceCalibration
    };

    struct RotationBodyUiState
    {
        RotationBodyWorkflow workflow{ RotationBodyWorkflow::ModelTransform };
        RotationBodyMainViewMode mainViewMode{ RotationBodyMainViewMode::Scene3d };
        RotationBodyRightWorkflow rightWorkflow{
            RotationBodyRightWorkflow::TrajectoryPlanning
        };
    };

    using CalibrationCoordinateRow = std::array<std::optional<double>, 3>;

    struct WorkpieceCalibrationWorkspace
    {
        std::array<CalibrationCoordinateRow, 12> cylinderRows;
        std::array<CalibrationCoordinateRow, 6> circleRows;
        CalibrationCoordinateRow topReference;
        CalibrationCoordinateRow yDirectionStart;
        CalibrationCoordinateRow yDirectionEnd;
        double workpieceHeightMeters{ 0.12 };
        domain::CalibrationMode activeMode{ domain::CalibrationMode::Cylinder3d };
        domain::CalibrationMode axisSource{ domain::CalibrationMode::Cylinder3d };
        bool showCylinder{ true };
        bool showCircle{ true };
        std::optional<domain::CalibrationAxisFit> cylinderFit;
        std::optional<domain::CalibrationAxisFit> circleFit;
        std::optional<domain::WorkpieceFrameCalibration> frame;
    };

    struct RotationBodyAlgorithmParameters
    {
        domain::YzSectionOptions section;
        domain::ToothRecognitionOptions region;
    };

    struct RotationBodyPlanningDraft
    {
        int schemaVersion{ 1 };
        std::string objectId;
        std::string sourcePath;
        std::string sourceFingerprint;
        std::uint64_t meshFingerprint{ 0 };
        domain::PlanningObjectType objectType{ domain::PlanningObjectType::CompletePart };
        domain::SignedAxis originalRotationAxis{ domain::SignedAxis::PositiveZ };
        domain::SignedAxis toothOutwardAxis{ domain::SignedAxis::PositiveY };
        double motherMaximumDiameterMeters{ 0.0 };
        Eigen::Isometry3d planningFromMesh = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d baseFromPlanning = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d automaticBaseline = Eigen::Isometry3d::Identity();
        Eigen::Vector3d directedRotaryAxisInMesh = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d bottomAxisCenterInMesh = Eigen::Vector3d::Zero();
        domain::ModelStatistics statistics;
        domain::PlanningStage stage{ domain::PlanningStage::NoModel };
        RotationBodyAlgorithmParameters parameters;
        PublishFrame publishFrame{ PublishFrame::BaseFrame };
        domain::BoundaryMode boundaryMode{ domain::BoundaryMode::MaximumToothTopY };
        std::optional<domain::SectionContour> section;
        std::optional<domain::RegionAssignment> automaticRegions;
        std::vector<domain::RegionOverrideCommand> editCommands;
        std::size_t appliedEditCommandCount{ 0 };
        std::optional<domain::SprayBoundary> boundary;
        WorkpieceCalibrationWorkspace calibration;
        RotationBodyUiState uiState;
    };

    struct RotationBodyTrajectoryDraft
    {
        int schemaVersion{ 1 };
        std::string objectId;
        std::string sourceFingerprint;
        std::uint64_t meshFingerprint{ 0 };
        domain::TrajectoryWorkspace workspace;
        domain::RapidExportSettings rapidSettings;
        std::vector<domain::RapidSequenceEntry> rapidSequence;
    };

    enum class DraftRestoreDisposition
    {
        Restored,
        SourceChanged
    };

    class RotationBodyPlanningSession
    {
    public:
        void clear();

        domain::PlanningResult<void> loadModel(
            std::string objectId,
            std::string sourcePath,
            std::string sourceFingerprint,
            domain::TriangleMesh mesh,
            const domain::AlignmentResult& alignment,
            domain::PlanningObjectType objectType,
            domain::SignedAxis originalRotationAxis = domain::SignedAxis::PositiveZ,
            domain::SignedAxis toothOutwardAxis = domain::SignedAxis::PositiveY,
            double motherMaximumDiameterMeters = 0.0);

        domain::PlanningResult<DraftRestoreDisposition> restoreDraft(
            RotationBodyPlanningDraft draft,
            domain::TriangleMesh currentMesh,
            const std::string& currentSourceFingerprint);
        RotationBodyPlanningDraft makeDraft() const;
        domain::PlanningResult<void> restoreTrajectoryDraft(
            RotationBodyTrajectoryDraft draft);
        RotationBodyTrajectoryDraft makeTrajectoryDraft() const;
        domain::PublishedTrajectoryPlan makePublishedTrajectoryPlan() const;

        bool hasModel() const noexcept;
        const domain::TriangleMesh* mesh() const noexcept;
        const std::string& objectId() const noexcept;
        const std::string& sourcePath() const noexcept;
        const std::string& sourceFingerprint() const noexcept;
        std::uint64_t meshFingerprint() const noexcept;
        domain::PlanningObjectType objectType() const noexcept;
        domain::SignedAxis originalRotationAxis() const noexcept;
        domain::SignedAxis toothOutwardAxis() const noexcept;
        double motherMaximumDiameterMeters() const noexcept;
        domain::PlanningStage stage() const noexcept;
        std::uint64_t generation() const noexcept;
        bool hasPendingChanges() const noexcept;
        void markProgressSaved() noexcept;

        const Eigen::Isometry3d& planningFromMesh() const noexcept;
        const Eigen::Isometry3d& baseFromPlanning() const noexcept;
        const Eigen::Isometry3d& automaticBaseline() const noexcept;
        Eigen::Isometry3d publishedMeshTransform() const;
        const domain::ModelStatistics& statistics() const noexcept;
        const Eigen::Vector3d& directedRotaryAxisInMesh() const noexcept;
        const Eigen::Vector3d& bottomAxisCenterInMesh() const noexcept;

        domain::PlanningResult<void> setPlanningFromMesh(const Eigen::Isometry3d& transform);
        domain::PlanningResult<void> setBaseFromPlanning(const Eigen::Isometry3d& transform);
        domain::PlanningResult<void> confirmFrame();
        void setPublishFrame(PublishFrame frame) noexcept;
        PublishFrame publishFrame() const noexcept;

        const RotationBodyAlgorithmParameters& parameters() const noexcept;
        domain::PlanningResult<void> setSectionOptions(const domain::YzSectionOptions& options);
        domain::PlanningResult<void> setRegionOptions(const domain::ToothRecognitionOptions& options);
        domain::BoundaryMode boundaryMode() const noexcept;
        void setBoundaryMode(domain::BoundaryMode mode);

        bool acceptSection(std::uint64_t inputGeneration, domain::SectionContour section);
        bool acceptAutomaticRegions(
            std::uint64_t inputGeneration,
            domain::RegionAssignment automaticRegions);
        domain::PlanningResult<void> applyRegionOverride(
            const domain::YzRectangle& rectangle,
            domain::RegionLabel label);
        bool undoRegionOverride();
        bool redoRegionOverride();
        bool restoreAutomaticRegions();
        bool acceptBoundary(std::uint64_t inputGeneration, domain::SprayBoundary boundary);
        domain::PlanningResult<void> reopenBoundaryForEditing();

        const std::optional<domain::SectionContour>& section() const noexcept;
        const std::optional<domain::RegionAssignment>& automaticRegions() const noexcept;
        std::optional<domain::RegionAssignment> resolvedRegions() const;
        const std::optional<domain::SprayBoundary>& boundary() const noexcept;
        const domain::RegionEditHistory* editHistory() const noexcept;

        const domain::TrajectoryWorkspace& trajectoryWorkspace() const noexcept;
        domain::PlanningResult<void> setTrajectoryParameters(
            const domain::TrajectoryGenerationParameters& parameters);
        domain::PlanningResult<void> generateCurrentTrajectory();
        domain::PlanningResult<void> swapCurrentTrajectoryDirection();
        void setTrajectoryDisplayMode(domain::TrajectoryDisplayMode mode) noexcept;
        domain::PlanningResult<void> interpolateCurrentTrajectory(
            const std::vector<std::size_t>& selectedIndices,
            double intervalSeconds);
        domain::PlanningResult<void> transformCurrentTrajectoryPoints(
            const std::vector<std::size_t>& selectedIndices,
            const domain::TransformComponents& delta);
        domain::PlanningResult<void> beginNewTrajectory();
        domain::PlanningResult<void> loadTrajectoryForEditing(const std::string& passId);
        domain::PlanningResult<std::string> saveCurrentTrajectoryToGroup();
        domain::PlanningResult<void> removeTrajectoryPass(const std::string& passId);
        domain::PlanningResult<void> setTrajectoryPassVisible(
            const std::string& passId,
            bool visible);
        domain::PlanningResult<void> setTrajectoryTransitionAfter(
            const std::string& passId,
            double seconds);

        const domain::RapidExportSettings& rapidSettings() const noexcept;
        const std::vector<domain::RapidSequenceEntry>& rapidSequence() const noexcept;
        domain::PlanningResult<void> setRapidSettings(
            const domain::RapidExportSettings& settings);
        domain::PlanningResult<void> setRapidSequence(
            std::vector<domain::RapidSequenceEntry> sequence);
        void setDefaultRapidOutputDirectory(std::string directory);

        const WorkpieceCalibrationWorkspace& calibrationWorkspace() const noexcept;
        domain::PlanningResult<void> setCalibrationWorkspace(
            WorkpieceCalibrationWorkspace workspace);
        const RotationBodyUiState& uiState() const noexcept;
        void setUiState(const RotationBodyUiState& state) noexcept;

        const domain::PlanningError& error() const noexcept;
        void setError(domain::PlanningError error);
        void clearError() noexcept;

    private:
        static bool validTransform(const Eigen::Isometry3d& transform) noexcept;
        static bool validSectionOptions(const domain::YzSectionOptions& options) noexcept;
        static bool validRegionOptions(const domain::ToothRecognitionOptions& options) noexcept;
        static bool validRegionAssignment(
            const domain::SectionContour& section,
            const domain::RegionAssignment& assignment) noexcept;
        static bool validBoundary(const domain::SprayBoundary& boundary) noexcept;
        void invalidateAfterPlanningTransform();
        void invalidateAfterSection();
        void invalidateBoundary();
        void clearCurrentTrajectory() noexcept;
        void clearAllTrajectories() noexcept;
        void advanceGeneration() noexcept;

        std::optional<domain::TriangleMesh> m_mesh;
        std::string m_objectId;
        std::string m_sourcePath;
        std::string m_sourceFingerprint;
        std::uint64_t m_meshFingerprint{ 0 };
        domain::PlanningObjectType m_objectType{ domain::PlanningObjectType::CompletePart };
        domain::SignedAxis m_originalRotationAxis{ domain::SignedAxis::PositiveZ };
        domain::SignedAxis m_toothOutwardAxis{ domain::SignedAxis::PositiveY };
        double m_motherMaximumDiameterMeters{ 0.0 };
        Eigen::Isometry3d m_planningFromMesh = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_baseFromPlanning = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_automaticBaseline = Eigen::Isometry3d::Identity();
        Eigen::Vector3d m_directedRotaryAxisInMesh = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d m_bottomAxisCenterInMesh = Eigen::Vector3d::Zero();
        domain::ModelStatistics m_statistics;
        domain::PlanningStage m_stage{ domain::PlanningStage::NoModel };
        std::uint64_t m_generation{ 0 };
        RotationBodyAlgorithmParameters m_parameters;
        PublishFrame m_publishFrame{ PublishFrame::BaseFrame };
        domain::BoundaryMode m_boundaryMode{ domain::BoundaryMode::MaximumToothTopY };
        std::optional<domain::SectionContour> m_section;
        std::optional<domain::RegionAssignment> m_automaticRegions;
        std::optional<domain::RegionEditHistory> m_editHistory;
        std::optional<domain::SprayBoundary> m_boundary;
        domain::TrajectoryWorkspace m_trajectoryWorkspace;
        domain::RapidExportSettings m_rapidSettings;
        std::vector<domain::RapidSequenceEntry> m_rapidSequence;
        WorkpieceCalibrationWorkspace m_calibrationWorkspace;
        RotationBodyUiState m_uiState;
        domain::PlanningError m_error;
        bool m_hasPendingChanges{ false };
    };
}
