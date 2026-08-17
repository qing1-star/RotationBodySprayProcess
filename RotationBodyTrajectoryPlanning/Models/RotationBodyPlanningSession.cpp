#include "RotationBodyPlanningSession.h"

#include <RotationBodyTrajectoryPlanning/Alignment/RotationBodyAlignmentSolver.h>
#include <RotationBodyTrajectoryPlanning/Alignment/SimulationBlockPlacementSolver.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryEditor.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryGroupEditor.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryPlanner.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        bool stageAtLeast(domain::PlanningStage value, domain::PlanningStage threshold)
        {
            return static_cast<int>(value) >= static_cast<int>(threshold);
        }

        bool sameSectionOptions(
            const domain::YzSectionOptions& lhs,
            const domain::YzSectionOptions& rhs)
        {
            return lhs.relativePlaneTolerance == rhs.relativePlaneTolerance &&
                lhs.relativeWeldTolerance == rhs.relativeWeldTolerance &&
                lhs.minimumToleranceMeters == rhs.minimumToleranceMeters;
        }

        bool sameRegionOptions(
            const domain::ToothRecognitionOptions& lhs,
            const domain::ToothRecognitionOptions& rhs)
        {
            return lhs.minimumResampleCount == rhs.minimumResampleCount &&
                lhs.maximumResampleCount == rhs.maximumResampleCount &&
                lhs.smoothingWindowFraction == rhs.smoothingWindowFraction &&
                lhs.minimumRadialProminenceFraction == rhs.minimumRadialProminenceFraction &&
                lhs.axialSurfaceThreshold == rhs.axialSurfaceThreshold &&
                lhs.minimumConfidence == rhs.minimumConfidence &&
                lhs.transitionExtentFraction == rhs.transitionExtentFraction;
        }

        bool sameTrajectoryParameters(
            const domain::TrajectoryGenerationParameters& lhs,
            const domain::TrajectoryGenerationParameters& rhs)
        {
            return lhs.sprayDistanceMeters == rhs.sprayDistanceMeters &&
                lhs.tiltRadians == rhs.tiltRadians &&
                lhs.speedMetersPerSecond == rhs.speedMetersPerSecond &&
                lhs.startExtensionMeters == rhs.startExtensionMeters &&
                lhs.endExtensionMeters == rhs.endExtensionMeters &&
                lhs.pointCount == rhs.pointCount &&
                lhs.positionerRpm == rhs.positionerRpm &&
                lhs.reversed == rhs.reversed;
        }

        bool sameTrajectoryPoints(
            const std::vector<domain::TrajectoryPosePoint>& lhs,
            const std::vector<domain::TrajectoryPosePoint>& rhs)
        {
            if(lhs.size() != rhs.size()) {
                return false;
            }
            for(std::size_t index = 0; index < lhs.size(); ++index) {
                if(std::abs(lhs[index].timeSeconds - rhs[index].timeSeconds) > 1.0e-12 ||
                    lhs[index].interpolated != rhs[index].interpolated ||
                    !lhs[index].planningFromTool.matrix().isApprox(
                        rhs[index].planningFromTool.matrix(),
                        1.0e-12)) {
                    return false;
                }
            }
            return true;
        }

        bool samePlannedTrajectory(
            const domain::PlannedTrajectory& lhs,
            const domain::PlannedTrajectory& rhs)
        {
            return sameTrajectoryParameters(lhs.parameters, rhs.parameters) &&
                lhs.targetSurfaceStart.isApprox(rhs.targetSurfaceStart, 1.0e-12) &&
                lhs.targetSurfaceEnd.isApprox(rhs.targetSurfaceEnd, 1.0e-12) &&
                sameTrajectoryPoints(lhs.linearPoints, rhs.linearPoints) &&
                sameTrajectoryPoints(
                    lhs.relativeHelicalPoints,
                    rhs.relativeHelicalPoints);
        }

        bool validTrajectoryParameters(
            const domain::TrajectoryGenerationParameters& parameters) noexcept
        {
            constexpr double pi = 3.14159265358979323846;
            return std::isfinite(parameters.sprayDistanceMeters) &&
                parameters.sprayDistanceMeters > 0.0 &&
                std::isfinite(parameters.tiltRadians) &&
                std::abs(parameters.tiltRadians) < 80.0 * pi / 180.0 &&
                std::isfinite(parameters.speedMetersPerSecond) &&
                parameters.speedMetersPerSecond > 0.0 &&
                std::isfinite(parameters.startExtensionMeters) &&
                parameters.startExtensionMeters >= 0.0 &&
                std::isfinite(parameters.endExtensionMeters) &&
                parameters.endExtensionMeters >= 0.0 &&
                (parameters.pointCount == 0 ||
                    (parameters.pointCount >= domain::TrajectoryPlanner::minimumPointCount &&
                        parameters.pointCount <= domain::TrajectoryPlanner::maximumPointCount)) &&
                std::isfinite(parameters.positionerRpm);
        }

        bool validRapidFileName(const std::string& value) noexcept
        {
            return !value.empty() &&
                value.find_first_of("\\/:*?\"<>|") == std::string::npos;
        }

        bool validCalibrationRow(const CalibrationCoordinateRow& row) noexcept
        {
            return std::all_of(
                row.begin(),
                row.end(),
                [](const std::optional<double>& value) {
                    return !value || std::isfinite(*value);
                });
        }

        bool validCalibrationFit(const domain::CalibrationAxisFit& fit) noexcept
        {
            return fit.axisPointBaseMeters.allFinite() &&
                fit.axisDirectionBase.allFinite() &&
                fit.axisDirectionBase.norm() > 1.0e-12 &&
                std::isfinite(fit.radiusMeters) && fit.radiusMeters > 0.0 &&
                std::isfinite(fit.rmsResidualMeters) && fit.rmsResidualMeters >= 0.0 &&
                std::isfinite(fit.maximumResidualMeters) &&
                fit.maximumResidualMeters >= 0.0 &&
                std::all_of(
                    fit.sourcePointsBaseMeters.begin(),
                    fit.sourcePointsBaseMeters.end(),
                    [](const Eigen::Vector3d& point) { return point.allFinite(); });
        }

        bool validCalibrationFrame(
            const domain::WorkpieceFrameCalibration& frame) noexcept
        {
            return frame.baseFromPlanning.matrix().allFinite() &&
                frame.topReferenceBaseMeters.allFinite() &&
                frame.topCenterBaseMeters.allFinite() &&
                frame.baseOriginBaseMeters.allFinite() &&
                frame.yDirectionStartBaseMeters.allFinite() &&
                frame.yDirectionEndBaseMeters.allFinite() &&
                frame.xAxisBase.allFinite() && frame.yAxisBase.allFinite() &&
                frame.zAxisBase.allFinite() && frame.abbQuaternionWxyz.allFinite() &&
                std::isfinite(frame.workpieceHeightMeters) &&
                frame.workpieceHeightMeters > 0.0 &&
                std::isfinite(frame.fittedRadiusMeters) &&
                frame.fittedRadiusMeters > 0.0 &&
                std::isfinite(frame.yDirectionDistanceMeters) &&
                frame.yDirectionDistanceMeters > 0.0;
        }

        bool validCalibrationWorkspace(
            const WorkpieceCalibrationWorkspace& workspace) noexcept
        {
            const bool rowsValid = std::all_of(
                    workspace.cylinderRows.begin(),
                    workspace.cylinderRows.end(),
                    validCalibrationRow) &&
                std::all_of(
                    workspace.circleRows.begin(),
                    workspace.circleRows.end(),
                    validCalibrationRow) &&
                validCalibrationRow(workspace.topReference) &&
                validCalibrationRow(workspace.yDirectionStart) &&
                validCalibrationRow(workspace.yDirectionEnd);
            return rowsValid &&
                std::isfinite(workspace.workpieceHeightMeters) &&
                workspace.workpieceHeightMeters > 0.0 &&
                (!workspace.cylinderFit || validCalibrationFit(*workspace.cylinderFit)) &&
                (!workspace.circleFit || validCalibrationFit(*workspace.circleFit)) &&
                (!workspace.frame || validCalibrationFrame(*workspace.frame));
        }
    }

    void RotationBodyPlanningSession::clear()
    {
        m_mesh.reset();
        m_objectId.clear();
        m_sourcePath.clear();
        m_sourceFingerprint.clear();
        m_meshFingerprint = 0;
        m_objectType = domain::PlanningObjectType::CompletePart;
        m_originalRotationAxis = domain::SignedAxis::PositiveZ;
        m_toothOutwardAxis = domain::SignedAxis::PositiveY;
        m_motherMaximumDiameterMeters = 0.0;
        m_planningFromMesh.setIdentity();
        m_baseFromPlanning.setIdentity();
        m_automaticBaseline.setIdentity();
        m_directedRotaryAxisInMesh = Eigen::Vector3d::UnitZ();
        m_bottomAxisCenterInMesh.setZero();
        m_statistics = {};
        m_stage = domain::PlanningStage::NoModel;
        m_parameters = {};
        m_publishFrame = PublishFrame::BaseFrame;
        m_boundaryMode = domain::BoundaryMode::MaximumToothTopY;
        m_section.reset();
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearCurrentTrajectory();
        m_trajectoryWorkspace = {};
        m_rapidSettings = {};
        m_rapidSequence.clear();
        m_calibrationWorkspace = {};
        m_uiState = {};
        m_error = {};
        m_hasPendingChanges = false;
        advanceGeneration();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::loadModel(
        std::string objectId,
        std::string sourcePath,
        std::string sourceFingerprint,
        domain::TriangleMesh mesh,
        const domain::AlignmentResult& alignment,
        domain::PlanningObjectType objectType,
        domain::SignedAxis originalRotationAxis,
        domain::SignedAxis toothOutwardAxis,
        double motherMaximumDiameterMeters)
    {
        const domain::PlanningResult<void> validation = mesh.validate();
        if(!validation) {
            return validation;
        }
        if(objectId.empty() || sourcePath.empty() || sourceFingerprint.empty()) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "A loaded planning model requires an object id, source path and fingerprint.");
        }
        if(!validTransform(alignment.planningFromMesh) ||
            !validTransform(alignment.automaticBaseline) ||
            !alignment.bottomAxisCenterInMesh.allFinite() ||
            !alignment.statistics.estimatedAxisInMesh.allFinite() ||
            alignment.statistics.estimatedAxisInMesh.norm() <= 1.0e-12) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::NonFiniteGeometry,
                "The automatic alignment contains invalid geometry.");
        }
        if(objectType == domain::PlanningObjectType::SimulationBlock &&
            (!std::isfinite(motherMaximumDiameterMeters) || motherMaximumDiameterMeters <= 0.0)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "A simulation block requires a positive mother maximum diameter.");
        }

        m_meshFingerprint = mesh.stableFingerprint();
        m_mesh = std::move(mesh);
        m_objectId = std::move(objectId);
        m_sourcePath = std::move(sourcePath);
        m_sourceFingerprint = std::move(sourceFingerprint);
        m_objectType = objectType;
        m_originalRotationAxis = originalRotationAxis;
        m_toothOutwardAxis = toothOutwardAxis;
        m_motherMaximumDiameterMeters = motherMaximumDiameterMeters;
        m_planningFromMesh = alignment.planningFromMesh;
        m_automaticBaseline = alignment.automaticBaseline;
        m_baseFromPlanning.setIdentity();
        m_directedRotaryAxisInMesh = alignment.statistics.estimatedAxisInMesh.normalized();
        m_bottomAxisCenterInMesh = alignment.bottomAxisCenterInMesh;
        m_statistics = alignment.statistics;
        m_calibrationWorkspace = {};
        if(m_statistics.heightMeters > 0.0) {
            m_calibrationWorkspace.workpieceHeightMeters = m_statistics.heightMeters;
        }
        m_uiState = {};
        m_stage = domain::PlanningStage::ModelLoaded;
        m_section.reset();
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearCurrentTrajectory();
        clearAllTrajectories();
        m_error = {};
        m_hasPendingChanges = true;
        advanceGeneration();
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<DraftRestoreDisposition> RotationBodyPlanningSession::restoreDraft(
        RotationBodyPlanningDraft draft,
        domain::TriangleMesh currentMesh,
        const std::string& currentSourceFingerprint)
    {
        const domain::PlanningResult<void> validation = currentMesh.validate();
        if(!validation) {
            return domain::PlanningResult<DraftRestoreDisposition>::failure(
                validation.error.code,
                validation.error.message);
        }
        if(draft.schemaVersion != 1 || draft.objectId.empty() || draft.sourcePath.empty() ||
            currentSourceFingerprint.empty() || !validTransform(draft.planningFromMesh) ||
            !validTransform(draft.baseFromPlanning) || !validTransform(draft.automaticBaseline) ||
            !draft.directedRotaryAxisInMesh.allFinite() ||
            draft.directedRotaryAxisInMesh.norm() <= 1.0e-12 ||
            !draft.bottomAxisCenterInMesh.allFinite()) {
            return domain::PlanningResult<DraftRestoreDisposition>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The planning draft identity or transform data is invalid.");
        }

        const std::uint64_t currentMeshFingerprint = currentMesh.stableFingerprint();
        const bool sourceChanged = draft.sourceFingerprint != currentSourceFingerprint ||
            draft.meshFingerprint != currentMeshFingerprint;
        std::optional<domain::AlignmentResult> refreshedAlignment;
        if(sourceChanged) {
            domain::PlanningResult<domain::AlignmentResult> alignment =
                draft.objectType == domain::PlanningObjectType::CompletePart
                ? domain::RotationBodyAlignmentSolver::solve(currentMesh)
                : domain::SimulationBlockPlacementSolver::solve(
                    currentMesh,
                    draft.originalRotationAxis,
                    draft.toothOutwardAxis,
                    draft.motherMaximumDiameterMeters);
            if(!alignment) {
                return domain::PlanningResult<DraftRestoreDisposition>::failure(
                    alignment.error.code,
                    alignment.error.message);
            }
            refreshedAlignment = std::move(alignment.value);
        }
        if(!validSectionOptions(draft.parameters.section) ||
            !validRegionOptions(draft.parameters.region) ||
            !validCalibrationWorkspace(draft.calibration)) {
            return domain::PlanningResult<DraftRestoreDisposition>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The planning draft contains unsupported algorithm parameters.");
        }
        if(!sourceChanged && draft.section && draft.automaticRegions &&
            !validRegionAssignment(*draft.section, *draft.automaticRegions)) {
            return domain::PlanningResult<DraftRestoreDisposition>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The planning draft contains invalid automatic region confidence values.");
        }

        m_mesh = std::move(currentMesh);
        m_objectId = std::move(draft.objectId);
        m_sourcePath = std::move(draft.sourcePath);
        m_sourceFingerprint = currentSourceFingerprint;
        m_meshFingerprint = currentMeshFingerprint;
        m_objectType = draft.objectType;
        m_originalRotationAxis = draft.originalRotationAxis;
        m_toothOutwardAxis = draft.toothOutwardAxis;
        m_motherMaximumDiameterMeters = draft.motherMaximumDiameterMeters;
        m_planningFromMesh = draft.planningFromMesh;
        m_baseFromPlanning = draft.baseFromPlanning;
        m_automaticBaseline = refreshedAlignment
            ? refreshedAlignment->automaticBaseline
            : draft.automaticBaseline;
        m_directedRotaryAxisInMesh = refreshedAlignment
            ? refreshedAlignment->statistics.estimatedAxisInMesh.normalized()
            : draft.directedRotaryAxisInMesh;
        m_bottomAxisCenterInMesh = refreshedAlignment
            ? refreshedAlignment->bottomAxisCenterInMesh
            : draft.bottomAxisCenterInMesh;
        m_statistics = refreshedAlignment ? refreshedAlignment->statistics : draft.statistics;
        m_parameters = draft.parameters;
        m_publishFrame = draft.publishFrame;
        m_boundaryMode = draft.boundaryMode;
        m_calibrationWorkspace = std::move(draft.calibration);
        m_uiState = draft.uiState;
        m_error = {};

        m_section.reset();
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearAllTrajectories();
        if(sourceChanged) {
            m_stage = domain::PlanningStage::ModelLoaded;
            m_hasPendingChanges = true;
            advanceGeneration();
            return domain::PlanningResult<DraftRestoreDisposition>::success(
                DraftRestoreDisposition::SourceChanged);
        }

        m_stage = domain::PlanningStage::ModelLoaded;
        if(stageAtLeast(draft.stage, domain::PlanningStage::FrameConfirmed)) {
            m_stage = domain::PlanningStage::FrameConfirmed;
        }
        if(draft.section && stageAtLeast(draft.stage, domain::PlanningStage::SectionReady)) {
            m_section = std::move(draft.section);
            m_stage = domain::PlanningStage::SectionReady;
        }
        if(m_section && draft.automaticRegions &&
            draft.automaticRegions->matches(*m_section) &&
            stageAtLeast(draft.stage, domain::PlanningStage::RegionsReady)) {
            domain::RegionEditHistory history;
            const domain::PlanningResult<void> initialized =
                history.resetAutomatic(*m_section, *draft.automaticRegions);
            if(!initialized) {
                return domain::PlanningResult<DraftRestoreDisposition>::failure(
                    initialized.error.code,
                    initialized.error.message);
            }
            m_automaticRegions = std::move(draft.automaticRegions);
            m_editHistory = std::move(history);
            for(const domain::RegionOverrideCommand& command : draft.editCommands) {
                const domain::PlanningResult<void> applied =
                    m_editHistory->applyRectangle(command.rectangle, command.label);
                if(!applied) {
                    m_editHistory.reset();
                    m_automaticRegions.reset();
                    m_stage = domain::PlanningStage::SectionReady;
                    break;
                }
            }
            if(m_editHistory) {
                const std::size_t targetApplied =
                    std::min(draft.appliedEditCommandCount, draft.editCommands.size());
                while(m_editHistory->appliedCommandCount() > targetApplied) {
                    m_editHistory->undo();
                }
                m_stage = domain::PlanningStage::RegionsReady;
            }
        }
        if(m_editHistory && draft.boundary &&
            draft.boundary->mode == m_boundaryMode && validBoundary(*draft.boundary) &&
            stageAtLeast(draft.stage, domain::PlanningStage::SprayBoundaryConfirmed)) {
            m_boundary = std::move(draft.boundary);
            m_stage = domain::PlanningStage::SprayBoundaryConfirmed;
        }
        m_hasPendingChanges = false;
        advanceGeneration();
        return domain::PlanningResult<DraftRestoreDisposition>::success(
            DraftRestoreDisposition::Restored);
    }

    RotationBodyPlanningDraft RotationBodyPlanningSession::makeDraft() const
    {
        RotationBodyPlanningDraft draft;
        draft.objectId = m_objectId;
        draft.sourcePath = m_sourcePath;
        draft.sourceFingerprint = m_sourceFingerprint;
        draft.meshFingerprint = m_meshFingerprint;
        draft.objectType = m_objectType;
        draft.originalRotationAxis = m_originalRotationAxis;
        draft.toothOutwardAxis = m_toothOutwardAxis;
        draft.motherMaximumDiameterMeters = m_motherMaximumDiameterMeters;
        draft.planningFromMesh = m_planningFromMesh;
        draft.baseFromPlanning = m_baseFromPlanning;
        draft.automaticBaseline = m_automaticBaseline;
        draft.directedRotaryAxisInMesh = m_directedRotaryAxisInMesh;
        draft.bottomAxisCenterInMesh = m_bottomAxisCenterInMesh;
        draft.statistics = m_statistics;
        draft.stage = m_stage;
        draft.parameters = m_parameters;
        draft.publishFrame = m_publishFrame;
        draft.boundaryMode = m_boundaryMode;
        draft.section = m_section;
        draft.automaticRegions = m_automaticRegions;
        if(m_editHistory) {
            draft.editCommands = m_editHistory->commands();
            draft.appliedEditCommandCount = m_editHistory->appliedCommandCount();
        }
        draft.boundary = m_boundary;
        draft.calibration = m_calibrationWorkspace;
        draft.uiState = m_uiState;
        return draft;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::restoreTrajectoryDraft(
        RotationBodyTrajectoryDraft draft)
    {
        if(!hasModel() || draft.schemaVersion != 1 ||
            draft.objectId != m_objectId ||
            draft.sourceFingerprint != m_sourceFingerprint ||
            draft.meshFingerprint != m_meshFingerprint ||
            !validTrajectoryParameters(draft.workspace.parameters)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The saved trajectory workspace does not match the planning workpiece.");
        }

        domain::PlanningResult<void> groupValidation =
            domain::TrajectoryGroupEditor::validate(draft.workspace.group);
        if(!groupValidation) {
            return groupValidation;
        }
        if(draft.workspace.currentTrajectory) {
            if(!validBoundary(draft.workspace.currentTrajectory->sourceBoundary)) {
                return domain::PlanningResult<void>::failure(
                    domain::PlanningErrorCode::InvalidArgument,
                    "The saved current trajectory has an invalid source boundary.");
            }
            domain::PlanningResult<void> rebuilt = domain::TrajectoryEditor::rebuildDerived(
                *draft.workspace.currentTrajectory);
            if(!rebuilt) {
                return rebuilt;
            }
        }
        if(!draft.workspace.editingPassId.empty() &&
            domain::TrajectoryGroupEditor::find(
                draft.workspace.group,
                draft.workspace.editingPassId) == nullptr) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The saved trajectory editing target no longer exists.");
        }
        if(!draft.rapidSettings.safetyPositionBaseMeters.allFinite() ||
            !std::isfinite(draft.rapidSettings.safetySpeedMetersPerSecond) ||
            draft.rapidSettings.safetySpeedMetersPerSecond <= 0.0 ||
            !domain::RapidModuleGenerator::isValidRapidIdentifier(
                draft.rapidSettings.moduleName) ||
            !domain::RapidModuleGenerator::isValidRapidIdentifier(
                draft.rapidSettings.toolDataName) ||
            !validRapidFileName(draft.rapidSettings.fileName)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The saved ABB RAPID settings are invalid.");
        }
        for(const domain::RapidSequenceEntry& entry : draft.rapidSequence) {
            if(entry.kind == domain::RapidSequenceEntryKind::Trajectory &&
                domain::TrajectoryGroupEditor::find(
                    draft.workspace.group,
                    entry.trajectoryPassId) == nullptr) {
                return domain::PlanningResult<void>::failure(
                    domain::PlanningErrorCode::InvalidArgument,
                    "The saved ABB sequence references a missing trajectory.");
            }
        }

        // Older builds retained an exact saved pass as the current edit, which
        // made it visible even when every group checkbox was cleared. Preserve
        // genuine unsaved edits, but discard that byte-equivalent legacy copy.
        if(draft.workspace.currentTrajectory &&
            !draft.workspace.editingPassId.empty()) {
            const domain::TrajectoryPass* editingPass =
                domain::TrajectoryGroupEditor::find(
                    draft.workspace.group,
                    draft.workspace.editingPassId);
            if(editingPass && samePlannedTrajectory(
                *draft.workspace.currentTrajectory,
                editingPass->trajectory)) {
                draft.workspace.currentTrajectory.reset();
                draft.workspace.editingPassId.clear();
            }
        }

        m_trajectoryWorkspace = std::move(draft.workspace);
        m_rapidSettings = std::move(draft.rapidSettings);
        m_rapidSequence = std::move(draft.rapidSequence);
        m_hasPendingChanges = false;
        return domain::PlanningResult<void>::success();
    }

    RotationBodyTrajectoryDraft RotationBodyPlanningSession::makeTrajectoryDraft() const
    {
        RotationBodyTrajectoryDraft draft;
        draft.objectId = m_objectId;
        draft.sourceFingerprint = m_sourceFingerprint;
        draft.meshFingerprint = m_meshFingerprint;
        draft.workspace = m_trajectoryWorkspace;
        draft.rapidSettings = m_rapidSettings;
        draft.rapidSequence = m_rapidSequence;
        return draft;
    }

    domain::PublishedTrajectoryPlan RotationBodyPlanningSession::makePublishedTrajectoryPlan() const
    {
        domain::PublishedTrajectoryPlan plan;
        plan.objectId = m_objectId;
        plan.baseFromPlanning = m_baseFromPlanning;
        plan.group = m_trajectoryWorkspace.group;
        plan.safetyPositionBaseMeters = m_rapidSettings.safetyPositionBaseMeters;
        plan.safetySpeedMetersPerSecond = m_rapidSettings.safetySpeedMetersPerSecond;
        plan.executionSequence = m_rapidSequence;
        return plan;
    }

    bool RotationBodyPlanningSession::hasModel() const noexcept { return m_mesh.has_value(); }
    const domain::TriangleMesh* RotationBodyPlanningSession::mesh() const noexcept
    {
        return m_mesh ? &*m_mesh : nullptr;
    }
    const std::string& RotationBodyPlanningSession::objectId() const noexcept { return m_objectId; }
    const std::string& RotationBodyPlanningSession::sourcePath() const noexcept { return m_sourcePath; }
    const std::string& RotationBodyPlanningSession::sourceFingerprint() const noexcept
    {
        return m_sourceFingerprint;
    }
    std::uint64_t RotationBodyPlanningSession::meshFingerprint() const noexcept
    {
        return m_meshFingerprint;
    }
    domain::PlanningObjectType RotationBodyPlanningSession::objectType() const noexcept
    {
        return m_objectType;
    }
    domain::SignedAxis RotationBodyPlanningSession::originalRotationAxis() const noexcept
    {
        return m_originalRotationAxis;
    }
    domain::SignedAxis RotationBodyPlanningSession::toothOutwardAxis() const noexcept
    {
        return m_toothOutwardAxis;
    }
    double RotationBodyPlanningSession::motherMaximumDiameterMeters() const noexcept
    {
        return m_motherMaximumDiameterMeters;
    }
    domain::PlanningStage RotationBodyPlanningSession::stage() const noexcept { return m_stage; }
    std::uint64_t RotationBodyPlanningSession::generation() const noexcept { return m_generation; }
    bool RotationBodyPlanningSession::hasPendingChanges() const noexcept
    {
        return m_hasPendingChanges;
    }
    void RotationBodyPlanningSession::markProgressSaved() noexcept
    {
        m_hasPendingChanges = false;
    }
    const Eigen::Isometry3d& RotationBodyPlanningSession::planningFromMesh() const noexcept
    {
        return m_planningFromMesh;
    }
    const Eigen::Isometry3d& RotationBodyPlanningSession::baseFromPlanning() const noexcept
    {
        return m_baseFromPlanning;
    }
    const Eigen::Isometry3d& RotationBodyPlanningSession::automaticBaseline() const noexcept
    {
        return m_automaticBaseline;
    }
    Eigen::Isometry3d RotationBodyPlanningSession::publishedMeshTransform() const
    {
        return m_publishFrame == PublishFrame::BaseFrame
            ? m_baseFromPlanning * m_planningFromMesh
            : m_planningFromMesh;
    }
    const domain::ModelStatistics& RotationBodyPlanningSession::statistics() const noexcept
    {
        return m_statistics;
    }
    const Eigen::Vector3d& RotationBodyPlanningSession::directedRotaryAxisInMesh() const noexcept
    {
        return m_directedRotaryAxisInMesh;
    }
    const Eigen::Vector3d& RotationBodyPlanningSession::bottomAxisCenterInMesh() const noexcept
    {
        return m_bottomAxisCenterInMesh;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setPlanningFromMesh(
        const Eigen::Isometry3d& transform)
    {
        if(!hasModel()) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::EmptyMesh,
                "Load a model before changing its planning transform.");
        }
        if(!validTransform(transform)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The planning transform must be a finite rigid transform.");
        }
        if(m_planningFromMesh.matrix().isApprox(transform.matrix(), 1.0e-12)) {
            return domain::PlanningResult<void>::success();
        }
        m_planningFromMesh = transform;
        m_hasPendingChanges = true;
        invalidateAfterPlanningTransform();
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setBaseFromPlanning(
        const Eigen::Isometry3d& transform)
    {
        if(!validTransform(transform)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The base transform must be a finite rigid transform.");
        }
        m_baseFromPlanning = transform;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::confirmFrame()
    {
        if(!hasModel()) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::EmptyMesh,
                "Load a model before confirming the planning frame.");
        }
        if(stageAtLeast(m_stage, domain::PlanningStage::FrameConfirmed)) {
            return domain::PlanningResult<void>::success();
        }
        invalidateAfterPlanningTransform();
        m_stage = domain::PlanningStage::FrameConfirmed;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    void RotationBodyPlanningSession::setPublishFrame(PublishFrame frame) noexcept
    {
        if(m_publishFrame == frame) {
            return;
        }
        m_publishFrame = frame;
        m_hasPendingChanges = true;
    }
    PublishFrame RotationBodyPlanningSession::publishFrame() const noexcept { return m_publishFrame; }
    const RotationBodyAlgorithmParameters& RotationBodyPlanningSession::parameters() const noexcept
    {
        return m_parameters;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setSectionOptions(
        const domain::YzSectionOptions& options)
    {
        if(!validSectionOptions(options)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Section tolerances are outside their supported ranges.");
        }
        if(sameSectionOptions(m_parameters.section, options)) {
            return domain::PlanningResult<void>::success();
        }
        m_parameters.section = options;
        m_hasPendingChanges = true;
        m_section.reset();
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearCurrentTrajectory();
        if(stageAtLeast(m_stage, domain::PlanningStage::FrameConfirmed)) {
            m_stage = domain::PlanningStage::FrameConfirmed;
        }
        advanceGeneration();
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setRegionOptions(
        const domain::ToothRecognitionOptions& options)
    {
        if(!validRegionOptions(options)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Region recognition options are outside their supported ranges.");
        }
        if(sameRegionOptions(m_parameters.region, options)) {
            return domain::PlanningResult<void>::success();
        }
        m_parameters.region = options;
        m_hasPendingChanges = true;
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearCurrentTrajectory();
        if(m_section) {
            m_stage = domain::PlanningStage::SectionReady;
        } else if(stageAtLeast(m_stage, domain::PlanningStage::FrameConfirmed)) {
            m_stage = domain::PlanningStage::FrameConfirmed;
        }
        advanceGeneration();
        return domain::PlanningResult<void>::success();
    }
    domain::BoundaryMode RotationBodyPlanningSession::boundaryMode() const noexcept
    {
        return m_boundaryMode;
    }

    void RotationBodyPlanningSession::setBoundaryMode(domain::BoundaryMode mode)
    {
        if(m_boundaryMode == mode) {
            return;
        }
        m_boundaryMode = mode;
        m_hasPendingChanges = true;
        invalidateBoundary();
    }

    bool RotationBodyPlanningSession::acceptSection(
        std::uint64_t inputGeneration,
        domain::SectionContour section)
    {
        if(inputGeneration != m_generation ||
            !stageAtLeast(m_stage, domain::PlanningStage::FrameConfirmed) ||
            section.segmentCount() == 0) {
            return false;
        }
        m_section = std::move(section);
        m_hasPendingChanges = true;
        invalidateAfterSection();
        m_stage = domain::PlanningStage::SectionReady;
        return true;
    }

    bool RotationBodyPlanningSession::acceptAutomaticRegions(
        std::uint64_t inputGeneration,
        domain::RegionAssignment automaticRegions)
    {
        if(inputGeneration != m_generation || !m_section ||
            !validRegionAssignment(*m_section, automaticRegions)) {
            return false;
        }
        domain::RegionEditHistory history;
        const domain::PlanningResult<void> initialized =
            history.resetAutomatic(*m_section, automaticRegions);
        if(!initialized) {
            return false;
        }
        m_automaticRegions = std::move(automaticRegions);
        m_hasPendingChanges = true;
        m_editHistory = std::move(history);
        m_boundary.reset();
        clearCurrentTrajectory();
        m_stage = domain::PlanningStage::RegionsReady;
        advanceGeneration();
        return true;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::applyRegionOverride(
        const domain::YzRectangle& rectangle,
        domain::RegionLabel label)
    {
        if(!m_editHistory) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InsufficientRegionData,
                "Automatic regions must exist before applying an override.");
        }
        const domain::PlanningResult<void> result = m_editHistory->applyRectangle(rectangle, label);
        if(result) {
            m_hasPendingChanges = true;
            invalidateBoundary();
        }
        return result;
    }

    bool RotationBodyPlanningSession::undoRegionOverride()
    {
        if(!m_editHistory || !m_editHistory->undo()) {
            return false;
        }
        m_hasPendingChanges = true;
        invalidateBoundary();
        return true;
    }

    bool RotationBodyPlanningSession::redoRegionOverride()
    {
        if(!m_editHistory || !m_editHistory->redo()) {
            return false;
        }
        m_hasPendingChanges = true;
        invalidateBoundary();
        return true;
    }

    bool RotationBodyPlanningSession::restoreAutomaticRegions()
    {
        if(!m_editHistory) {
            return false;
        }
        m_editHistory->restoreAutomatic();
        m_hasPendingChanges = true;
        invalidateBoundary();
        return true;
    }

    bool RotationBodyPlanningSession::acceptBoundary(
        std::uint64_t inputGeneration,
        domain::SprayBoundary boundary)
    {
        if(inputGeneration != m_generation || !m_editHistory ||
            boundary.mode != m_boundaryMode || !validBoundary(boundary)) {
            return false;
        }
        m_boundary = std::move(boundary);
        m_stage = domain::PlanningStage::SprayBoundaryConfirmed;
        m_hasPendingChanges = true;
        return true;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::reopenBoundaryForEditing()
    {
        if(!m_editHistory || !m_boundary) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InsufficientRegionData,
                "Confirm a spray boundary before reopening it for editing.");
        }
        m_boundary.reset();
        m_stage = domain::PlanningStage::RegionsReady;
        clearCurrentTrajectory();
        m_hasPendingChanges = true;
        advanceGeneration();
        return domain::PlanningResult<void>::success();
    }

    const std::optional<domain::SectionContour>& RotationBodyPlanningSession::section() const noexcept
    {
        return m_section;
    }
    const std::optional<domain::RegionAssignment>&
    RotationBodyPlanningSession::automaticRegions() const noexcept
    {
        return m_automaticRegions;
    }
    std::optional<domain::RegionAssignment> RotationBodyPlanningSession::resolvedRegions() const
    {
        return m_editHistory
            ? std::optional<domain::RegionAssignment>(m_editHistory->resolved())
            : std::nullopt;
    }
    const std::optional<domain::SprayBoundary>& RotationBodyPlanningSession::boundary() const noexcept
    {
        return m_boundary;
    }
    const domain::RegionEditHistory* RotationBodyPlanningSession::editHistory() const noexcept
    {
        return m_editHistory ? &*m_editHistory : nullptr;
    }

    const domain::TrajectoryWorkspace&
    RotationBodyPlanningSession::trajectoryWorkspace() const noexcept
    {
        return m_trajectoryWorkspace;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setTrajectoryParameters(
        const domain::TrajectoryGenerationParameters& parameters)
    {
        if(!validTrajectoryParameters(parameters)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Trajectory parameters are outside their supported ranges.");
        }
        if(sameTrajectoryParameters(m_trajectoryWorkspace.parameters, parameters)) {
            return domain::PlanningResult<void>::success();
        }
        m_trajectoryWorkspace.parameters = parameters;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::generateCurrentTrajectory()
    {
        const domain::SprayBoundary* generationBoundary = m_boundary
            ? &*m_boundary
            : nullptr;
        if(!m_trajectoryWorkspace.editingPassId.empty() &&
            m_trajectoryWorkspace.currentTrajectory) {
            generationBoundary =
                &m_trajectoryWorkspace.currentTrajectory->sourceBoundary;
        }
        if(generationBoundary == nullptr) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InsufficientRegionData,
                "Confirm the spray boundary before generating a trajectory.");
        }
        domain::PlanningResult<domain::PlannedTrajectory> generated =
            domain::TrajectoryPlanner::generate(
                *generationBoundary,
                m_trajectoryWorkspace.parameters);
        if(!generated) {
            return domain::PlanningResult<void>::failure(
                generated.error.code,
                generated.error.message);
        }
        m_trajectoryWorkspace.currentTrajectory = std::move(generated.value);
        m_trajectoryWorkspace.parameters =
            m_trajectoryWorkspace.currentTrajectory->parameters;
        m_trajectoryWorkspace.displayMode = domain::TrajectoryDisplayMode::Linear;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::swapCurrentTrajectoryDirection()
    {
        if(!m_trajectoryWorkspace.currentTrajectory) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Generate or select a trajectory before swapping its direction.");
        }
        domain::PlanningResult<void> result = domain::TrajectoryEditor::swapDirection(
            *m_trajectoryWorkspace.currentTrajectory);
        if(result) {
            m_trajectoryWorkspace.parameters =
                m_trajectoryWorkspace.currentTrajectory->parameters;
            m_hasPendingChanges = true;
        }
        return result;
    }

    void RotationBodyPlanningSession::setTrajectoryDisplayMode(
        domain::TrajectoryDisplayMode mode) noexcept
    {
        if(m_trajectoryWorkspace.displayMode == mode) {
            return;
        }
        m_trajectoryWorkspace.displayMode = mode;
        m_hasPendingChanges = true;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::interpolateCurrentTrajectory(
        const std::vector<std::size_t>& selectedIndices,
        double intervalSeconds)
    {
        if(!m_trajectoryWorkspace.currentTrajectory) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Generate or select a trajectory before interpolating points.");
        }
        domain::PlanningResult<void> result = domain::TrajectoryEditor::interpolateRange(
            *m_trajectoryWorkspace.currentTrajectory,
            selectedIndices,
            intervalSeconds);
        if(result) {
            m_trajectoryWorkspace.parameters =
                m_trajectoryWorkspace.currentTrajectory->parameters;
            m_hasPendingChanges = true;
        }
        return result;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::transformCurrentTrajectoryPoints(
        const std::vector<std::size_t>& selectedIndices,
        const domain::TransformComponents& delta)
    {
        if(!m_trajectoryWorkspace.currentTrajectory) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Generate or select a trajectory before editing points.");
        }
        domain::PlanningResult<void> result = domain::TrajectoryEditor::transformSelected(
            *m_trajectoryWorkspace.currentTrajectory,
            selectedIndices,
            delta);
        if(result) {
            m_trajectoryWorkspace.parameters =
                m_trajectoryWorkspace.currentTrajectory->parameters;
            m_hasPendingChanges = true;
        }
        return result;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::beginNewTrajectory()
    {
        clearCurrentTrajectory();
        m_trajectoryWorkspace.displayMode = domain::TrajectoryDisplayMode::Linear;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::loadTrajectoryForEditing(
        const std::string& passId)
    {
        const domain::TrajectoryPass* pass = domain::TrajectoryGroupEditor::find(
            m_trajectoryWorkspace.group,
            passId);
        if(pass == nullptr) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The selected trajectory does not exist.");
        }
        m_trajectoryWorkspace.currentTrajectory = pass->trajectory;
        m_trajectoryWorkspace.parameters = pass->trajectory.parameters;
        m_trajectoryWorkspace.editingPassId = passId;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<std::string>
    RotationBodyPlanningSession::saveCurrentTrajectoryToGroup()
    {
        if(!m_trajectoryWorkspace.currentTrajectory) {
            return domain::PlanningResult<std::string>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "Generate a trajectory before saving it to the group.");
        }
        domain::PlanningResult<std::string> saved = domain::TrajectoryGroupEditor::addOrUpdate(
            m_trajectoryWorkspace.group,
            *m_trajectoryWorkspace.currentTrajectory,
            m_trajectoryWorkspace.editingPassId);
        if(saved) {
            clearCurrentTrajectory();
            m_hasPendingChanges = true;
        }
        return saved;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::removeTrajectoryPass(
        const std::string& passId)
    {
        domain::PlanningResult<void> result = domain::TrajectoryGroupEditor::remove(
            m_trajectoryWorkspace.group,
            passId);
        if(!result) {
            return result;
        }
        if(m_trajectoryWorkspace.editingPassId == passId) {
            clearCurrentTrajectory();
        }
        m_rapidSequence.erase(
            std::remove_if(
                m_rapidSequence.begin(),
                m_rapidSequence.end(),
                [&](const domain::RapidSequenceEntry& entry) {
                    return entry.kind == domain::RapidSequenceEntryKind::Trajectory &&
                        entry.trajectoryPassId == passId;
                }),
            m_rapidSequence.end());
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setTrajectoryPassVisible(
        const std::string& passId,
        bool visible)
    {
        domain::PlanningResult<void> result = domain::TrajectoryGroupEditor::setVisible(
            m_trajectoryWorkspace.group,
            passId,
            visible);
        if(result) {
            m_hasPendingChanges = true;
        }
        return result;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setTrajectoryTransitionAfter(
        const std::string& passId,
        double seconds)
    {
        domain::PlanningResult<void> result = domain::TrajectoryGroupEditor::setTransitionAfter(
            m_trajectoryWorkspace.group,
            passId,
            seconds);
        if(result) {
            m_hasPendingChanges = true;
        }
        return result;
    }

    const domain::RapidExportSettings& RotationBodyPlanningSession::rapidSettings() const noexcept
    {
        return m_rapidSettings;
    }

    const std::vector<domain::RapidSequenceEntry>&
    RotationBodyPlanningSession::rapidSequence() const noexcept
    {
        return m_rapidSequence;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setRapidSettings(
        const domain::RapidExportSettings& settings)
    {
        if(!settings.safetyPositionBaseMeters.allFinite() ||
            !std::isfinite(settings.safetySpeedMetersPerSecond) ||
            settings.safetySpeedMetersPerSecond <= 0.0 ||
            !domain::RapidModuleGenerator::isValidRapidIdentifier(settings.moduleName) ||
            !domain::RapidModuleGenerator::isValidRapidIdentifier(settings.toolDataName) ||
            !validRapidFileName(settings.fileName)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "ABB RAPID settings are invalid.");
        }
        m_rapidSettings = settings;
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setRapidSequence(
        std::vector<domain::RapidSequenceEntry> sequence)
    {
        for(const domain::RapidSequenceEntry& entry : sequence) {
            if(entry.kind == domain::RapidSequenceEntryKind::Trajectory &&
                domain::TrajectoryGroupEditor::find(
                    m_trajectoryWorkspace.group,
                    entry.trajectoryPassId) == nullptr) {
                return domain::PlanningResult<void>::failure(
                    domain::PlanningErrorCode::InvalidArgument,
                    "The ABB instruction sequence references a missing trajectory.");
            }
        }
        m_rapidSequence = std::move(sequence);
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    void RotationBodyPlanningSession::setDefaultRapidOutputDirectory(std::string directory)
    {
        if(!m_rapidSettings.outputDirectory.empty() || directory.empty()) {
            return;
        }
        m_rapidSettings.outputDirectory = std::move(directory);
    }

    const WorkpieceCalibrationWorkspace&
    RotationBodyPlanningSession::calibrationWorkspace() const noexcept
    {
        return m_calibrationWorkspace;
    }

    domain::PlanningResult<void> RotationBodyPlanningSession::setCalibrationWorkspace(
        WorkpieceCalibrationWorkspace workspace)
    {
        if(!validCalibrationWorkspace(workspace)) {
            return domain::PlanningResult<void>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The workpiece calibration workspace is invalid.");
        }
        m_calibrationWorkspace = std::move(workspace);
        m_hasPendingChanges = true;
        return domain::PlanningResult<void>::success();
    }

    const RotationBodyUiState& RotationBodyPlanningSession::uiState() const noexcept
    {
        return m_uiState;
    }

    void RotationBodyPlanningSession::setUiState(const RotationBodyUiState& state) noexcept
    {
        if(m_uiState.workflow == state.workflow &&
            m_uiState.mainViewMode == state.mainViewMode &&
            m_uiState.rightWorkflow == state.rightWorkflow) {
            return;
        }
        m_uiState = state;
        if(hasModel()) {
            m_hasPendingChanges = true;
        }
    }
    const domain::PlanningError& RotationBodyPlanningSession::error() const noexcept { return m_error; }
    void RotationBodyPlanningSession::setError(domain::PlanningError error) { m_error = std::move(error); }
    void RotationBodyPlanningSession::clearError() noexcept { m_error = {}; }

    bool RotationBodyPlanningSession::validTransform(const Eigen::Isometry3d& transform) noexcept
    {
        if(!transform.matrix().allFinite()) {
            return false;
        }
        const Eigen::Matrix3d shouldBeIdentity = transform.linear().transpose() * transform.linear();
        return shouldBeIdentity.isApprox(Eigen::Matrix3d::Identity(), 1.0e-9) &&
            std::abs(transform.linear().determinant() - 1.0) <= 1.0e-9;
    }

    bool RotationBodyPlanningSession::validSectionOptions(
        const domain::YzSectionOptions& options) noexcept
    {
        return std::isfinite(options.relativePlaneTolerance) &&
            std::isfinite(options.relativeWeldTolerance) &&
            std::isfinite(options.minimumToleranceMeters) &&
            options.relativePlaneTolerance > 0.0 &&
            options.relativePlaneTolerance <= 1.0e-4 &&
            options.relativeWeldTolerance > 0.0 &&
            options.relativeWeldTolerance <= 1.0e-3 &&
            options.minimumToleranceMeters >= 0.0;
    }

    bool RotationBodyPlanningSession::validRegionOptions(
        const domain::ToothRecognitionOptions& options) noexcept
    {
        return options.minimumResampleCount >= 8 &&
            options.maximumResampleCount >= options.minimumResampleCount &&
            std::isfinite(options.smoothingWindowFraction) &&
            options.smoothingWindowFraction >= 0.0 &&
            std::isfinite(options.minimumRadialProminenceFraction) &&
            options.minimumRadialProminenceFraction > 0.0 &&
            std::isfinite(options.axialSurfaceThreshold) &&
            options.axialSurfaceThreshold > 0.5 &&
            options.axialSurfaceThreshold < 1.0 &&
            std::isfinite(options.minimumConfidence) &&
            options.minimumConfidence >= 0.0 &&
            options.minimumConfidence <= 1.0 &&
            std::isfinite(options.transitionExtentFraction) &&
            options.transitionExtentFraction >= 0.0;
    }

    bool RotationBodyPlanningSession::validRegionAssignment(
        const domain::SectionContour& section,
        const domain::RegionAssignment& assignment) noexcept
    {
        if(!assignment.matches(section)) {
            return false;
        }
        if(section.segmentCount() == 0) {
            return false;
        }
        if(assignment.segmentConfidence.empty()) {
            return true;
        }
        if(assignment.segmentConfidence.size() != section.segmentCount()) {
            return false;
        }
        return std::all_of(
            assignment.segmentConfidence.begin(),
            assignment.segmentConfidence.end(),
            [](double confidence) {
                return std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0;
            });
    }

    bool RotationBodyPlanningSession::validBoundary(
        const domain::SprayBoundary& boundary) noexcept
    {
        if(boundary.polygonYz.size() < 3 || !std::isfinite(boundary.minimumY) ||
            !std::isfinite(boundary.maximumY) || !std::isfinite(boundary.minimumZ) ||
            !std::isfinite(boundary.maximumZ) ||
            !std::isfinite(boundary.outerLineSlopeYPerZ) ||
            !std::isfinite(boundary.outerLineInterceptY)) {
            return false;
        }
        return std::all_of(
            boundary.polygonYz.begin(),
            boundary.polygonYz.end(),
            [](const Eigen::Vector2d& point) { return point.allFinite(); });
    }

    void RotationBodyPlanningSession::invalidateAfterPlanningTransform()
    {
        m_section.reset();
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearAllTrajectories();
        m_stage = domain::PlanningStage::ModelLoaded;
        advanceGeneration();
    }

    void RotationBodyPlanningSession::invalidateAfterSection()
    {
        m_automaticRegions.reset();
        m_editHistory.reset();
        m_boundary.reset();
        clearCurrentTrajectory();
        advanceGeneration();
    }

    void RotationBodyPlanningSession::invalidateBoundary()
    {
        m_boundary.reset();
        clearCurrentTrajectory();
        if(m_editHistory) {
            m_stage = domain::PlanningStage::RegionsReady;
        }
        advanceGeneration();
    }

    void RotationBodyPlanningSession::clearCurrentTrajectory() noexcept
    {
        m_trajectoryWorkspace.currentTrajectory.reset();
        m_trajectoryWorkspace.editingPassId.clear();
    }

    void RotationBodyPlanningSession::clearAllTrajectories() noexcept
    {
        const domain::TrajectoryGenerationParameters parameters =
            m_trajectoryWorkspace.parameters;
        m_trajectoryWorkspace = {};
        m_trajectoryWorkspace.parameters = parameters;
        m_rapidSequence.clear();
    }

    void RotationBodyPlanningSession::advanceGeneration() noexcept
    {
        ++m_generation;
        if(m_generation == 0) {
            ++m_generation;
        }
    }
}
