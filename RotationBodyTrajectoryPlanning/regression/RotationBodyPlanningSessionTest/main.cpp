#include "Adapters/RotationBodyMeshAdapter.h"
#include "Controllers/RotationBodyPlanningController.h"
#include "Controllers/RotationBodyPlanningWorkflowCoordinator.h"
#include "FakeViewportServices.h"
#include "Models/RotationBodyPlanningSession.h"
#include "Models/RotationBodyPlanningTranslations.h"
#include "Persistence/RotationBodyPlanningDocumentOperations.h"
#include "Persistence/RotationBodyPlanningDraftStore.h"
#include "Persistence/RotationBodyTrajectoryProjectStore.h"
#include "Widgets/ModelTransformPanel.h"
#include "Widgets/RotationBodyPlanningLeftPanel.h"
#include "Widgets/RotationBodyPlanningRightPanel.h"
#include "Widgets/RotationBodyWorkflowNavigation.h"
#include "Widgets/SectionRegionPanel.h"
#include "Widgets/SectionView.h"

#include <AssetCore/ModelDesc.h>
#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerOperationStatus.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportPreviewState.h"

#include <RotationBodyTrajectoryPlanning/Alignment/RotationBodyAlignmentSolver.h>

#include <Eigen/Geometry>

#include <glm/glm.hpp>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QToolButton>
#include <QThread>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ROTATION_BODY_TEST_SOURCE_ROOT
#error ROTATION_BODY_TEST_SOURCE_ROOT must identify the repository source root.
#endif

namespace
{
    namespace app = smrobot::workbench::spray::rotationbody;
    namespace domain = smrobot::spray::rotationbody;

    int failures = 0;

    void expect(bool condition, const std::string& message)
    {
        if(!condition) {
            ++failures;
            std::cerr << "FAILED: " << message << '\n';
        }
    }

    struct ControllerFixture
    {
        simulation_project::ProjectSession projectSession;
        robot_qt_viewer::RobotQtViewerEventHub eventHub;
        robot_qt_viewer::RobotQtViewerDocumentController documentController;
        robot_qt_viewer::RobotQtViewerSelectionModel selectionModel;
        robot_qt_viewer::RobotQtViewerViewportPreviewState previewState;
        robot_qt_viewer::RobotQtViewerOperationStatusStore operationStatusStore;
        robot_qt_viewer::RobotQtViewerDocumentContext context;
        FakeRotationBodyViewportServices viewport;
        app::RotationBodyPlanningController controller;

        ControllerFixture()
            : documentController(projectSession, eventHub)
            , selectionModel(eventHub)
            , previewState(eventHub)
            , operationStatusStore(eventHub)
            , context(
                projectSession,
                documentController,
                selectionModel,
                previewState,
                eventHub,
                operationStatusStore)
            , controller(context)
        {
            context.setViewportServices(&viewport);
        }
    };

    domain::TriangleMesh tetrahedron(double offset = 0.0)
    {
        domain::TriangleMesh mesh;
        mesh.positions = {
            { offset + 0.0, 0.0, 0.0 },
            { offset + 1.0, 0.0, 0.0 },
            { offset + 0.0, 1.0, 0.0 },
            { offset + 0.0, 0.0, 1.0 }
        };
        mesh.triangles = {
            { 0, 2, 1 },
            { 0, 1, 3 },
            { 0, 3, 2 },
            { 1, 2, 3 }
        };
        return mesh;
    }

    domain::TriangleMesh box(
        const Eigen::Vector3d& minimum,
        const Eigen::Vector3d& maximum)
    {
        domain::TriangleMesh mesh;
        mesh.positions = {
            { minimum.x(), minimum.y(), minimum.z() },
            { maximum.x(), minimum.y(), minimum.z() },
            { maximum.x(), maximum.y(), minimum.z() },
            { minimum.x(), maximum.y(), minimum.z() },
            { minimum.x(), minimum.y(), maximum.z() },
            { maximum.x(), minimum.y(), maximum.z() },
            { maximum.x(), maximum.y(), maximum.z() },
            { minimum.x(), maximum.y(), maximum.z() }
        };
        mesh.triangles = {
            { 0, 2, 1 }, { 0, 3, 2 },
            { 4, 5, 6 }, { 4, 6, 7 },
            { 0, 1, 5 }, { 0, 5, 4 },
            { 1, 2, 6 }, { 1, 6, 5 },
            { 2, 3, 7 }, { 2, 7, 6 },
            { 3, 0, 4 }, { 3, 4, 7 }
        };
        return mesh;
    }

    bool processEventsUntil(
        const std::function<bool()>& predicate,
        int timeoutMs = 15000)
    {
        QElapsedTimer timer;
        timer.start();
        while(!predicate() && timer.elapsed() < timeoutMs) {
            QApplication::processEvents(QEventLoop::AllEvents, 20);
            QThread::msleep(1);
        }
        QApplication::processEvents(QEventLoop::AllEvents, 20);
        return predicate();
    }

    domain::TriangleMesh revolvedProfile(
        const std::vector<double>& axialStations,
        const std::vector<double>& radii,
        std::size_t sectorCount = 32)
    {
        domain::TriangleMesh mesh;
        constexpr double pi = 3.14159265358979323846;
        for(std::size_t station = 0; station < axialStations.size(); ++station) {
            for(std::size_t sector = 0; sector < sectorCount; ++sector) {
                const double angle = 2.0 * pi * static_cast<double>(sector) /
                    static_cast<double>(sectorCount);
                mesh.positions.push_back({
                    radii[station] * std::cos(angle),
                    radii[station] * std::sin(angle),
                    axialStations[station]
                });
            }
        }
        for(std::size_t station = 0; station + 1 < axialStations.size(); ++station) {
            for(std::size_t sector = 0; sector < sectorCount; ++sector) {
                const std::size_t next = (sector + 1) % sectorCount;
                const int lower = static_cast<int>(station * sectorCount + sector);
                const int lowerNext = static_cast<int>(station * sectorCount + next);
                const int upper = static_cast<int>((station + 1) * sectorCount + sector);
                const int upperNext = static_cast<int>((station + 1) * sectorCount + next);
                mesh.triangles.push_back({ lower, lowerNext, upperNext });
                mesh.triangles.push_back({ lower, upperNext, upper });
            }
        }
        const int lowerCenter = static_cast<int>(mesh.positions.size());
        mesh.positions.push_back({ 0.0, 0.0, axialStations.front() });
        const int upperCenter = static_cast<int>(mesh.positions.size());
        mesh.positions.push_back({ 0.0, 0.0, axialStations.back() });
        const int upperOffset = static_cast<int>((axialStations.size() - 1) * sectorCount);
        for(std::size_t sector = 0; sector < sectorCount; ++sector) {
            const int current = static_cast<int>(sector);
            const int next = static_cast<int>((sector + 1) % sectorCount);
            mesh.triangles.push_back({ lowerCenter, next, current });
            mesh.triangles.push_back({ upperCenter, upperOffset + current, upperOffset + next });
        }
        return mesh;
    }

    domain::AlignmentResult identityAlignment(const domain::TriangleMesh& mesh)
    {
        domain::AlignmentResult result;
        result.statistics.vertexCount = mesh.vertexCount();
        result.statistics.triangleCount = mesh.triangleCount();
        result.statistics.boundsMinimum = Eigen::Vector3d::Zero();
        result.statistics.boundsMaximum = Eigen::Vector3d::Ones();
        result.statistics.estimatedAxisInMesh = Eigen::Vector3d::UnitZ();
        result.statistics.heightMeters = 1.0;
        result.statistics.maximumDiameterMeters = 2.0;
        result.statistics.minimumDiameterMeters = 1.0;
        result.statistics.axisConfidence = 0.9;
        return result;
    }

    domain::SectionContour sampleSection()
    {
        domain::SectionContour section;
        section.pointsYz = { { 0.1, 0.0 }, { 0.2, 0.5 }, { 0.15, 1.0 } };
        section.points3d = { { 0.0, 0.1, 0.0 }, { 0.0, 0.2, 0.5 }, { 0.0, 0.15, 1.0 } };
        section.cumulativeArcLength = { 0.0, 0.51, 1.02 };
        section.toleranceMeters = 1.0e-8;
        return section;
    }

    domain::SectionContour toothRegionSection()
    {
        domain::SectionContour contour;
        contour.pointsYz = {
            { 1.0, 0.00 }, { 1.0, 0.18 }, { 1.03, 0.23 }, { 1.0, 0.28 },
            { 1.22, 0.28 }, { 1.22, 0.36 }, { 1.0, 0.36 }, { 1.0, 0.42 },
            { 1.24, 0.42 }, { 1.24, 0.50 }, { 1.0, 0.50 }, { 1.0, 0.56 },
            { 1.21, 0.56 }, { 1.21, 0.64 }, { 1.0, 0.64 }, { 1.03, 0.69 },
            { 1.0, 0.74 }, { 1.0, 0.92 }
        };
        contour.points3d.reserve(contour.pointsYz.size());
        for(const Eigen::Vector2d& point : contour.pointsYz) {
            contour.points3d.push_back({ 0.0, point.x(), point.y() });
        }
        contour.closed = false;
        contour.toleranceMeters = 1.0e-8;
        return contour;
    }

    domain::RegionAssignment sampleRegions()
    {
        domain::RegionAssignment regions;
        regions.segmentLabels = { domain::RegionLabel::ToothTop, domain::RegionLabel::ToothWall };
        regions.segmentConfidence = { 0.9, 0.8 };
        return regions;
    }

    domain::SprayBoundary sampleBoundary()
    {
        domain::SprayBoundary boundary;
        boundary.mode = domain::BoundaryMode::ToothTopEnvelope;
        boundary.polygonYz = { { 0.1, 0.0 }, { 0.3, 0.0 }, { 0.25, 1.0 }, { 0.1, 1.0 } };
        boundary.minimumY = 0.1;
        boundary.maximumY = 0.3;
        boundary.minimumZ = 0.0;
        boundary.maximumZ = 1.0;
        boundary.outerLineSlopeYPerZ = -0.05;
        boundary.outerLineInterceptY = 0.3;
        return boundary;
    }

    app::RotationBodyPlanningSession populatedSession()
    {
        app::RotationBodyPlanningSession session;
        domain::TriangleMesh mesh = tetrahedron();
        const auto loaded = session.loadModel(
            "workpiece_1",
            "${DATA_DIR}/Spray420/test.stl",
            "fingerprint-a",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::SimulationBlock,
            domain::SignedAxis::NegativeX,
            domain::SignedAxis::PositiveY,
            0.42);
        expect(loaded.ok(), "session accepts a valid aligned model");
        domain::YzSectionOptions sectionOptions;
        sectionOptions.relativePlaneTolerance = 2.0e-9;
        sectionOptions.relativeWeldTolerance = 9.0e-9;
        sectionOptions.minimumToleranceMeters = 2.0e-12;
        expect(session.setSectionOptions(sectionOptions).ok(),
            "session accepts non-default section options");
        domain::ToothRecognitionOptions regionOptions;
        regionOptions.minimumResampleCount = 128;
        regionOptions.maximumResampleCount = 1536;
        regionOptions.smoothingWindowFraction = 0.02;
        regionOptions.minimumRadialProminenceFraction = 0.03;
        regionOptions.axialSurfaceThreshold = 0.7;
        regionOptions.minimumConfidence = 0.35;
        regionOptions.transitionExtentFraction = 0.04;
        expect(session.setRegionOptions(regionOptions).ok(),
            "session accepts non-default region options");
        Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
        base.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
        expect(session.setBaseFromPlanning(base).ok(), "session accepts T_BP");
        session.setPublishFrame(app::PublishFrame::PlanningLocalFrame);
        expect(session.confirmFrame().ok(), "session confirms the planning frame");
        std::uint64_t generation = session.generation();
        expect(session.acceptSection(generation, sampleSection()), "session accepts current section result");
        generation = session.generation();
        expect(session.acceptAutomaticRegions(generation, sampleRegions()),
            "session accepts current automatic regions");
        const domain::YzRectangle rectangle{ { 0.15, 0.2 }, { 0.25, 0.8 } };
        expect(session.applyRegionOverride(rectangle, domain::RegionLabel::Transition).ok(),
            "session records a region override");
        session.setBoundaryMode(domain::BoundaryMode::ToothTopEnvelope);
        generation = session.generation();
        expect(session.acceptBoundary(generation, sampleBoundary()),
            "session accepts a boundary for the selected mode");
        return session;
    }

    void testSessionGenerationAndInvalidation()
    {
        app::RotationBodyPlanningSession session;
        expect(!session.hasPendingChanges(),
            "a new planning session has no pending changes");
        domain::TriangleMesh mesh = tetrahedron();
        expect(session.loadModel(
            "workpiece_1",
            "model.stl",
            "fingerprint-a",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::CompletePart).ok(),
            "loadModel succeeds");
        expect(session.hasPendingChanges(),
            "loading a model marks planning progress pending");
        session.markProgressSaved();
        expect(!session.hasPendingChanges(),
            "saving progress clears the pending flag");
        const std::uint64_t loadGeneration = session.generation();
        expect(session.confirmFrame().ok(), "confirmFrame succeeds");
        expect(session.hasPendingChanges(),
            "confirming the planning frame marks progress pending");
        expect(session.generation() != loadGeneration, "confirming frame changes generation");
        expect(!session.acceptSection(loadGeneration, sampleSection()),
            "stale section result is rejected");
        expect(session.acceptSection(session.generation(), sampleSection()),
            "current section result is accepted");
        const std::uint64_t sectionGeneration = session.generation();
        domain::RegionAssignment wrongConfidenceCount = sampleRegions();
        wrongConfidenceCount.segmentConfidence.pop_back();
        expect(!session.acceptAutomaticRegions(sectionGeneration, wrongConfidenceCount),
            "region confidence count must match the contour");
        domain::RegionAssignment nonFiniteConfidence = sampleRegions();
        nonFiniteConfidence.segmentConfidence.front() =
            std::numeric_limits<double>::quiet_NaN();
        expect(!session.acceptAutomaticRegions(sectionGeneration, nonFiniteConfidence),
            "non-finite region confidence is rejected");
        domain::RegionAssignment outOfRangeConfidence = sampleRegions();
        outOfRangeConfidence.segmentConfidence.front() = 1.01;
        expect(!session.acceptAutomaticRegions(sectionGeneration, outOfRangeConfidence),
            "out-of-range region confidence is rejected");
        expect(!session.acceptAutomaticRegions(loadGeneration, sampleRegions()),
            "stale region result is rejected");
        expect(session.acceptAutomaticRegions(sectionGeneration, sampleRegions()),
            "current region result is accepted");
        session.setBoundaryMode(domain::BoundaryMode::ToothTopEnvelope);
        expect(!session.acceptBoundary(sectionGeneration, sampleBoundary()),
            "stale boundary result is rejected");
        expect(session.acceptBoundary(session.generation(), sampleBoundary()),
            "current boundary result is accepted");

        session.markProgressSaved();
        const std::uint64_t confirmedGeneration = session.generation();
        const Eigen::Isometry3d confirmedTransform = session.planningFromMesh();
        expect(session.confirmFrame().ok(),
            "reconfirming an unchanged planning frame succeeds");
        expect(session.generation() == confirmedGeneration &&
            session.stage() == domain::PlanningStage::SprayBoundaryConfirmed &&
            session.section().has_value() && session.automaticRegions().has_value() &&
            session.boundary().has_value() && !session.hasPendingChanges(),
            "reconfirming an unchanged frame preserves all derived planning state");
        expect(session.setPlanningFromMesh(confirmedTransform).ok() &&
            session.generation() == confirmedGeneration &&
            session.stage() == domain::PlanningStage::SprayBoundaryConfirmed,
            "setting the identical planning transform is idempotent");

        const std::uint64_t beforeBaseChange = session.generation();
        Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
        base.translation().x() = 0.25;
        expect(session.setBaseFromPlanning(base).ok(), "T_BP change succeeds");
        expect(session.generation() == beforeBaseChange,
            "T_BP does not invalidate local geometry generation");
        expect(session.boundary().has_value(), "T_BP preserves boundary");

        domain::ToothRecognitionOptions changedRegionOptions = session.parameters().region;
        changedRegionOptions.minimumConfidence = 0.41;
        const std::uint64_t beforeRegionOptions = session.generation();
        expect(session.setRegionOptions(changedRegionOptions).ok(),
            "valid region options update succeeds");
        expect(session.generation() != beforeRegionOptions &&
            session.stage() == domain::PlanningStage::SectionReady,
            "region option change advances generation and returns to SectionReady");
        expect(session.section().has_value() && !session.automaticRegions() && !session.boundary(),
            "region option change preserves section and clears regions and boundary");
        domain::RegionAssignment confidenceOptional = sampleRegions();
        confidenceOptional.segmentConfidence.clear();
        expect(session.acceptAutomaticRegions(session.generation(), confidenceOptional),
            "empty optional confidence vector is accepted");
        expect(session.acceptBoundary(session.generation(), sampleBoundary()),
            "boundary can be rebuilt after region option invalidation");

        domain::YzSectionOptions changedSectionOptions = session.parameters().section;
        changedSectionOptions.relativePlaneTolerance = 3.0e-9;
        const std::uint64_t beforeSectionOptions = session.generation();
        expect(session.setSectionOptions(changedSectionOptions).ok(),
            "valid section options update succeeds");
        expect(session.generation() != beforeSectionOptions &&
            session.stage() == domain::PlanningStage::FrameConfirmed,
            "section option change advances generation and returns to FrameConfirmed");
        expect(!session.section() && !session.automaticRegions() && !session.boundary(),
            "section option change clears every derived geometry result");

        domain::YzSectionOptions invalidSectionOptions = changedSectionOptions;
        invalidSectionOptions.relativeWeldTolerance = 0.0;
        const std::uint64_t beforeInvalidSection = session.generation();
        expect(!session.setSectionOptions(invalidSectionOptions),
            "invalid section options are rejected");
        expect(session.generation() == beforeInvalidSection,
            "rejected section options do not change generation");
        domain::ToothRecognitionOptions invalidRegionOptions = changedRegionOptions;
        invalidRegionOptions.axialSurfaceThreshold = 1.0;
        expect(!session.setRegionOptions(invalidRegionOptions),
            "invalid region options are rejected");

        Eigen::Isometry3d planning = session.planningFromMesh();
        planning.translation().y() += 0.01;
        expect(session.setPlanningFromMesh(planning).ok(), "T_PM change succeeds");
        expect(session.stage() == domain::PlanningStage::ModelLoaded,
            "T_PM change requires frame confirmation again");
        expect(!session.section() && !session.automaticRegions() && !session.boundary(),
            "T_PM change clears all downstream results");
    }

    void testSessionClearRestoresDefaults()
    {
        app::RotationBodyPlanningSession session;
        domain::YzSectionOptions sectionOptions;
        sectionOptions.relativePlaneTolerance = 4.0e-9;
        sectionOptions.relativeWeldTolerance = 8.0e-9;
        sectionOptions.minimumToleranceMeters = 3.0e-12;
        domain::ToothRecognitionOptions regionOptions;
        regionOptions.minimumResampleCount = 160;
        regionOptions.maximumResampleCount = 1400;
        regionOptions.smoothingWindowFraction = 0.025;
        regionOptions.minimumRadialProminenceFraction = 0.04;
        regionOptions.axialSurfaceThreshold = 0.72;
        regionOptions.minimumConfidence = 0.4;
        regionOptions.transitionExtentFraction = 0.05;
        expect(session.setSectionOptions(sectionOptions).ok() &&
            session.setRegionOptions(regionOptions).ok(),
            "clear regression starts with non-default algorithm options");
        const std::uint64_t beforeClear = session.generation();
        session.clear();
        const domain::YzSectionOptions defaultSection;
        const domain::ToothRecognitionOptions defaultRegion;
        expect(session.stage() == domain::PlanningStage::NoModel &&
            session.generation() != beforeClear,
            "clear returns to NoModel and advances generation");
        expect(session.parameters().section.relativePlaneTolerance ==
                defaultSection.relativePlaneTolerance &&
            session.parameters().section.relativeWeldTolerance ==
                defaultSection.relativeWeldTolerance &&
            session.parameters().section.minimumToleranceMeters ==
                defaultSection.minimumToleranceMeters,
            "clear restores all default section options");
        expect(session.parameters().region.minimumResampleCount ==
                defaultRegion.minimumResampleCount &&
            session.parameters().region.maximumResampleCount ==
                defaultRegion.maximumResampleCount &&
            session.parameters().region.smoothingWindowFraction ==
                defaultRegion.smoothingWindowFraction &&
            session.parameters().region.minimumRadialProminenceFraction ==
                defaultRegion.minimumRadialProminenceFraction &&
            session.parameters().region.axialSurfaceThreshold ==
                defaultRegion.axialSurfaceThreshold &&
            session.parameters().region.minimumConfidence ==
                defaultRegion.minimumConfidence &&
            session.parameters().region.transitionExtentFraction ==
                defaultRegion.transitionExtentFraction,
            "clear restores all default region options");
    }

    void testTrajectoryWorkspacePersistenceAndInvalidation()
    {
        app::RotationBodyPlanningSession session = populatedSession();
        domain::TrajectoryGenerationParameters parameters;
        parameters.sprayDistanceMeters = 0.08;
        parameters.tiltRadians = 0.15;
        parameters.speedMetersPerSecond = 0.04;
        parameters.startExtensionMeters = 0.01;
        parameters.endExtensionMeters = 0.02;
        parameters.pointCount = 7;
        parameters.positionerRpm = 4.5;
        expect(session.setTrajectoryParameters(parameters).ok() &&
            session.generateCurrentTrajectory().ok(),
            "session generates a current linear and relative helical trajectory");
        const auto saved = session.saveCurrentTrajectoryToGroup();
        expect(saved.ok() && session.trajectoryWorkspace().group.passes.size() == 1 &&
            !session.trajectoryWorkspace().currentTrajectory.has_value() &&
            session.trajectoryWorkspace().editingPassId.empty(),
            "saving finalizes the trajectory and leaves group checkboxes as the only visibility source");
        if(!saved) {
            return;
        }

        domain::RapidExportSettings rapidSettings;
        rapidSettings.safetyPositionBaseMeters = { 0.5, 0.6, 0.7 };
        rapidSettings.outputDirectory = "data/ABBrapid";
        expect(session.setRapidSettings(rapidSettings).ok(),
            "session accepts concise ABB RAPID settings");
        std::vector<domain::RapidSequenceEntry> sequence(2);
        sequence[0].kind = domain::RapidSequenceEntryKind::SafetyPoint;
        sequence[1].kind = domain::RapidSequenceEntryKind::Trajectory;
        sequence[1].trajectoryPassId = saved.value;
        expect(session.setRapidSequence(sequence).ok(),
            "session accepts a repeated-safe-point-capable ABB instruction sequence");

        simulation_project::ProjectDocument document;
        expect(app::RotationBodyTrajectoryProjectStore::writeWorkspace(
            document,
            session.makeTrajectoryDraft()),
            "private trajectory workspace is written to a dedicated extension");
        expect(app::RotationBodyTrajectoryProjectStore::writePublishedPlan(
            document,
            session.makePublishedTrajectoryPlan()),
            "public finalized trajectory plan is written to a separate extension");
        const app::TrajectoryWorkspaceReadResult workspaceRead =
            app::RotationBodyTrajectoryProjectStore::readWorkspace(document);
        const app::PublishedTrajectoryReadResult publishedRead =
            app::RotationBodyTrajectoryProjectStore::readPublishedPlan(document);
        expect(workspaceRead.usable() && workspaceRead.draft.has_value() &&
            publishedRead.usable() && publishedRead.plan.has_value(),
            "both trajectory extensions round-trip as structured versioned data");

        app::RotationBodyPlanningSession restored = populatedSession();
        expect(workspaceRead.draft &&
            restored.restoreTrajectoryDraft(*workspaceRead.draft).ok(),
            "matching workpiece identity restores private trajectory progress");
        expect(restored.trajectoryWorkspace().group.passes.size() == 1 &&
            restored.rapidSequence().size() == 2 &&
            !restored.trajectoryWorkspace().currentTrajectory.has_value(),
            "trajectory group and ABB sequence survive restoration without a stale current draft");

        if(workspaceRead.draft) {
            app::RotationBodyTrajectoryDraft legacyDraft = *workspaceRead.draft;
            legacyDraft.workspace.currentTrajectory =
                legacyDraft.workspace.group.passes.front().trajectory;
            legacyDraft.workspace.editingPassId =
                legacyDraft.workspace.group.passes.front().id;
            app::RotationBodyPlanningSession legacyRestored = populatedSession();
            expect(legacyRestored.restoreTrajectoryDraft(legacyDraft).ok() &&
                !legacyRestored.trajectoryWorkspace().currentTrajectory.has_value() &&
                legacyRestored.trajectoryWorkspace().editingPassId.empty(),
                "legacy exact saved-pass copies are removed so unchecked trajectories stay hidden");
        }

        expect(restored.reopenBoundaryForEditing().ok(),
            "confirmed yellow boundary can be reopened for region correction");
        expect(!restored.boundary().has_value() &&
            !restored.trajectoryWorkspace().currentTrajectory.has_value() &&
            restored.trajectoryWorkspace().group.passes.size() == 1 &&
            restored.trajectoryWorkspace().parameters.pointCount == 7,
            "reopening the boundary clears only the current trajectory and preserves group and parameters");

        Eigen::Isometry3d changedPlanning = restored.planningFromMesh();
        changedPlanning.translation().x() += 0.001;
        expect(restored.setPlanningFromMesh(changedPlanning).ok() &&
            restored.trajectoryWorkspace().group.passes.empty() &&
            restored.rapidSequence().empty(),
            "changing the planning transform clears all dependent trajectories and ABB references");
    }

    void testDraftRoundTripAndDegradation()
    {
        app::RotationBodyPlanningSession session = populatedSession();
        app::WorkpieceCalibrationWorkspace calibration;
        calibration.cylinderRows[0] = {
            std::optional<double>(1.0),
            std::optional<double>(2.0),
            std::optional<double>(3.0)
        };
        calibration.circleRows[2][1] = 0.125;
        calibration.topReference[2] = 0.9;
        calibration.workpieceHeightMeters = 0.45;
        calibration.activeMode = domain::CalibrationMode::Circle2d;
        calibration.axisSource = domain::CalibrationMode::Circle2d;
        calibration.showCylinder = false;
        calibration.showCircle = true;
        domain::CalibrationAxisFit storedFit;
        storedFit.mode = domain::CalibrationMode::Circle2d;
        storedFit.axisPointBaseMeters = { 1.0, 2.0, 3.0 };
        storedFit.axisDirectionBase = Eigen::Vector3d::UnitZ();
        storedFit.radiusMeters = 0.2;
        storedFit.rmsResidualMeters = 0.0001;
        storedFit.maximumResidualMeters = 0.0002;
        storedFit.iterations = 4;
        storedFit.sourcePointsBaseMeters.push_back({ 1.2, 2.0, 3.0 });
        calibration.circleFit = storedFit;
        domain::WorkpieceFrameCalibration storedFrame;
        storedFrame.baseFromPlanning.translation() =
            Eigen::Vector3d(1.0, 2.0, 2.55);
        storedFrame.baseFromPlanningComponents.translationMeters =
            storedFrame.baseFromPlanning.translation();
        storedFrame.topReferenceBaseMeters = { 1.2, 2.0, 3.0 };
        storedFrame.topCenterBaseMeters = { 1.0, 2.0, 3.0 };
        storedFrame.baseOriginBaseMeters = { 1.0, 2.0, 2.55 };
        storedFrame.yDirectionStartBaseMeters = { 1.0, 2.0, 2.55 };
        storedFrame.yDirectionEndBaseMeters = { 1.0, 2.1, 2.55 };
        storedFrame.workpieceHeightMeters = 0.45;
        storedFrame.fittedRadiusMeters = 0.2;
        storedFrame.yDirectionDistanceMeters = 0.1;
        calibration.frame = storedFrame;
        expect(session.setCalibrationWorkspace(calibration).ok(),
            "session accepts module-private calibration progress");
        app::RotationBodyUiState uiState;
        uiState.workflow = app::RotationBodyWorkflow::SectionRegionPlanning;
        uiState.mainViewMode = app::RotationBodyMainViewMode::Section;
        uiState.rightWorkflow = app::RotationBodyRightWorkflow::WorkpieceCalibration;
        session.setUiState(uiState);
        app::RotationBodyPlanningDraft original = session.makeDraft();
        simulation_project::ProjectDocument document;
        expect(app::RotationBodyPlanningDraftStore::write(document, original),
            "first draft write changes the document");
        expect(!app::RotationBodyPlanningDraftStore::write(document, original),
            "identical draft write is idempotent");

        const app::DraftReadResult read = app::RotationBodyPlanningDraftStore::read(document);
        expect(read.status == app::DraftReadStatus::Loaded && read.draft.has_value(),
            "version 1 draft reads successfully");
        if(read.draft) {
            expect(read.draft->schemaVersion == 1 && read.draft->objectId == original.objectId,
                "schema version and object id round trip");
            expect(read.draft->sourcePath == original.sourcePath &&
                read.draft->sourceFingerprint == original.sourceFingerprint &&
                read.draft->meshFingerprint == original.meshFingerprint,
                "source identity and fingerprints round trip");
            expect(read.draft->objectType == domain::PlanningObjectType::SimulationBlock,
                "planning type round trips");
            expect(read.draft->originalRotationAxis == domain::SignedAxis::NegativeX &&
                read.draft->toothOutwardAxis == domain::SignedAxis::PositiveY,
                "both axis selections round trip");
            expect(std::abs(read.draft->motherMaximumDiameterMeters - 0.42) < 1.0e-12,
                "simulation diameter round trips in meters");
            expect(read.draft->planningFromMesh.matrix().isApprox(original.planningFromMesh.matrix()),
                "T_PM round trips");
            expect(read.draft->baseFromPlanning.matrix().isApprox(original.baseFromPlanning.matrix()),
                "T_BP round trips");
            expect(read.draft->automaticBaseline.matrix().isApprox(original.automaticBaseline.matrix()) &&
                read.draft->directedRotaryAxisInMesh.isApprox(original.directedRotaryAxisInMesh) &&
                read.draft->bottomAxisCenterInMesh.isApprox(original.bottomAxisCenterInMesh),
                "automatic alignment metadata round trips");
            expect(read.draft->statistics.vertexCount == original.statistics.vertexCount &&
                read.draft->statistics.triangleCount == original.statistics.triangleCount &&
                read.draft->statistics.boundsMinimum.isApprox(original.statistics.boundsMinimum) &&
                read.draft->statistics.boundsMaximum.isApprox(original.statistics.boundsMaximum) &&
                read.draft->statistics.estimatedAxisInMesh.isApprox(
                    original.statistics.estimatedAxisInMesh) &&
                read.draft->statistics.heightMeters == original.statistics.heightMeters &&
                read.draft->statistics.maximumDiameterMeters ==
                    original.statistics.maximumDiameterMeters &&
                read.draft->statistics.minimumDiameterMeters ==
                    original.statistics.minimumDiameterMeters &&
                read.draft->statistics.axisConfidence == original.statistics.axisConfidence,
                "all model statistics round trip");
            expect(read.draft->stage == domain::PlanningStage::SprayBoundaryConfirmed &&
                read.draft->publishFrame == app::PublishFrame::PlanningLocalFrame,
                "workflow stage and publish frame round trip");
            expect(read.draft->parameters.section.relativePlaneTolerance == 2.0e-9 &&
                read.draft->parameters.section.relativeWeldTolerance == 9.0e-9 &&
                read.draft->parameters.section.minimumToleranceMeters == 2.0e-12 &&
                read.draft->parameters.region.minimumResampleCount == 128 &&
                read.draft->parameters.region.maximumResampleCount == 1536 &&
                read.draft->parameters.region.smoothingWindowFraction == 0.02 &&
                read.draft->parameters.region.minimumRadialProminenceFraction == 0.03 &&
                read.draft->parameters.region.axialSurfaceThreshold == 0.7 &&
                read.draft->parameters.region.minimumConfidence == 0.35 &&
                read.draft->parameters.region.transitionExtentFraction == 0.04,
                "all non-default algorithm parameters round trip");
            expect(read.draft->section.has_value() && read.draft->automaticRegions.has_value() &&
                read.draft->boundary.has_value(),
                "derived planning progress round trips");
            if(read.draft->section && read.draft->automaticRegions && read.draft->boundary) {
                expect(read.draft->section->pointsYz.size() == original.section->pointsYz.size() &&
                    read.draft->section->points3d.size() == original.section->points3d.size() &&
                    read.draft->section->cumulativeArcLength ==
                        original.section->cumulativeArcLength &&
                    read.draft->section->closed == original.section->closed &&
                    read.draft->section->toleranceMeters == original.section->toleranceMeters,
                    "section geometry and topology round trip");
                expect(read.draft->automaticRegions->segmentLabels ==
                    original.automaticRegions->segmentLabels &&
                    read.draft->automaticRegions->segmentConfidence ==
                        original.automaticRegions->segmentConfidence,
                    "automatic labels and confidence round trip");
                expect(read.draft->boundary->mode == original.boundary->mode &&
                    read.draft->boundary->polygonYz.size() == original.boundary->polygonYz.size() &&
                    read.draft->boundary->minimumY == original.boundary->minimumY &&
                    read.draft->boundary->maximumY == original.boundary->maximumY &&
                    read.draft->boundary->minimumZ == original.boundary->minimumZ &&
                    read.draft->boundary->maximumZ == original.boundary->maximumZ &&
                    read.draft->boundary->outerLineSlopeYPerZ ==
                        original.boundary->outerLineSlopeYPerZ &&
                    read.draft->boundary->outerLineInterceptY ==
                        original.boundary->outerLineInterceptY,
                    "all boundary parameters round trip");
            }
            expect(read.draft->editCommands.size() == 1 &&
                read.draft->appliedEditCommandCount == 1,
                "edit history and applied cursor round trip");
            if(!read.draft->editCommands.empty()) {
                expect(read.draft->editCommands.front().label == domain::RegionLabel::Transition &&
                    read.draft->editCommands.front().rectangle.minimum.isApprox(
                        original.editCommands.front().rectangle.minimum) &&
                    read.draft->editCommands.front().rectangle.maximum.isApprox(
                        original.editCommands.front().rectangle.maximum),
                    "edit command rectangle and label round trip");
            }
            expect(read.draft->boundaryMode == domain::BoundaryMode::ToothTopEnvelope,
                "boundary mode round trips");
            expect(read.draft->calibration.cylinderRows[0][0] &&
                std::abs(*read.draft->calibration.cylinderRows[0][0] - 1.0) < 1.0e-12 &&
                read.draft->calibration.circleRows[2][1] &&
                std::abs(*read.draft->calibration.circleRows[2][1] - 0.125) < 1.0e-12 &&
                !read.draft->calibration.showCylinder &&
                read.draft->calibration.activeMode == domain::CalibrationMode::Circle2d &&
                read.draft->calibration.circleFit.has_value() &&
                read.draft->calibration.frame.has_value() &&
                read.draft->calibration.frame->baseFromPlanning.translation().isApprox(
                    storedFrame.baseFromPlanning.translation()),
                "calibration points, fit, frame, mode and display toggles round trip privately");
            expect(read.draft->uiState.workflow ==
                    app::RotationBodyWorkflow::SectionRegionPlanning &&
                read.draft->uiState.mainViewMode ==
                    app::RotationBodyMainViewMode::Section &&
                read.draft->uiState.rightWorkflow ==
                    app::RotationBodyRightWorkflow::WorkpieceCalibration,
                "last workflow, main view and right-side page round trip privately");

            app::RotationBodyPlanningSession restored;
            const auto restoredResult = restored.restoreDraft(
                *read.draft,
                tetrahedron(),
                "fingerprint-a");
            expect(restoredResult.ok() &&
                restoredResult.value == app::DraftRestoreDisposition::Restored,
                "matching draft restores into a session");
            expect(restored.stage() == domain::PlanningStage::SprayBoundaryConfirmed &&
                restored.editHistory() != nullptr && restored.boundary().has_value(),
                "matching restore resumes the saved planning stage and edit history");
            expect(restored.calibrationWorkspace().topReference[2] &&
                std::abs(*restored.calibrationWorkspace().topReference[2] - 0.9) < 1.0e-12 &&
                restored.uiState().rightWorkflow ==
                    app::RotationBodyRightWorkflow::WorkpieceCalibration,
                "matching restore resumes private calibration and page progress");
            expect(!restored.hasPendingChanges(),
                "matching draft restore starts without pending changes");
        }

        app::RotationBodyPlanningDraft optionalConfidenceDraft = original;
        optionalConfidenceDraft.automaticRegions->segmentConfidence.clear();
        simulation_project::ProjectDocument optionalConfidenceDocument;
        expect(app::RotationBodyPlanningDraftStore::write(
            optionalConfidenceDocument,
            optionalConfidenceDraft),
            "draft with omitted optional confidence is written");
        const app::DraftReadResult optionalConfidenceRead =
            app::RotationBodyPlanningDraftStore::read(optionalConfidenceDocument);
        expect(optionalConfidenceRead.status == app::DraftReadStatus::Loaded &&
            optionalConfidenceRead.draft.has_value() &&
            optionalConfidenceRead.draft->automaticRegions.has_value() &&
            optionalConfidenceRead.draft->automaticRegions->segmentConfidence.empty(),
            "empty optional confidence survives persistence round trip");
        if(optionalConfidenceRead.draft) {
            app::RotationBodyPlanningSession optionalConfidenceRestored;
            const auto optionalConfidenceRestore = optionalConfidenceRestored.restoreDraft(
                *optionalConfidenceRead.draft,
                tetrahedron(),
                "fingerprint-a");
            expect(optionalConfidenceRestore.ok() &&
                optionalConfidenceRestored.stage() ==
                    domain::PlanningStage::SprayBoundaryConfirmed,
                "draft with empty optional confidence resumes its saved stage");
        }

        const app::DraftReadResult degraded = app::RotationBodyPlanningDraftStore::read(
            document,
            "fingerprint-b",
            original.meshFingerprint + 1);
        expect(degraded.status == app::DraftReadStatus::SourceChanged && degraded.draft.has_value(),
            "source fingerprint change produces a usable degraded draft");
        if(degraded.draft) {
            expect(degraded.draft->planningFromMesh.matrix().isApprox(original.planningFromMesh.matrix()) &&
                degraded.draft->baseFromPlanning.matrix().isApprox(original.baseFromPlanning.matrix()),
                "source change preserves model pose");
            expect(degraded.draft->sourceFingerprint == original.sourceFingerprint &&
                degraded.draft->meshFingerprint == original.meshFingerprint,
                "degraded draft retains the mismatching fingerprints for session detection");
            expect(degraded.draft->stage == domain::PlanningStage::ModelLoaded,
                "source change requires planning frame confirmation again");
            expect(!degraded.draft->section && !degraded.draft->automaticRegions &&
                degraded.draft->editCommands.empty() && !degraded.draft->boundary,
                "source change clears all derived progress");
        }

        simulation_project::ProjectDocument duplicateDocument = document;
        duplicateDocument.extensions.push_back(duplicateDocument.extensions.front());
        expect(app::RotationBodyPlanningDraftStore::read(duplicateDocument).status ==
            app::DraftReadStatus::InvalidPayload,
            "duplicate draft extension keys are rejected while reading");
        expect(app::RotationBodyPlanningDraftStore::write(duplicateDocument, original),
            "write normalizes duplicate current-version draft entries");
        const std::size_t normalizedCount = static_cast<std::size_t>(std::count_if(
            duplicateDocument.extensions.begin(),
            duplicateDocument.extensions.end(),
            [](const simulation_project::ProjectExtensionDesc& extension) {
                return extension.key == app::RotationBodyPlanningDraftStore::extensionKey;
            }));
        expect(normalizedCount == 1,
            "duplicate normalization leaves exactly one version 1 extension");
        expect(duplicateDocument.extensions.front().version == 1,
            "duplicate normalization writes the current extension version");

        simulation_project::ProjectDocument futureDocument = document;
        futureDocument.extensions.front().version = 2;
        futureDocument.extensions.front().serializedPayload = "future-payload";
        const simulation_project::ProjectDocument futureBefore = futureDocument;
        bool futureWriteThrew = false;
        try {
            app::RotationBodyPlanningDraftStore::write(futureDocument, original);
        } catch(const std::runtime_error&) {
            futureWriteThrew = true;
        }
        expect(futureWriteThrew &&
            futureDocument.extensions.front().version == futureBefore.extensions.front().version &&
            futureDocument.extensions.front().serializedPayload ==
                futureBefore.extensions.front().serializedPayload,
            "future draft payload is rejected without modifying the document");
        expect(app::RotationBodyPlanningDraftStore::read(futureDocument).status ==
            app::DraftReadStatus::UnsupportedVersion,
            "unsupported extension version is rejected");

        simulation_project::ProjectDocument badJsonDocument = document;
        badJsonDocument.extensions.front().serializedPayload = "{not-json";
        expect(app::RotationBodyPlanningDraftStore::read(badJsonDocument).status ==
            app::DraftReadStatus::InvalidPayload,
            "malformed JSON payload is rejected");

        simulation_project::ProjectDocument highPayloadDocument = document;
        highPayloadDocument.extensions.front().serializedPayload = "{\"schemaVersion\":2}";
        expect(app::RotationBodyPlanningDraftStore::read(highPayloadDocument).status ==
            app::DraftReadStatus::UnsupportedVersion,
            "future payload schema is rejected before reading legacy fields");
    }

    void testSessionDraftRestoreSourceChange()
    {
        app::RotationBodyPlanningSession originalSession = populatedSession();
        app::RotationBodyPlanningDraft draft = originalSession.makeDraft();
        const Eigen::Isometry3d expectedPlanningFromMesh = draft.planningFromMesh;
        const Eigen::Isometry3d expectedBaseFromPlanning = draft.baseFromPlanning;
        draft.statistics.vertexCount = 999;
        draft.statistics.heightMeters = 123.0;
        draft.automaticBaseline.translation() = Eigen::Vector3d(99.0, 98.0, 97.0);
        app::RotationBodyPlanningSession restored;
        const auto result = restored.restoreDraft(draft, tetrahedron(0.25), "fingerprint-b");
        expect(result.ok() && result.value == app::DraftRestoreDisposition::SourceChanged,
            "session detects a source change while restoring");
        expect(restored.hasModel() && restored.stage() == domain::PlanningStage::ModelLoaded,
            "source-changed restore keeps the model but requires frame confirmation");
        expect(restored.planningFromMesh().matrix().isApprox(expectedPlanningFromMesh.matrix()) &&
            restored.baseFromPlanning().matrix().isApprox(expectedBaseFromPlanning.matrix()) &&
            restored.publishFrame() == app::PublishFrame::PlanningLocalFrame,
            "source-changed restore preserves user T_PM, T_BP and publish frame");
        expect(restored.statistics().vertexCount == 4 &&
            restored.statistics().heightMeters != 123.0 &&
            !restored.automaticBaseline().matrix().isApprox(draft.automaticBaseline.matrix()),
            "simulation-block source change refreshes statistics and automatic baseline");
        expect(!restored.section() && !restored.automaticRegions() && !restored.boundary(),
            "source-changed restore drops derived state");
        expect(restored.hasPendingChanges(),
            "source-changed draft restore marks degraded progress pending");

        const domain::TriangleMesh completeOriginal = revolvedProfile(
            { 0.0, 0.5, 1.0, 1.5, 2.0 },
            { 0.35, 0.42, 0.48, 0.48, 0.48 });
        const auto originalAlignment = domain::RotationBodyAlignmentSolver::solve(completeOriginal);
        expect(originalAlignment.ok(), "complete-part source-change fixture aligns");
        if(originalAlignment) {
            app::RotationBodyPlanningSession completeSession;
            expect(completeSession.loadModel(
                "complete",
                "complete.stl",
                "complete-a",
                completeOriginal,
                originalAlignment.value,
                domain::PlanningObjectType::CompletePart).ok(),
                "complete-part session loads");
            app::RotationBodyPlanningDraft completeDraft = completeSession.makeDraft();
            completeDraft.statistics.maximumDiameterMeters = 999.0;
            const domain::TriangleMesh completeCurrent = revolvedProfile(
                { 0.0, 0.8, 1.6, 2.4, 3.2 },
                { 0.55, 0.62, 0.68, 0.68, 0.68 });
            const auto expectedCurrentAlignment =
                domain::RotationBodyAlignmentSolver::solve(completeCurrent);
            app::RotationBodyPlanningSession completeRestored;
            const auto completeResult = completeRestored.restoreDraft(
                completeDraft,
                completeCurrent,
                "complete-b");
            expect(completeResult.ok() && expectedCurrentAlignment.ok(),
                "complete-part changed source is re-aligned");
            if(completeResult && expectedCurrentAlignment) {
                expect(std::abs(
                    completeRestored.statistics().maximumDiameterMeters -
                    expectedCurrentAlignment.value.statistics.maximumDiameterMeters) < 1.0e-9,
                    "complete-part metadata comes from the current mesh");
            }
        }
    }

    void testMeshAdapter()
    {
        assetcore::ModelDesc indexedModel;
        assetcore::SubMeshDesc indexed;
        indexed.geometry.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f }
        };
        indexed.geometry.indices = { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 };
        indexedModel.addSubMesh(indexed);
        const auto indexedResult = app::RotationBodyMeshAdapter::build(indexedModel);
        expect(indexedResult.ok() && indexedResult.value.vertexCount() == 4 &&
            indexedResult.value.triangleCount() == 4,
            "indexed submesh converts to a planning mesh");

        assetcore::ModelDesc localTransformModel;
        localTransformModel.addSubMesh(indexed);
        glm::mat4 localTransform(1.0f);
        localTransform[0] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
        localTransform[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
        localTransform[2] = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
        localTransform[3] = glm::vec4(2.0f, 3.0f, 4.0f, 1.0f);
        localTransformModel.get_local() = localTransform;
        const auto localTransformResult = app::RotationBodyMeshAdapter::build(localTransformModel);
        expect(localTransformResult.ok(), "model local rotate/translate transform is accepted");
        if(localTransformResult) {
            const auto containsPosition = [&](const Eigen::Vector3d& expected) {
                return std::any_of(
                    localTransformResult.value.positions.begin(),
                    localTransformResult.value.positions.end(),
                    [&](const Eigen::Vector3d& actual) {
                        return actual.isApprox(expected, 1.0e-9);
                    });
            };
            expect(containsPosition(Eigen::Vector3d(2.0, 3.0, 4.0)) &&
                containsPosition(Eigen::Vector3d(2.0, 4.0, 4.0)) &&
                containsPosition(Eigen::Vector3d(1.0, 3.0, 4.0)),
                "adapter positions match runtime objectLocal * rawPosition semantics");
        }

        assetcore::ModelDesc multiSubmeshModel;
        assetcore::SubMeshDesc firstSubmesh;
        firstSubmesh.geometry.positions = {
            { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }
        };
        firstSubmesh.geometry.indices = { 0, 1, 2 };
        assetcore::SubMeshDesc secondSubmesh;
        secondSubmesh.geometry.positions = {
            { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 1.0f }
        };
        secondSubmesh.geometry.indices = { 0, 1, 2 };
        multiSubmeshModel.addSubMesh(firstSubmesh);
        multiSubmeshModel.addSubMesh(secondSubmesh);
        const auto multiSubmeshResult = app::RotationBodyMeshAdapter::build(multiSubmeshModel);
        expect(multiSubmeshResult.ok() && multiSubmeshResult.value.vertexCount() == 6 &&
            multiSubmeshResult.value.triangleCount() == 2 &&
            multiSubmeshResult.value.triangles[1].minCoeff() >= 3,
            "multiple indexed submeshes preserve independent global index offsets");

        assetcore::ModelDesc unusedOutlierModel;
        assetcore::SubMeshDesc unusedOutlier;
        unusedOutlier.geometry.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 1000000.0f, 1000000.0f, 1000000.0f }
        };
        unusedOutlier.geometry.indices = { 0, 1, 2 };
        unusedOutlierModel.addSubMesh(unusedOutlier);
        const auto unusedOutlierResult = app::RotationBodyMeshAdapter::build(unusedOutlierModel);
        expect(unusedOutlierResult.ok() && unusedOutlierResult.value.vertexCount() == 3,
            "indexed conversion omits vertices not referenced by any triangle");
        if(unusedOutlierResult) {
            const auto bounds = unusedOutlierResult.value.bounds();
            expect(bounds.ok() && bounds.value.max().maxCoeff() <= 1.0,
                "unused indexed outlier cannot pollute planning bounds");
        }

        assetcore::ModelDesc nonIndexedModel;
        assetcore::SubMeshDesc nonIndexed;
        nonIndexed.geometry.positions = {
            { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f },
            { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }
        };
        nonIndexedModel.addSubMesh(nonIndexed);
        const auto nonIndexedResult = app::RotationBodyMeshAdapter::build(nonIndexedModel);
        expect(nonIndexedResult.ok() && nonIndexedResult.value.triangleCount() == 4,
            "non-indexed triangle triplets convert to a planning mesh");

        assetcore::ModelDesc invalidModel;
        assetcore::SubMeshDesc invalid;
        invalid.geometry.positions = indexed.geometry.positions;
        invalid.geometry.indices = { 0, 1, 99 };
        invalidModel.addSubMesh(invalid);
        expect(!app::RotationBodyMeshAdapter::build(invalidModel),
            "out-of-range indices are rejected");

        assetcore::ModelDesc nonFinitePositionModel;
        assetcore::SubMeshDesc nonFinitePosition;
        nonFinitePosition.geometry.positions = firstSubmesh.geometry.positions;
        nonFinitePosition.geometry.positions.front().x() =
            std::numeric_limits<float>::infinity();
        nonFinitePosition.geometry.indices = { 0, 1, 2 };
        nonFinitePositionModel.addSubMesh(nonFinitePosition);
        expect(!app::RotationBodyMeshAdapter::build(nonFinitePositionModel),
            "non-finite raw positions are rejected before conversion");

        assetcore::ModelDesc nonFiniteLocalModel;
        nonFiniteLocalModel.addSubMesh(firstSubmesh);
        nonFiniteLocalModel.get_local()[0][0] = std::numeric_limits<float>::infinity();
        expect(!app::RotationBodyMeshAdapter::build(nonFiniteLocalModel),
            "non-finite model local transform is rejected");

        assetcore::ModelDesc invalidHomogeneousModel;
        invalidHomogeneousModel.addSubMesh(firstSubmesh);
        invalidHomogeneousModel.get_local()[0][3] = 1.0f;
        expect(!app::RotationBodyMeshAdapter::build(invalidHomogeneousModel),
            "projective local transform with invalid homogeneous w is rejected");

        assetcore::ModelDesc degenerateModel;
        assetcore::SubMeshDesc degenerate;
        degenerate.geometry.positions = {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 2.0f, 0.0f, 0.0f }
        };
        degenerateModel.addSubMesh(degenerate);
        expect(!app::RotationBodyMeshAdapter::build(degenerateModel),
            "zero-area triangles are rejected");
    }

    void testPlanningObjectReplacement()
    {
        simulation_project::ProjectDocument document;
        simulation_project::SceneObjectDesc previous;
        previous.id = "previous";
        previous.name = "Previous";
        simulation_project::SceneObjectDesc current;
        current.id = "current";
        current.name = "Current";
        simulation_project::SceneObjectDesc unrelated;
        unrelated.id = "unrelated";
        unrelated.name = "Unrelated";
        document.objects = { previous, current, unrelated };
        app::RotationBodyPlanningDraft draft = populatedSession().makeDraft();
        draft.objectId = previous.id;
        app::RotationBodyPlanningDraftStore::write(document, draft);

        simulation_project::ProjectDocumentService service(document);
        const app::PlanningObjectReplacementResult replaced =
            app::RotationBodyPlanningDocumentOperations::replacePlanningObject(
                service,
                previous.id,
                current.id);
        expect(replaced.success && replaced.changed,
            "planning document operation replaces the previous workpiece");
        expect(service.findSceneObject(previous.id) == nullptr &&
            service.findSceneObject(current.id) != nullptr &&
            service.findSceneObject(unrelated.id) != nullptr,
            "replacement removes only the previous planning workpiece");
        expect(app::RotationBodyPlanningDraftStore::read(document).status ==
            app::DraftReadStatus::NotFound,
            "replacement removes the stale private draft until the new work is saved");

        simulation_project::ProjectDocument missingCurrentDocument;
        missingCurrentDocument.objects = { previous };
        simulation_project::ProjectDocumentService missingCurrentService(missingCurrentDocument);
        const app::PlanningObjectReplacementResult missingCurrent =
            app::RotationBodyPlanningDocumentOperations::replacePlanningObject(
                missingCurrentService,
                previous.id,
                "missing");
        expect(!missingCurrent.success &&
            missingCurrentService.findSceneObject(previous.id) != nullptr,
            "failed replacement leaves the previous workpiece untouched");
    }

    simulation_project::SceneObjectDesc projectObject(const std::string& objectId)
    {
        simulation_project::SceneObjectDesc object;
        object.id = objectId;
        object.name = objectId;
        object.sourcePath = "models/" + objectId + ".stl";
        return object;
    }

    void testControllerActivationIsolationAndRollback()
    {
        {
            ControllerFixture fixture;
            fixture.projectSession.document().objects.push_back(projectObject("project_a"));
            expect(fixture.controller.activate().success,
                "project A planning controller activates");
            domain::TriangleMesh mesh = tetrahedron();
            expect(fixture.controller.session().loadModel(
                "project_a",
                "project_a.stl",
                "project-a-fingerprint",
                mesh,
                identityAlignment(mesh),
                domain::PlanningObjectType::CompletePart).ok(),
                "project A session loads");
            expect(fixture.controller.saveProgressAndExit().success &&
                !fixture.controller.isActive(),
                "project A saves and exits");

            simulation_project::ProjectDocument projectB;
            projectB.objects.push_back(projectObject("project_b"));
            fixture.projectSession.setDocument(
                std::move(projectB),
                std::filesystem::path(),
                false,
                true);
            const app::RotationBodyControllerResult projectBActivation =
                fixture.controller.activate();
            expect(projectBActivation.success && fixture.controller.isActive(),
                "project B planning controller activates without a draft");
            expect(!fixture.controller.session().hasModel() &&
                fixture.controller.session().stage() == domain::PlanningStage::NoModel &&
                !fixture.controller.hasPendingChanges() &&
                fixture.viewport.focusedObjectId.isEmpty() &&
                fixture.viewport.focusObjectCalls == 0,
                "project B cannot inherit project A state or focus without a model");
            expect(fixture.controller.deactivate().success,
                "project B empty planning session exits");
        }

        {
            ControllerFixture fixture;
            domain::TriangleMesh staleMesh = tetrahedron();
            expect(fixture.controller.session().loadModel(
                "stale_session",
                "stale.stl",
                "stale-fingerprint",
                staleMesh,
                identityAlignment(staleMesh),
                domain::PlanningObjectType::CompletePart).ok(),
                "activation rollback fixture starts with stale session data");
            simulation_project::ProjectExtensionDesc invalidDraft;
            invalidDraft.key = app::RotationBodyPlanningDraftStore::extensionKey;
            invalidDraft.version = app::RotationBodyPlanningDraftStore::currentVersion;
            invalidDraft.serializedPayload = "{invalid-json";
            fixture.projectSession.document().extensions.push_back(std::move(invalidDraft));

            const QString ownerId =
                QStringLiteral("spray.rotation_body_trajectory_planning");
            robot_qt_viewer::ViewportLineOverlay staleOverlay;
            staleOverlay.ownerId = ownerId;
            staleOverlay.overlayId = QStringLiteral("section");
            fixture.viewport.upsertLineOverlay(staleOverlay);
            fixture.viewport.focusObjectFrameObject(QStringLiteral("stale_session"));

            const app::RotationBodyControllerResult activation =
                fixture.controller.activate();
            expect(!activation.success && !fixture.controller.isActive(),
                "invalid current-project draft fails activation and leaves it inactive");
            expect(!fixture.controller.session().hasModel() &&
                fixture.controller.session().stage() == domain::PlanningStage::NoModel &&
                !fixture.controller.hasPendingChanges(),
                "failed activation clears all stale planning session data");
            expect(fixture.viewport.overlay(ownerId, QStringLiteral("section")) == nullptr &&
                fixture.viewport.focusedObjectId.isEmpty() &&
                fixture.viewport.clearObjectFocusCalls > 0,
                "failed activation clears stale focus and overlay presentation");

            fixture.projectSession.document().objects.push_back(projectObject("after_failure"));
            fixture.projectSession.setDirty(true);
            expect(fixture.controller.discardAndExit().success &&
                fixture.projectSession.document().objects.size() == 1 &&
                fixture.projectSession.document().objects.front().id == "after_failure" &&
                fixture.projectSession.isDirty(),
                "failed activation retains no entry snapshot that a later discard could restore");
        }
    }

    void testControllerRecoversFromOrphanedDraft()
    {
        ControllerFixture fixture;
        app::RotationBodyPlanningDraft draft = populatedSession().makeDraft();
        draft.objectId = "deleted_workpiece";
        draft.sourcePath = "models/deleted_workpiece.stl";
        fixture.projectSession.document().objects.push_back(
            projectObject(draft.objectId));
        expect(app::RotationBodyPlanningDraftStore::write(
            fixture.projectSession.document(),
            draft),
            "orphan recovery fixture stores a valid private draft");

        simulation_project::ProjectDocumentService service(
            fixture.projectSession.document());
        simulation_project::ProjectReferenceCleanupReport cleanup;
        std::string removalError;
        expect(service.removeObject(draft.objectId, cleanup, &removalError),
            "Scene Explorer deletion removes the draft's public workpiece");
        expect(app::RotationBodyPlanningDraftStore::read(
            fixture.projectSession.document()).status == app::DraftReadStatus::Loaded,
            "public workpiece deletion leaves the private draft orphaned");

        const app::RotationBodyControllerResult firstActivation =
            fixture.controller.activate();
        expect(firstActivation.success && fixture.controller.isActive() &&
            !fixture.controller.session().hasModel() &&
            fixture.controller.session().stage() == domain::PlanningStage::NoModel,
            "an orphaned private draft is ignored and does not block module activation");
        expect(app::RotationBodyPlanningDraftStore::read(
            fixture.projectSession.document()).status == app::DraftReadStatus::NotFound &&
            fixture.projectSession.isDirty(),
            "orphan recovery removes the private extension and marks the project dirty");

        expect(fixture.controller.deactivate().success &&
            app::RotationBodyPlanningDraftStore::read(
                fixture.projectSession.document()).status == app::DraftReadStatus::NotFound,
            "discarding the empty planning session does not resurrect the orphaned draft");

        const app::RotationBodyControllerResult secondActivation =
            fixture.controller.activate();
        expect(secondActivation.success && fixture.controller.isActive() &&
            !fixture.controller.session().hasModel(),
            "later module activations remain usable after orphan recovery");
        expect(fixture.controller.deactivate().success,
            "orphan recovery fixture exits cleanly after reactivation");
    }

    void testControllerAsyncPlanningOperations()
    {
        ControllerFixture fixture;
        fixture.projectSession.document().objects.push_back(
            projectObject("async_workpiece"));
        expect(fixture.controller.activate().success,
            "async planning fixture activates");
        domain::TriangleMesh mesh = box(
            { -1.0, -2.0, 0.0 },
            { 1.0, 2.0, 3.0 });
        expect(fixture.controller.session().loadModel(
            "async_workpiece",
            "async_workpiece.stl",
            "async-workpiece-fingerprint",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::CompletePart).ok() &&
            fixture.controller.confirmFrame().success,
            "async planning fixture loads and confirms a simple model");

        int completionCount = 0;
        app::RotationBodyControllerResult lastCompletion;
        app::RotationBodyPlanningLeftPanel leftPanel;
        app::RotationBodyPlanningRightPanel rightPanel;
        app::RotationBodyPlanningWorkflowCoordinator coordinator(
            fixture.controller,
            leftPanel,
            rightPanel);
        auto* importButton = leftPanel.findChild<QToolButton*>(
            QStringLiteral("rotationBodyModel.load"));
        auto* discardButton = leftPanel.findChild<QPushButton*>(
            QStringLiteral("rotationBodyCommand.discard"));
        auto* extractButton = leftPanel.findChild<QPushButton*>(
            QStringLiteral("rotationBodySection.extract"));
        QObject::connect(
            &fixture.controller,
            &app::RotationBodyPlanningController::asyncOperationFinished,
            [&completionCount, &lastCompletion](
                const app::RotationBodyControllerResult& result) {
                ++completionCount;
                lastCompletion = result;
            });

        const app::RotationBodyControllerResult sectionStarted =
            fixture.controller.startExtractSection();
        expect(sectionStarted.success && fixture.controller.isBusy() &&
            fixture.controller.viewModel().isBusy &&
            !fixture.controller.viewModel().canCreateSection &&
            !fixture.controller.viewModel().canSaveProgress &&
            importButton != nullptr && !importButton->isEnabled() &&
            extractButton != nullptr && !extractButton->isEnabled() &&
            discardButton != nullptr && discardButton->isEnabled(),
            "section worker enters busy state and disables conflicting commands");
        const app::RotationBodyControllerResult duplicateSection =
            fixture.controller.startExtractSection();
        expect(!duplicateSection.success && completionCount == 0,
            "a second planning operation is rejected while the section worker runs");
        expect(processEventsUntil([&completionCount]() {
            return completionCount == 1;
        }) && lastCompletion.success && !fixture.controller.isBusy() &&
            fixture.controller.session().stage() == domain::PlanningStage::SectionReady &&
            fixture.controller.session().section().has_value(),
            "section worker commits its result on the controller thread");

        expect(fixture.controller.session().acceptSection(
            fixture.controller.session().generation(),
            toothRegionSection()),
            "async recognition fixture installs a representative tooth contour");
        const app::RotationBodyControllerResult recognitionStarted =
            fixture.controller.startRecognizeRegions();
        expect(recognitionStarted.success && fixture.controller.isBusy(),
            "automatic recognition starts as a background operation");
        expect(processEventsUntil([&completionCount]() {
            return completionCount == 2;
        }) && lastCompletion.success && !fixture.controller.isBusy() &&
            fixture.controller.session().stage() == domain::PlanningStage::RegionsReady &&
            fixture.controller.session().automaticRegions().has_value(),
            "automatic recognition commits matching labels after returning to the GUI thread");
        expect(fixture.controller.deactivate().success,
            "async planning fixture exits cleanly");
    }

    void testControllerAsyncImportAndCancellation()
    {
        const std::filesystem::path sourceRoot =
            std::filesystem::u8path(ROTATION_BODY_TEST_SOURCE_ROOT);
        const std::filesystem::path modelPath =
            sourceRoot / "data/Spray420/ATPPZ350/meshes/Link2.STL";
        app::RotationBodyImportOptions options;
        options.objectType = domain::PlanningObjectType::SimulationBlock;
        options.originalRotationAxis = domain::SignedAxis::PositiveZ;
        options.toothOutwardAxis = domain::SignedAxis::PositiveY;
        options.motherMaximumDiameterMeters = 1.0;

        {
            ControllerFixture fixture;
            fixture.projectSession.setPath(
                sourceRoot / "RotationBodyAsyncImportTest.sys.json");
            int completionCount = 0;
            app::RotationBodyControllerResult completion;
            QObject::connect(
                &fixture.controller,
                &app::RotationBodyPlanningController::asyncOperationFinished,
                [&completionCount, &completion](
                    const app::RotationBodyControllerResult& result) {
                    ++completionCount;
                    completion = result;
                });
            const app::RotationBodyControllerResult started =
                fixture.controller.startImportModel(modelPath, options);
            expect(started.success && fixture.controller.isActive() &&
                fixture.controller.isBusy() &&
                fixture.projectSession.document().objects.size() == 1,
                "real STL import registers its public object before background loading");
            expect(processEventsUntil([&completionCount]() {
                return completionCount == 1;
            }, 30000) && completion.success &&
                fixture.controller.session().hasModel() &&
                !fixture.controller.isBusy() &&
                fixture.viewport.loadProjectCalls == 1,
                "real STL loading and alignment complete asynchronously before GUI commit");
            expect(fixture.controller.deactivate().success,
                "successful async import fixture exits cleanly");
        }

        {
            ControllerFixture fixture;
            fixture.projectSession.setPath(
                sourceRoot / "RotationBodyAsyncImportFailureTest.sys.json");
            const bool initialDirty = fixture.projectSession.isDirty();
            int completionCount = 0;
            app::RotationBodyControllerResult completion;
            QObject::connect(
                &fixture.controller,
                &app::RotationBodyPlanningController::asyncOperationFinished,
                [&completionCount, &completion](
                    const app::RotationBodyControllerResult& result) {
                    ++completionCount;
                    completion = result;
                });
            const app::RotationBodyControllerResult started =
                fixture.controller.startImportModel(
                    sourceRoot / "data/Spray420/missing-async-workpiece.stl",
                    options);
            expect(started.success && fixture.projectSession.document().objects.size() == 1,
                "missing async source still exercises the temporary public-object transaction");
            expect(processEventsUntil([&completionCount]() {
                return completionCount == 1;
            }) && !completion.success &&
                fixture.projectSession.document().objects.empty() &&
                fixture.projectSession.isDirty() == initialDirty &&
                !fixture.controller.session().hasModel() &&
                !fixture.controller.isBusy(),
                "failed background model load restores the document, dirty state and session");
            expect(fixture.controller.deactivate().success,
                "failed async import fixture exits cleanly");
        }

        {
            ControllerFixture fixture;
            fixture.projectSession.document().objects.push_back(
                projectObject("cancelled_workpiece"));
            expect(fixture.controller.activate().success,
                "async cancellation fixture activates");
            domain::TriangleMesh mesh = revolvedProfile(
                { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 },
                { 0.8, 1.0, 0.85, 1.1, 0.9, 0.8 },
                4096);
            expect(fixture.controller.session().loadModel(
                "cancelled_workpiece",
                "cancelled_workpiece.stl",
                "cancelled-workpiece-fingerprint",
                mesh,
                identityAlignment(mesh),
                domain::PlanningObjectType::CompletePart).ok() &&
                fixture.controller.confirmFrame().success,
                "async cancellation fixture prepares a non-trivial mesh");
            int completionCount = 0;
            QObject::connect(
                &fixture.controller,
                &app::RotationBodyPlanningController::asyncOperationFinished,
                [&completionCount](const app::RotationBodyControllerResult&) {
                    ++completionCount;
                });
            expect(fixture.controller.startExtractSection().success &&
                fixture.controller.isBusy(),
                "cancellation fixture starts a background section");
            expect(fixture.controller.discardAndExit().success &&
                !fixture.controller.isActive() &&
                !fixture.controller.isBusy(),
                "discard invalidates the active worker token and restores the entry snapshot");
            QApplication::processEvents(QEventLoop::AllEvents, 100);
            expect(completionCount == 0 &&
                !fixture.controller.session().hasModel() &&
                fixture.projectSession.document().objects.size() == 1 &&
                fixture.projectSession.document().objects.front().id == "cancelled_workpiece",
                "a late worker result cannot mutate the inactive session or restored document");
        }
    }

    void testControllerDestructionDrainsPrivateWorkerPool()
    {
        simulation_project::ProjectSession projectSession;
        robot_qt_viewer::RobotQtViewerEventHub eventHub;
        robot_qt_viewer::RobotQtViewerDocumentController documentController(
            projectSession,
            eventHub);
        robot_qt_viewer::RobotQtViewerSelectionModel selectionModel(eventHub);
        robot_qt_viewer::RobotQtViewerViewportPreviewState previewState(eventHub);
        robot_qt_viewer::RobotQtViewerOperationStatusStore operationStatusStore(eventHub);
        robot_qt_viewer::RobotQtViewerDocumentContext context(
            projectSession,
            documentController,
            selectionModel,
            previewState,
            eventHub,
            operationStatusStore);
        FakeRotationBodyViewportServices viewport;
        context.setViewportServices(&viewport);
        projectSession.document().objects.push_back(
            projectObject("destruction_workpiece"));

        int completionCount = 0;
        {
            auto controller = std::make_unique<app::RotationBodyPlanningController>(context);
            expect(controller->activate().success,
                "controller destruction fixture activates");
            domain::TriangleMesh mesh = revolvedProfile(
                { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 },
                { 0.8, 1.0, 0.85, 1.1, 0.9, 0.8 },
                4096);
            expect(controller->session().loadModel(
                "destruction_workpiece",
                "destruction_workpiece.stl",
                "destruction-workpiece-fingerprint",
                mesh,
                identityAlignment(mesh),
                domain::PlanningObjectType::CompletePart).ok() &&
                controller->confirmFrame().success,
                "controller destruction fixture prepares a background section input");
            QObject::connect(
                controller.get(),
                &app::RotationBodyPlanningController::asyncOperationFinished,
                [&completionCount](const app::RotationBodyControllerResult&) {
                    ++completionCount;
                });
            expect(controller->startExtractSection().success && controller->isBusy(),
                "controller destruction fixture destroys the controller while work is pending");
        }

        QApplication::processEvents(QEventLoop::AllEvents, 100);
        expect(completionCount == 0 &&
            projectSession.document().objects.size() == 1 &&
            projectSession.document().objects.front().id == "destruction_workpiece",
            "controller destruction drains its worker and removes late callbacks without changing the document");
    }

    void testSaveCommitSurvivesViewportReloadFailure()
    {
        ControllerFixture fixture;
        fixture.projectSession.document().objects.push_back(projectObject("saved_workpiece"));
        expect(fixture.controller.activate().success,
            "save warning fixture activates");
        domain::TriangleMesh mesh = tetrahedron();
        expect(fixture.controller.session().loadModel(
            "saved_workpiece",
            "saved_workpiece.stl",
            "saved-fingerprint",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::CompletePart).ok(),
            "save warning fixture session loads");
        Eigen::Isometry3d planningFromMesh = Eigen::Isometry3d::Identity();
        planningFromMesh.translation() = Eigen::Vector3d(0.12, -0.23, 0.34);
        planningFromMesh.linear() =
            Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        expect(fixture.controller.updatePlanningTransform(planningFromMesh).success,
            "save warning fixture applies a non-identity planning transform");

        fixture.viewport.nextLoadSucceeds = false;
        const app::RotationBodyControllerResult saved =
            fixture.controller.saveProgressAndExit();
        expect(saved.success && !saved.message.isEmpty() &&
            saved.notice == app::RotationBodyControllerNotice::SavedViewportRefreshFailed &&
            !fixture.controller.isActive() &&
            !fixture.controller.hasPendingChanges(),
            "viewport reload failure is a successful committed save with a warning");
        expect(fixture.projectSession.isDirty() &&
            fixture.viewport.loadProjectCalls == 1 &&
            fixture.viewport.focusedObjectId.isEmpty(),
            "committed save remains dirty and clears module presentation after reload failure");
        const simulation_project::TransformDesc committed =
            fixture.projectSession.document().objects.front().transform;
        expect(std::abs(committed.x - 0.12) < 1.0e-12 &&
            std::abs(committed.y + 0.23) < 1.0e-12 &&
            std::abs(committed.z - 0.34) < 1.0e-12 &&
            std::abs(committed.yaw - 0.25) < 1.0e-12,
            "save commits the published non-identity model transform before reloading");
        expect(app::RotationBodyPlanningDraftStore::read(
            fixture.projectSession.document()).status == app::DraftReadStatus::Loaded,
            "save commits private planning progress before viewport reload");

        expect(fixture.controller.discardAndExit().success &&
            fixture.viewport.loadProjectCalls == 1 &&
            fixture.projectSession.document().objects.front().transform.x == committed.x &&
            app::RotationBodyPlanningDraftStore::read(
                fixture.projectSession.document()).status == app::DraftReadStatus::Loaded,
            "post-warning discard cannot roll back the already committed project");
    }

    void testCoordinatorReportsCommittedSaveWarning()
    {
        ControllerFixture fixture;
        fixture.projectSession.document().objects.push_back(projectObject("warning_workpiece"));
        expect(fixture.controller.activate().success,
            "coordinator save warning fixture activates");
        domain::TriangleMesh mesh = tetrahedron();
        expect(fixture.controller.session().loadModel(
            "warning_workpiece",
            "warning_workpiece.stl",
            "warning-fingerprint",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::CompletePart).ok(),
            "coordinator save warning fixture session loads");

        app::RotationBodyPlanningLeftPanel leftPanel;
        app::RotationBodyPlanningRightPanel rightPanel;
        app::RotationBodyPlanningWorkflowCoordinator coordinator(
            fixture.controller,
            leftPanel,
            rightPanel);
        coordinator.setLanguageCode(QStringLiteral("zh-CN"));
        QString reportedStatus;
        bool exitedAsSaved = false;
        QObject::connect(
            &coordinator,
            &app::RotationBodyPlanningWorkflowCoordinator::statusMessageRequested,
            [&reportedStatus](const QString& message, int) {
                reportedStatus = message;
            });
        QObject::connect(
            &coordinator,
            &app::RotationBodyPlanningWorkflowCoordinator::exitCompleted,
            [&exitedAsSaved](bool saved) {
                exitedAsSaved = saved;
            });

        fixture.viewport.nextLoadSucceeds = false;
        leftPanel.saveProgressAndExitRequested();
        const QString warning = app::RotationBodyPlanningTranslations::text(
            QStringLiteral("zh-CN"),
            "status.saved_view_refresh_failed");
        const QString normalSaved = app::RotationBodyPlanningTranslations::text(
            QStringLiteral("zh-CN"),
            "status.saved");
        expect(reportedStatus == warning && reportedStatus != normalSaved &&
            exitedAsSaved && !fixture.controller.isActive(),
            "coordinator preserves the committed-save viewport warning status");
    }

    domain::SectionContour uiSection()
    {
        domain::SectionContour contour;
        contour.pointsYz = {
            { 0.10, 0.00 },
            { 0.30, 0.02 },
            { 0.34, 0.16 },
            { 0.22, 0.27 },
            { 0.29, 0.39 },
            { 0.12, 0.46 }
        };
        contour.points3d.reserve(contour.pointsYz.size());
        contour.cumulativeArcLength.reserve(contour.pointsYz.size());
        double arcLength = 0.0;
        for(std::size_t index = 0; index < contour.pointsYz.size(); ++index) {
            contour.points3d.push_back({
                0.0,
                contour.pointsYz[index].x(),
                contour.pointsYz[index].y()
            });
            if(index > 0) {
                arcLength += (contour.pointsYz[index] - contour.pointsYz[index - 1]).norm();
            }
            contour.cumulativeArcLength.push_back(arcLength);
        }
        contour.toleranceMeters = 1.0e-8;
        return contour;
    }

    domain::RegionAssignment uiRegions()
    {
        domain::RegionAssignment regions;
        regions.segmentLabels = {
            domain::RegionLabel::Unclassified,
            domain::RegionLabel::ToothTop,
            domain::RegionLabel::ToothWall,
            domain::RegionLabel::ToothBottom,
            domain::RegionLabel::Transition
        };
        regions.segmentConfidence.assign(regions.segmentLabels.size(), 0.9);
        return regions;
    }

    domain::SprayBoundary uiBoundary()
    {
        domain::SprayBoundary boundary;
        boundary.mode = domain::BoundaryMode::MaximumToothTopY;
        boundary.polygonYz = {
            { 0.08, -0.01 },
            { 0.36, -0.01 },
            { 0.36, 0.47 },
            { 0.08, 0.47 }
        };
        boundary.minimumY = 0.08;
        boundary.maximumY = 0.36;
        boundary.minimumZ = -0.01;
        boundary.maximumZ = 0.47;
        boundary.outerLineInterceptY = 0.36;
        return boundary;
    }

    void testViewportOverlays()
    {
        simulation_project::ProjectSession projectSession;
        robot_qt_viewer::RobotQtViewerEventHub eventHub;
        robot_qt_viewer::RobotQtViewerDocumentController documentController(
            projectSession,
            eventHub);
        robot_qt_viewer::RobotQtViewerSelectionModel selectionModel(eventHub);
        robot_qt_viewer::RobotQtViewerViewportPreviewState previewState(eventHub);
        robot_qt_viewer::RobotQtViewerOperationStatusStore operationStatusStore(eventHub);
        robot_qt_viewer::RobotQtViewerDocumentContext context(
            projectSession,
            documentController,
            selectionModel,
            previewState,
            eventHub,
            operationStatusStore);
        FakeRotationBodyViewportServices viewport;
        context.setViewportServices(&viewport);
        app::RotationBodyPlanningController controller(context);
        expect(controller.activate().success, "overlay controller activates");

        domain::TriangleMesh mesh = tetrahedron();
        expect(controller.session().loadModel(
            "overlay_workpiece",
            "overlay.stl",
            "overlay-fingerprint",
            mesh,
            identityAlignment(mesh),
            domain::PlanningObjectType::CompletePart).ok(),
            "overlay fixture model loads");
        Eigen::Isometry3d planningFromMesh = Eigen::Isometry3d::Identity();
        planningFromMesh.translation() = Eigen::Vector3d(0.11, -0.22, 0.33);
        planningFromMesh.linear() =
            Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        expect(controller.session().setPlanningFromMesh(planningFromMesh).ok(),
            "overlay fixture applies a non-identity planning transform");
        expect(controller.session().confirmFrame().ok(),
            "overlay fixture frame confirms");
        expect(controller.session().acceptSection(
            controller.session().generation(),
            uiSection()),
            "overlay fixture section is accepted");

        robot_qt_viewer::ViewportLineOverlay unrelated;
        unrelated.ownerId = QStringLiteral("another.module");
        unrelated.overlayId = QStringLiteral("keep");
        expect(viewport.upsertLineOverlay(unrelated),
            "unrelated overlay fixture is inserted");

        const QString ownerId =
            QStringLiteral("spray.rotation_body_trajectory_planning");
        expect(controller.rebuildViewportOverlay().success,
            "unclassified section overlay rebuild succeeds");
        const robot_qt_viewer::ViewportLineOverlay* section =
            viewport.overlay(ownerId, QStringLiteral("section"));
        expect(section != nullptr && section->segments.size() == uiSection().segmentCount(),
            "section overlay contains one continuous segment per contour segment");
        if(section != nullptr) {
            const QColor unclassified =
                app::SectionView::semanticColor(domain::RegionLabel::Unclassified);
            const bool allUnclassified = std::all_of(
                section->segments.begin(),
                section->segments.end(),
                [&](const robot_qt_viewer::ViewportLineSegment& segment) {
                    return std::abs(segment.color.r * 255.0 - unclassified.red()) < 1.0e-9 &&
                        std::abs(segment.color.g * 255.0 - unclassified.green()) < 1.0e-9 &&
                        std::abs(segment.color.b * 255.0 - unclassified.blue()) < 1.0e-9;
                });
            expect(allUnclassified,
                "section overlay falls back to gray-red before region labels exist");
        }
        expect(viewport.overlay(QStringLiteral("another.module"), QStringLiteral("keep")) != nullptr,
            "rebuild clears only the rotation-body overlay owner");

        expect(controller.session().acceptAutomaticRegions(
            controller.session().generation(),
            uiRegions()),
            "overlay fixture region labels are accepted");
        expect(controller.session().acceptBoundary(
            controller.session().generation(),
            uiBoundary()),
            "overlay fixture boundary is accepted");
        expect(controller.rebuildViewportOverlay().success,
            "classified section and boundary overlays rebuild");
        section = viewport.overlay(ownerId, QStringLiteral("section"));
        const robot_qt_viewer::ViewportLineOverlay* boundary =
            viewport.overlay(ownerId, QStringLiteral("boundary"));
        expect(section != nullptr && section->segments.size() == uiRegions().segmentLabels.size(),
            "classified section keeps region segmentation");
        if(section != nullptr) {
            for(std::size_t index = 0; index < section->segments.size(); ++index) {
                const QColor expectedColor =
                    app::SectionView::semanticColor(uiRegions().segmentLabels[index]);
                const simulation_project::ColorDesc& actual = section->segments[index].color;
                expect(std::abs(actual.r * 255.0 - expectedColor.red()) < 1.0e-9 &&
                    std::abs(actual.g * 255.0 - expectedColor.green()) < 1.0e-9 &&
                    std::abs(actual.b * 255.0 - expectedColor.blue()) < 1.0e-9,
                    "classified overlay segment uses its UI semantic color");
            }
        }
        expect(boundary != nullptr && boundary->segments.size() == uiBoundary().polygonYz.size(),
            "boundary overlay closes the complete YZ polygon");
        if(boundary != nullptr) {
            const bool yellowOnPlanningPlane =
                std::all_of(boundary->segments.begin(),
                            boundary->segments.end(),
                            [](const robot_qt_viewer::ViewportLineSegment& segment) {
                                return segment.startWorld.x == 0.0 && segment.endWorld.x == 0.0 &&
                                       segment.color.r == 1.0 &&
                                       std::abs(segment.color.g * 255.0 - 193.0) < 1.0e-9 &&
                                       std::abs(segment.color.b * 255.0 - 7.0) < 1.0e-9;
                            });
            expect(yellowOnPlanningPlane, "boundary overlay is yellow and mapped to planning X=0");
        }

        app::WorkpieceCalibrationWorkspace calibration;
        calibration.cylinderRows[0] = { std::optional<double>(0.01),
                                        std::optional<double>(0.02),
                                        std::optional<double>(0.03) };
        calibration.yDirectionStart = { std::optional<double>(0.0),
                                        std::optional<double>(0.0),
                                        std::optional<double>(0.0) };
        calibration.yDirectionEnd = { std::optional<double>(0.0),
                                      std::optional<double>(0.05),
                                      std::optional<double>(0.0) };
        domain::CalibrationAxisFit calibrationFit;
        calibrationFit.mode = domain::CalibrationMode::Cylinder3d;
        calibrationFit.axisPointBaseMeters = Eigen::Vector3d::Zero();
        calibrationFit.axisDirectionBase = Eigen::Vector3d::UnitZ();
        calibrationFit.radiusMeters = 0.02;
        calibrationFit.sourcePointsBaseMeters.push_back({ 0.02, 0.0, 0.0 });
        calibration.cylinderFit = calibrationFit;
        expect(controller.session().setCalibrationWorkspace(calibration).ok() &&
                   controller.rebuildViewportOverlay().success &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-points")) != nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-fits")) != nullptr,
               "calibration point, Y reference and fit helpers are rendered");
        expect(controller.setCalibrationHelpersVisible(false).success &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-points")) == nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-fits")) == nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("section")) != nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("boundary")) != nullptr,
               "robot-view presentation hides only calibration helpers");

        const robot_qt_viewer::RobotQtViewerViewportLoadResult simulatedReload =
            viewport.loadProjectDocument(projectSession.document(), std::filesystem::path());
        expect(simulatedReload.success && viewport.overlays.isEmpty() &&
                   viewport.previewedObjectId.isEmpty() && viewport.focusedObjectId.isEmpty(),
               "fake viewport reload drops model preview, focus and overlays");
        expect(controller.restoreViewportPresentation().success &&
                   viewport.overlay(ownerId, QStringLiteral("section")) != nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("boundary")) != nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-points")) != nullptr &&
                   viewport.overlay(ownerId, QStringLiteral("calibration-fits")) != nullptr,
               "presentation restore rebuilds planning and calibration overlays "
               "after viewport reload");
        expect(
            viewport.previewedObjectId == QStringLiteral("overlay_workpiece") &&
                viewport.focusedObjectId ==
                    QStringLiteral("overlay_workpiece") &&
                viewport.focusedObjectView ==
                    robot_qt_viewer::RobotQtViewerObjectFocusView::Isometric &&
                std::abs(viewport.previewedObjectTransform.x - 0.11) <
                    1.0e-12 &&
                std::abs(viewport.previewedObjectTransform.y + 0.22) <
                    1.0e-12 &&
                std::abs(viewport.previewedObjectTransform.z - 0.33) <
                    1.0e-12 &&
                std::abs(viewport.previewedObjectTransform.yaw - 0.2) < 1.0e-12,
            "presentation restore replays the model preview with isometric "
            "focus");

        app::RotationBodyPlanningLeftPanel leftPanel;
        app::RotationBodyPlanningRightPanel rightPanel;
        app::RotationBodyPlanningWorkflowCoordinator coordinator(
            controller,
            leftPanel,
            rightPanel);
        coordinator.setLanguageCode(QStringLiteral("zh-CN"));
        QString presentationStatus;
        QObject::connect(
            &coordinator,
            &app::RotationBodyPlanningWorkflowCoordinator::statusMessageRequested,
            [&presentationStatus](const QString& message, int) {
                presentationStatus = message;
            });
        viewport.rejectOverlayUpserts = true;
        const app::RotationBodyControllerResult failedPresentation =
            coordinator.restoreViewportPresentation();
        const QString expectedPresentationError =
            app::RotationBodyPlanningTranslations::text(
                QStringLiteral("zh-CN"),
                "error.viewport_presentation");
        expect(!failedPresentation.success &&
            presentationStatus == expectedPresentationError,
            "coordinator reports a localized warning when viewport overlays cannot be restored");
        viewport.rejectOverlayUpserts = false;

        auto* returnButton = leftPanel.findChild<QToolButton*>(
            QStringLiteral("rotationBodyCommand.returnToWorkpiece"));
        coordinator.setMainViewMode(app::RotationBodyMainViewMode::Section);
        viewport.focusedObjectId = QStringLiteral("robot_selected_in_scene_explorer");
        const int focusCallsBeforeReturn = viewport.focusObjectCalls;
        if(returnButton != nullptr) {
            returnButton->click();
            QApplication::processEvents();
        }
        const QString expectedReturnStatus =
            app::RotationBodyPlanningTranslations::text(
                QStringLiteral("zh-CN"),
                "status.returned_to_workpiece");
        expect(returnButton != nullptr &&
            coordinator.mainViewMode() == app::RotationBodyMainViewMode::Scene3d &&
            viewport.focusObjectCalls == focusCallsBeforeReturn + 1 &&
            viewport.focusedObjectId == QStringLiteral("overlay_workpiece") &&
            presentationStatus == expectedReturnStatus,
            "persistent return command restores the planning workpiece after unrelated scene selection");

        const app::RotationBodyControllerResult exitResult = controller.deactivate();
        expect(exitResult.success && !controller.isActive(),
            "overlay controller exits cleanly");
        expect(!viewport.overlays.contains(ownerId) &&
            viewport.clearedOwners.contains(ownerId),
            "module exit clears its overlay owner after viewport reload");
    }

    void testCoordinatorReportsSourceChangedDraft()
    {
        ControllerFixture fixture;
        const std::filesystem::path sourceRoot =
            std::filesystem::u8path(ROTATION_BODY_TEST_SOURCE_ROOT);
        fixture.projectSession.resetNew();
        fixture.projectSession.setPath(
            sourceRoot / "RotationBodyPlanningSessionTest.sys.json");

        simulation_project::SceneObjectDesc object;
        object.id = "source_changed_workpiece";
        object.name = "source_changed_workpiece";
        object.objectType = "workpiece";
        object.sourcePath = "data/Spray420/ATPPZ350/meshes/Link2.STL";
        object.visualScale = 1.0;
        fixture.projectSession.document().objects.push_back(object);

        const auto loaded = app::RotationBodyMeshAdapter::load(
            fixture.projectSession,
            fixture.projectSession.document().objects.back());
        expect(loaded.ok(), "source-changed coordinator fixture loads a real STL");
        if(!loaded) {
            return;
        }

        app::RotationBodyPlanningSession staleSession;
        domain::TriangleMesh draftMesh = loaded.value.mesh;
        expect(staleSession.loadModel(
            object.id,
            object.sourcePath,
            "stale-source-fingerprint",
            std::move(draftMesh),
            identityAlignment(loaded.value.mesh),
            domain::PlanningObjectType::SimulationBlock,
            domain::SignedAxis::PositiveZ,
            domain::SignedAxis::PositiveY,
            1.0).ok(),
            "source-changed coordinator fixture creates a stale planning session");
        expect(staleSession.confirmFrame().ok() &&
            staleSession.acceptSection(staleSession.generation(), sampleSection()) &&
            staleSession.acceptAutomaticRegions(staleSession.generation(), sampleRegions()),
            "source-changed coordinator fixture contains derived planning progress");
        staleSession.setBoundaryMode(domain::BoundaryMode::ToothTopEnvelope);
        expect(staleSession.acceptBoundary(staleSession.generation(), sampleBoundary()),
            "source-changed coordinator fixture contains a confirmed boundary");
        expect(app::RotationBodyPlanningDraftStore::write(
            fixture.projectSession.document(),
            staleSession.makeDraft()),
            "source-changed coordinator fixture stores its private draft");

        app::RotationBodyPlanningLeftPanel leftPanel;
        app::RotationBodyPlanningRightPanel rightPanel;
        app::RotationBodyPlanningWorkflowCoordinator coordinator(
            fixture.controller,
            leftPanel,
            rightPanel);
        coordinator.setLanguageCode(QStringLiteral("zh-CN"));
        QString reportedStatus;
        QObject::connect(
            &coordinator,
            &app::RotationBodyPlanningWorkflowCoordinator::statusMessageRequested,
            [&reportedStatus](const QString& message, int) {
                reportedStatus = message;
            });

        const app::RotationBodyControllerResult restored = coordinator.activate();
        const QString sourceChangedStatus = app::RotationBodyPlanningTranslations::text(
            QStringLiteral("zh-CN"),
            "status.restored_source_changed");
        const QString ordinaryRestoreStatus = app::RotationBodyPlanningTranslations::text(
            QStringLiteral("zh-CN"),
            "status.restored");
        expect(restored.success &&
            restored.notice == app::RotationBodyControllerNotice::DraftSourceChanged &&
            fixture.controller.session().stage() == domain::PlanningStage::ModelLoaded &&
            fixture.controller.session().hasPendingChanges() &&
            !fixture.controller.session().section() &&
            reportedStatus == sourceChangedStatus &&
            reportedStatus != ordinaryRestoreStatus,
            "source-changed activation explains that alignment was refreshed and derived progress was invalidated");
        expect(coordinator.deactivate().success,
            "source-changed coordinator fixture exits cleanly");
    }

    app::RotationBodyPlanningViewModel uiViewModel(domain::PlanningStage stage)
    {
        app::RotationBodyPlanningViewModel view;
        view.stage = stage;
        view.hasModel = stage != domain::PlanningStage::NoModel;
        view.canEditModelTransform = view.hasModel;
        view.canConfirmFrame = stage == domain::PlanningStage::ModelLoaded;
        view.canCreateSection = static_cast<int>(stage) >=
            static_cast<int>(domain::PlanningStage::FrameConfirmed);
        view.canRecognizeRegions = static_cast<int>(stage) >=
            static_cast<int>(domain::PlanningStage::SectionReady);
        view.canEditRegions = static_cast<int>(stage) >=
            static_cast<int>(domain::PlanningStage::RegionsReady);
        view.canConfirmBoundary = view.canEditRegions;
        view.canRestoreAutomaticRegions = view.canEditRegions;
        view.canSaveProgress = view.hasModel;
        view.statistics.heightMeters = 0.42;
        view.statistics.maximumDiameterMeters = 0.30;
        view.statistics.minimumDiameterMeters = 0.18;
        view.statistics.boundsMinimum = { -0.01, -0.15, 0.0 };
        view.statistics.boundsMaximum = { 0.01, 0.15, 0.42 };
        view.statistics.estimatedAxisInMesh = Eigen::Vector3d::UnitZ();
        view.statistics.axisConfidence = 0.91;
        if(view.canRecognizeRegions) {
            view.sectionView.section = uiSection();
        }
        if(view.canEditRegions) {
            view.sectionView.regions = uiRegions();
            for(domain::RegionLabel label : view.sectionView.regions->segmentLabels) {
                ++view.regionSegmentCounts[static_cast<std::size_t>(label)];
            }
        }
        if(stage == domain::PlanningStage::SprayBoundaryConfirmed) {
            view.sectionView.boundary = uiBoundary();
        }
        return view;
    }

    void testWorkflowWidgetsAndTranslations()
    {
        app::RotationBodyWorkflowNavigation navigation;
        expect(navigation.workflowCount() == 2,
            "workflow navigation registers exactly two current pages");
        expect(navigation.workflowIds() == QStringList({
            QStringLiteral("modelTransform"),
            QStringLiteral("sectionRegion")
        }), "workflow ids contain no future trajectory or ABB entries");
        QString englishText;
        for(QToolButton* button : navigation.findChildren<QToolButton*>()) {
            englishText += button->text();
        }
        expect(englishText.contains(QStringLiteral("Model Transform")) &&
            englishText.contains(QStringLiteral("Section Partition")) &&
            englishText.contains(QStringLiteral("View Transform")) &&
            !englishText.contains(QStringLiteral("Trajectory")) &&
            !englishText.contains(QStringLiteral("ABB")),
            "navigation exposes only implemented English pages");

        navigation.setLanguageCode(QStringLiteral("zh_TW"));
        QString chineseText;
        for(QToolButton* button : navigation.findChildren<QToolButton*>()) {
            chineseText += button->text();
        }
        expect(navigation.languageCode() == QStringLiteral("zh-CN") &&
            chineseText.contains(QStringLiteral("模型变换")) &&
            chineseText.contains(QStringLiteral("剖切分区")) &&
            chineseText.contains(QStringLiteral("视角变换")),
            "Chinese variants canonicalize and retranslate live");
        navigation.setLanguageCode(QStringLiteral("unsupported"));
        expect(navigation.languageCode() == QStringLiteral("en"),
            "unknown language falls back to English");

        app::RotationBodyPlanningLeftPanel leftPanel;
        app::RotationBodyPlanningRightPanel rightPanel;
        expect(leftPanel.findChildren<QScrollArea*>().size() == 2 &&
            rightPanel.findChildren<QScrollArea*>().size() == 1,
            "trajectory planning keeps its own content scroll area");
        leftPanel.setLanguageCode(QStringLiteral("zh-CN"));
        rightPanel.setLanguageCode(QStringLiteral("zh-CN"));
        auto* saveButton = leftPanel.findChild<QPushButton*>(
            QStringLiteral("rotationBodyCommand.save"));
        auto* recognizeButton = leftPanel.findChild<QPushButton*>(
            QStringLiteral("rotationBodySection.recognize"));
        expect(saveButton != nullptr && saveButton->text().contains(QStringLiteral("保存进度")) &&
            recognizeButton != nullptr && recognizeButton->text().contains(QStringLiteral("自动识别")),
            "composite panels translate commands without rebuilding controls");
        leftPanel.setLanguageCode(QStringLiteral("en"));
        rightPanel.setLanguageCode(QStringLiteral("en"));

        auto* workflowStack = leftPanel.findChild<QStackedWidget*>(
            QStringLiteral("rotationBodyPlanningWorkflowStack"));
        auto* returnButton = leftPanel.findChild<QToolButton*>(
            QStringLiteral("rotationBodyCommand.returnToWorkpiece"));
        auto* flipButton = leftPanel.findChild<QToolButton*>(
            QStringLiteral("rotationBodyModel.flip"));
        auto* resetButton = leftPanel.findChild<QToolButton*>(
            QStringLiteral("rotationBodyModel.reset"));
        expect(workflowStack != nullptr && workflowStack->count() == 2 &&
            leftPanel.sectionRegionPanel() != nullptr &&
            rightPanel.hasVisibleContent() &&
            rightPanel.trajectoryPlanningPanel() != nullptr &&
            rightPanel.sectionRegionPanel() == nullptr,
            "left stack owns model and section pages while the right stack owns trajectory planning");
        expect(returnButton != nullptr &&
            returnButton->text() == app::RotationBodyPlanningTranslations::text(
                QStringLiteral("en"), "command.return_to_workpiece") &&
            flipButton != nullptr && flipButton->text() == QStringLiteral("Flip Workpiece") &&
            resetButton != nullptr && resetButton->text() == QStringLiteral("Restore Auto Alignment"),
            "persistent return, flip, and auto-alignment restore commands use descriptive text");
        QString modelPanelLabels;
        for(QLabel* label : leftPanel.modelTransformPanel()->findChildren<QLabel*>()) {
            modelPanelLabels += label->text();
        }
        expect(!modelPanelLabels.contains(
            QStringLiteral("confidence"), Qt::CaseInsensitive),
            "model transform UI does not expose alignment confidence or warnings");

        int deltaEmissionCount = 0;
        QObject::connect(
            &leftPanel,
            &app::RotationBodyPlanningLeftPanel::planningDeltaEdited,
            [&deltaEmissionCount](const domain::TransformComponents&) {
                ++deltaEmissionCount;
            });
        app::RotationBodyPlanningViewModel loaded =
            uiViewModel(domain::PlanningStage::ModelLoaded);
        loaded.planningDelta.translationMeters.x() = 0.012;
        leftPanel.setViewModel(loaded);
        expect(deltaEmissionCount == 0,
            "state-driven transform setter blocks user-edit signals");
        auto* xField = leftPanel.findChild<QDoubleSpinBox*>(
            QStringLiteral("rotationBodyModel.adjustment.X"));
        expect(xField != nullptr, "planning X editor is available for interaction");
        if(xField != nullptr) {
            xField->setValue(xField->value() + 1.0);
        }
        expect(deltaEmissionCount == 1,
            "one transform field edit emits exactly one planning delta");

        leftPanel.setCurrentWorkflow(app::RotationBodyWorkflow::SectionRegionPlanning);
        expect(workflowStack != nullptr &&
            workflowStack->currentWidget()->findChild<app::SectionRegionPanel*>() ==
                leftPanel.sectionRegionPanel(),
            "section and region planning is inside the second scrollable left page");
        const auto labelButtons = leftPanel.findChildren<QToolButton*>(
            QRegularExpression(QStringLiteral("rotationBodySection\\.label\\..*")));
        expect(labelButtons.size() == 5,
            "region planning exposes exactly five selectable labels");
    }

    #if 0 // Calibration UI moved to SMRobotWorkbenchCalibrationTranslation.
    void testWorkpieceCalibrationWidget()
    {
        app::WorkpieceCalibrationPanel panel;
        panel.setViewModel(uiViewModel(domain::PlanningStage::ModelLoaded));
        expect(panel.findChild<QTabWidget*>(QStringLiteral("rotationBodyCalibration.modes")) != nullptr,
            "calibration module exposes cylinder and circle calibration modes");

        auto* tabs = panel.findChild<QTabWidget*>(
            QStringLiteral("rotationBodyCalibration.modes"));
        auto* cylinderTable = panel.findChild<QTableWidget*>(
            QStringLiteral("rotationBodyCalibration.cylinderPoints"));
        auto* circleTable = panel.findChild<QTableWidget*>(
            QStringLiteral("rotationBodyCalibration.circlePoints"));
        expect(tabs != nullptr && cylinderTable != nullptr && circleTable != nullptr &&
            cylinderTable->rowCount() == 12 && circleTable->rowCount() == 6,
            "calibration keeps twelve cylinder touch rows and six circle touch rows");
        if(tabs == nullptr || circleTable == nullptr) return;

        tabs->setCurrentIndex(1);
        constexpr double centerXMillimeters = 500.0;
        constexpr double centerYMillimeters = -200.0;
        constexpr double centerZMillimeters = 1300.0;
        constexpr double radiusMillimeters = 100.0;
        for(int row = 0; row < 6; ++row)
        {
            constexpr double pi = 3.14159265358979323846;
            const double angle = 2.0 * pi * static_cast<double>(row) / 6.0;
            const std::array<double, 3> point{
                centerXMillimeters + radiusMillimeters * std::cos(angle),
                centerYMillimeters + radiusMillimeters * std::sin(angle),
                centerZMillimeters
            };
            for(int column = 0; column < 3; ++column)
            {
                circleTable->item(row, column)->setText(
                    QString::number(point[static_cast<std::size_t>(column)], 'f', 3));
            }
        }
        auto* fitButton = panel.findChild<QPushButton*>(
            QStringLiteral("rotationBodyCalibration.fitCurrent"));
        expect(fitButton != nullptr, "calibration exposes a current-mode fit command");
        if(fitButton == nullptr) return;
        fitButton->click();
        expect(panel.circleFit().has_value() &&
            std::abs(panel.circleFit()->radiusMeters - 0.1) < 1.0e-6,
            "calibration page converts millimeter entries and fits the six-point circle");

        const auto setCoordinate = [&panel](
            const QString& prefix,
            const std::array<double, 3>& point) {
            const std::array<QString, 3> axes{
                QStringLiteral("X"),
                QStringLiteral("Y"),
                QStringLiteral("Z")
            };
            for(int index = 0; index < 3; ++index)
            {
                if(QLineEdit* field = panel.findChild<QLineEdit*>(
                    QStringLiteral("rotationBodyCalibration.%1.%2")
                        .arg(prefix, axes[static_cast<std::size_t>(index)]))) {
                    field->setText(QString::number(
                        point[static_cast<std::size_t>(index)], 'f', 3));
                }
            }
        };
        setCoordinate(QStringLiteral("top"), { 600.0, -200.0, 1300.0 });
        setCoordinate(QStringLiteral("yStart"), { 500.0, -200.0, 1100.0 });
        setCoordinate(QStringLiteral("yEnd"), { 500.0, 0.0, 1100.0 });
        auto* height = panel.findChild<QDoubleSpinBox*>(
            QStringLiteral("rotationBodyCalibration.height"));
        if(height != nullptr) height->setValue(200.0);

        int emissionCount = 0;
        domain::TransformComponents emitted;
        QObject::connect(
            &panel,
            &app::WorkpieceCalibrationPanel::baseTransformCalculated,
            [&emissionCount, &emitted](const domain::TransformComponents& components) {
                ++emissionCount;
                emitted = components;
            });
        auto* calculateButton = panel.findChild<QPushButton*>(
            QStringLiteral("rotationBodyCalibration.calculateApply"));
        expect(calculateButton != nullptr && calculateButton->isEnabled(),
            "a valid fit enables calculation and application of the workpiece pose");
        if(calculateButton != nullptr) calculateButton->click();
        expect(emissionCount == 1 &&
            (emitted.translationMeters - Eigen::Vector3d(0.5, -0.2, 1.1)).norm() < 1.0e-9 &&
            emitted.rollPitchYawRadians.isZero(1.0e-9),
            "calibration forwards X/Y/Z/Rx/Ry/Rz through the right-panel base-pose signal");

        auto* poseResult = panel.findChild<QLabel*>(
            QStringLiteral("rotationBodyCalibration.poseResult"));
        const QString englishPose = poseResult != nullptr
            ? poseResult->text()
            : QString();
        panel.setLanguageCode(QStringLiteral("zh_CN"));
        expect(poseResult != nullptr && !englishPose.isEmpty() &&
            poseResult->text() != englishPose && panel.frameResult().has_value(),
            "calculated calibration result retranslates without losing its pose");
        panel.setLanguageCode(QStringLiteral("en"));
        expect(poseResult != nullptr && poseResult->text() == englishPose,
            "calibration pose result returns to English after live language switching");
    }
    #endif

    void testSixStageWidgetEnableMatrix()
    {
        app::ModelTransformPanel modelPanel;
        app::SectionRegionPanel sectionPanel;
        const std::array<domain::PlanningStage, 6> stages{ {
            domain::PlanningStage::NoModel,
            domain::PlanningStage::ModelLoaded,
            domain::PlanningStage::FrameConfirmed,
            domain::PlanningStage::SectionReady,
            domain::PlanningStage::RegionsReady,
            domain::PlanningStage::SprayBoundaryConfirmed
        } };
        for(domain::PlanningStage stage : stages) {
            const app::RotationBodyPlanningViewModel view = uiViewModel(stage);
            modelPanel.setViewModel(view);
            sectionPanel.setViewModel(view);
            const bool hasModel = stage != domain::PlanningStage::NoModel;
            const bool canConfirmFrame = stage == domain::PlanningStage::ModelLoaded;
            const bool canExtract = static_cast<int>(stage) >=
                static_cast<int>(domain::PlanningStage::FrameConfirmed);
            const bool canRecognize = static_cast<int>(stage) >=
                static_cast<int>(domain::PlanningStage::SectionReady);
            const bool canEdit = static_cast<int>(stage) >=
                static_cast<int>(domain::PlanningStage::RegionsReady);
            auto* confirmFrame = modelPanel.findChild<QPushButton*>(
                QStringLiteral("rotationBodyModel.confirmFrame"));
            auto* extract = sectionPanel.findChild<QPushButton*>(
                QStringLiteral("rotationBodySection.extract"));
            auto* recognize = sectionPanel.findChild<QPushButton*>(
                QStringLiteral("rotationBodySection.recognize"));
            auto* confirmBoundary = sectionPanel.findChild<QPushButton*>(
                QStringLiteral("rotationBodySection.confirmBoundary"));
            expect(confirmFrame != nullptr && confirmFrame->isEnabled() == canConfirmFrame,
                "frame confirmation follows the six-stage view model");
            expect(extract != nullptr && extract->isEnabled() == canExtract,
                "section extraction follows the six-stage view model");
            expect(recognize != nullptr && recognize->isEnabled() == canRecognize,
                "region recognition follows the six-stage view model");
            expect(confirmBoundary != nullptr && confirmBoundary->isEnabled() == canEdit,
                "boundary confirmation follows the six-stage view model");
        }
    }

    bool nearColor(const QColor& actual, const QColor& expected, int tolerance = 18)
    {
        return std::abs(actual.red() - expected.red()) <= tolerance &&
            std::abs(actual.green() - expected.green()) <= tolerance &&
            std::abs(actual.blue() - expected.blue()) <= tolerance;
    }

    void testSectionViewRenderingAndInteraction()
    {
        app::SectionView view;
        view.resize(560, 380);
        app::RotationBodySectionViewSnapshot snapshot;
        snapshot.section = uiSection();
        snapshot.regions = uiRegions();
        snapshot.boundary = uiBoundary();
        view.setSnapshot(snapshot);
        QApplication::processEvents();

        QImage image(view.size(), QImage::Format_ARGB32_Premultiplied);
        image.fill(Qt::transparent);
        {
            QPainter painter(&image);
            view.render(&painter);
        }
        std::array<int, 5> semanticPixels{};
        int boundaryPixels = 0;
        for(int y = 0; y < image.height(); ++y) {
            for(int x = 0; x < image.width(); ++x) {
                const QColor pixel = image.pixelColor(x, y);
                for(std::size_t index = 0; index < semanticPixels.size(); ++index) {
                    if(nearColor(pixel, app::SectionView::semanticColor(
                        static_cast<domain::RegionLabel>(index)))) {
                        ++semanticPixels[index];
                    }
                }
                if(nearColor(pixel, app::SectionView::boundaryColor())) {
                    ++boundaryPixels;
                }
            }
        }
        expect(std::all_of(
            semanticPixels.begin(),
            semanticPixels.end(),
            [](int count) { return count > 2; }),
            "section rendering contains all five semantic contour colors");
        expect(boundaryPixels > 10,
            "section rendering contains a visible yellow boundary");

        int rectangleCount = 0;
        domain::YzRectangle selectedRectangle;
        QObject::connect(
            &view,
            &app::SectionView::rectangleSelected,
            [&rectangleCount, &selectedRectangle](
                const domain::YzRectangle& rectangle,
                domain::RegionLabel label) {
                ++rectangleCount;
                selectedRectangle = rectangle;
                expect(label == domain::RegionLabel::ToothTop,
                    "section selection carries the active label");
            });
        const QPointF first = view.domainToViewport({ 0.14, 0.06 });
        const QPointF second = view.domainToViewport({ 0.30, 0.34 });
        QMouseEvent leftPress(
            QEvent::MouseButtonPress,
            first,
            Qt::LeftButton,
            Qt::LeftButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &leftPress);
        QMouseEvent leftMove(
            QEvent::MouseMove,
            second,
            Qt::NoButton,
            Qt::LeftButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &leftMove);
        QMouseEvent leftRelease(
            QEvent::MouseButtonRelease,
            second,
            Qt::LeftButton,
            Qt::NoButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &leftRelease);
        expect(rectangleCount == 1 &&
            selectedRectangle.minimum.x() <= selectedRectangle.maximum.x() &&
            selectedRectangle.minimum.y() <= selectedRectangle.maximum.y(),
            "left drag emits one normalized YZ rectangle");

        const double zoomBefore = view.zoomFactor();
        const QPointF zoomPoint(view.width() * 0.6, view.height() * 0.45);
        const Eigen::Vector2d anchorBefore = view.viewportToDomain(zoomPoint);
        QWheelEvent wheel(
            zoomPoint,
            view.mapToGlobal(zoomPoint.toPoint()),
            QPoint(),
            QPoint(0, 120),
            Qt::NoButton,
            Qt::NoModifier,
            Qt::NoScrollPhase,
            false);
        QApplication::sendEvent(&view, &wheel);
        const Eigen::Vector2d anchorAfter = view.viewportToDomain(zoomPoint);
        expect(view.zoomFactor() > zoomBefore && anchorAfter.isApprox(anchorBefore, 1.0e-9),
            "wheel zoom remains anchored under the cursor");

        const Eigen::Vector2d centerBefore = view.viewCenterYz();
        const QPointF panStart(view.width() * 0.5, view.height() * 0.5);
        const QPointF panEnd = panStart + QPointF(35.0, -20.0);
        QMouseEvent rightPress(
            QEvent::MouseButtonPress,
            panStart,
            Qt::RightButton,
            Qt::RightButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &rightPress);
        QMouseEvent rightMove(
            QEvent::MouseMove,
            panEnd,
            Qt::NoButton,
            Qt::RightButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &rightMove);
        QMouseEvent rightRelease(
            QEvent::MouseButtonRelease,
            panEnd,
            Qt::RightButton,
            Qt::NoButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &rightRelease);
        expect(!view.viewCenterYz().isApprox(centerBefore, 1.0e-12),
            "right drag pans the section view");

        QMouseEvent cancelledPress(
            QEvent::MouseButtonPress,
            first,
            Qt::LeftButton,
            Qt::LeftButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &cancelledPress);
        QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
        QApplication::sendEvent(&view, &escape);
        QMouseEvent cancelledRelease(
            QEvent::MouseButtonRelease,
            second,
            Qt::LeftButton,
            Qt::NoButton,
            Qt::NoModifier);
        QApplication::sendEvent(&view, &cancelledRelease);
        expect(rectangleCount == 1, "Escape cancels an in-progress rectangle selection");
    }
}

int main(int argc, char** argv)
{
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication application(argc, argv);
    testSessionGenerationAndInvalidation();
    testSessionClearRestoresDefaults();
    testTrajectoryWorkspacePersistenceAndInvalidation();
    testDraftRoundTripAndDegradation();
    testSessionDraftRestoreSourceChange();
    testMeshAdapter();
    testPlanningObjectReplacement();
    testControllerActivationIsolationAndRollback();
    testControllerRecoversFromOrphanedDraft();
    testControllerAsyncPlanningOperations();
    testControllerAsyncImportAndCancellation();
    testControllerDestructionDrainsPrivateWorkerPool();
    testSaveCommitSurvivesViewportReloadFailure();
    testCoordinatorReportsCommittedSaveWarning();
    testWorkflowWidgetsAndTranslations();
    testSixStageWidgetEnableMatrix();
    testSectionViewRenderingAndInteraction();
    testViewportOverlays();
    testCoordinatorReportsSourceChangedDraft();
    if(failures != 0) {
        std::cerr << failures << " regression assertion(s) failed.\n";
        return 1;
    }
    std::cout << "Rotation-body planning editor regression passed.\n";
    return 0;
}
