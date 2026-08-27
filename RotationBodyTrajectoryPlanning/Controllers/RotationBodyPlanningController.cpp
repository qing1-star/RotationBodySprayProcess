#include "RotationBodyPlanningController.h"

#include "../Adapters/RotationBodyMeshAdapter.h"
#include "../Persistence/RotationBodyPlanningDocumentOperations.h"
#include "../Persistence/RotationBodyPlanningDraftStore.h"
#include "../Persistence/RotationBodyTrajectoryProjectStore.h"

#include <RotationBodyTrajectoryPlanning/Alignment/RotationBodyAlignmentSolver.h>
#include <RotationBodyTrajectoryPlanning/Alignment/ModelTransformOperations.h>
#include <RotationBodyTrajectoryPlanning/Alignment/SimulationBlockPlacementSolver.h>
#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/SprayBoundaryBuilder.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/ToothRegionRecognizer.h>
#include <RotationBodyTrajectoryPlanning/Sectioning/YzSectionExtractor.h>
#include <CalibrationInstructionTranslation/ABBTranslation/RapidModuleGenerator.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/MergedTrajectoryTextExporter.h>

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "RobotQtViewerViewportServices.h"
#include "SceneEntityWorkflowController.h"
#include "ViewportReloadWorkflowController.h"

#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>
#include <SimulationProject/RuntimePaths.h>

#include <QMetaObject>
#include <QPointer>
#include <QCoreApplication>
#include <QEvent>
#include <QRunnable>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        const QString overlayOwnerId =
            QStringLiteral("spray.rotation_body_trajectory_planning");

        simulation_project::ColorDesc overlayColor(
            int red,
            int green,
            int blue,
            double alpha = 1.0)
        {
            return {
                static_cast<double>(red) / 255.0,
                static_cast<double>(green) / 255.0,
                static_cast<double>(blue) / 255.0,
                alpha
            };
        }

        simulation_project::ColorDesc regionOverlayColor(domain::RegionLabel label)
        {
            switch(label) {
            case domain::RegionLabel::Unclassified: return overlayColor(156, 103, 112);
            case domain::RegionLabel::ToothTop: return overlayColor(0, 119, 255);
            case domain::RegionLabel::ToothWall: return overlayColor(255, 122, 0);
            case domain::RegionLabel::ToothBottom: return overlayColor(22, 163, 74);
            case domain::RegionLabel::Transition: return overlayColor(217, 70, 239);
            }
            return overlayColor(156, 103, 112);
        }

        simulation_project::Vec3Desc worldPoint(const Eigen::Vector3d& point)
        {
            return { point.x(), point.y(), point.z() };
        }

        void appendPath(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const std::vector<domain::TrajectoryPosePoint>& points,
            const Eigen::Isometry3d& displayFromPlanning,
            const simulation_project::ColorDesc& color)
        {
            if(points.size() < 2) {
                return;
            }
            overlay.segments.reserve(overlay.segments.size() + points.size() - 1);
            for(std::size_t index = 1; index < points.size(); ++index) {
                robot_qt_viewer::ViewportLineSegment segment;
                segment.startWorld = worldPoint(
                    displayFromPlanning *
                    points[index - 1].planningFromTool.translation());
                segment.endWorld = worldPoint(
                    displayFromPlanning * points[index].planningFromTool.translation());
                segment.color = color;
                overlay.segments.push_back(std::move(segment));
            }
        }

        void appendPoseFrame(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Isometry3d& planningFromTool,
            double axisLengthMeters,
            bool selected)
        {
            const Eigen::Vector3d origin = planningFromTool.translation();
            const std::array<simulation_project::ColorDesc, 3> colors = selected
                ? std::array<simulation_project::ColorDesc, 3>{
                    overlayColor(255, 72, 72),
                    overlayColor(86, 255, 118),
                    overlayColor(88, 170, 255) }
                : std::array<simulation_project::ColorDesc, 3>{
                    overlayColor(190, 56, 56),
                    overlayColor(45, 180, 80),
                    overlayColor(55, 115, 205) };
            for(int axis = 0; axis < 3; ++axis) {
                robot_qt_viewer::ViewportLineSegment segment;
                segment.startWorld = worldPoint(origin);
                segment.endWorld = worldPoint(
                    origin + axisLengthMeters * planningFromTool.linear().col(axis));
                segment.color = colors[static_cast<std::size_t>(axis)];
                overlay.segments.push_back(std::move(segment));
            }
        }

        const std::vector<domain::TrajectoryPosePoint>& displayedTrajectoryPoints(
            const domain::PlannedTrajectory& trajectory,
            domain::TrajectoryDisplayMode mode)
        {
            return mode == domain::TrajectoryDisplayMode::RelativeHelical
                ? trajectory.relativeHelicalPoints
                : trajectory.linearPoints;
        }

        void appendCrossMarker(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Vector3d& center,
            double radiusMeters,
            const simulation_project::ColorDesc& color)
        {
            for(int axis = 0; axis < 3; ++axis) {
                Eigen::Vector3d offset = Eigen::Vector3d::Zero();
                offset[axis] = radiusMeters;
                robot_qt_viewer::ViewportLineSegment segment;
                segment.startWorld = worldPoint(center - offset);
                segment.endWorld = worldPoint(center + offset);
                segment.color = color;
                overlay.segments.push_back(std::move(segment));
            }
        }

        void appendCircle(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Vector3d& center,
            const Eigen::Vector3d& normal,
            double radiusMeters,
            const simulation_project::ColorDesc& color);

        void appendCalibrationMarker(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Vector3d& center,
            double radiusMeters,
            const simulation_project::ColorDesc& color)
        {
            // ManualCalibration presents touch samples as a compact glow/ring
            // marker rather than a bare axis cross.  The viewport overlay API
            // is line-only, so compose the same visual from a ring and a
            // shorter center cross.
            for(const Eigen::Vector3d& normal : {
                Eigen::Vector3d::UnitX(),
                Eigen::Vector3d::UnitY(),
                Eigen::Vector3d::UnitZ() }) {
                appendCircle(overlay, center, normal, radiusMeters, color);
            }
            appendCrossMarker(overlay, center, radiusMeters * 0.42, color);
        }

        void appendLine(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Vector3d& start,
            const Eigen::Vector3d& end,
            const simulation_project::ColorDesc& color)
        {
            robot_qt_viewer::ViewportLineSegment segment;
            segment.startWorld = worldPoint(start);
            segment.endWorld = worldPoint(end);
            segment.color = color;
            overlay.segments.push_back(std::move(segment));
        }

        Eigen::Vector3d perpendicularTo(const Eigen::Vector3d& normal)
        {
            const Eigen::Vector3d n = normal.normalized();
            Eigen::Vector3d candidate = std::abs(n.z()) < 0.8
                ? Eigen::Vector3d::UnitZ()
                : Eigen::Vector3d::UnitX();
            candidate -= candidate.dot(n) * n;
            return candidate.normalized();
        }

        void appendCircle(
            robot_qt_viewer::ViewportLineOverlay& overlay,
            const Eigen::Vector3d& center,
            const Eigen::Vector3d& normal,
            double radiusMeters,
            const simulation_project::ColorDesc& color)
        {
            if(!center.allFinite() || !normal.allFinite() ||
                normal.norm() <= 1.0e-12 || !std::isfinite(radiusMeters) ||
                radiusMeters <= 0.0) {
                return;
            }
            constexpr int segmentCount = 64;
            const Eigen::Vector3d u = perpendicularTo(normal);
            const Eigen::Vector3d v = normal.normalized().cross(u).normalized();
            Eigen::Vector3d previous = center + radiusMeters * u;
            for(int index = 1; index <= segmentCount; ++index) {
                const double angle = 2.0 * 3.14159265358979323846 *
                    static_cast<double>(index) / static_cast<double>(segmentCount);
                const Eigen::Vector3d current = center + radiusMeters *
                    (std::cos(angle) * u + std::sin(angle) * v);
                appendLine(overlay, previous, current, color);
                previous = current;
            }
        }

        std::optional<Eigen::Vector3d> calibrationPoint(
            const CalibrationCoordinateRow& row)
        {
            if(!row[0] || !row[1] || !row[2]) {
                return std::nullopt;
            }
            return Eigen::Vector3d(*row[0], *row[1], *row[2]);
        }

        domain::PlanningResult<domain::AlignmentResult> solveAlignment(
            const domain::TriangleMesh& mesh,
            const RotationBodyImportOptions& options)
        {
            return options.objectType == domain::PlanningObjectType::CompletePart
                ? domain::RotationBodyAlignmentSolver::solve(mesh)
                : domain::SimulationBlockPlacementSolver::solve(
                    mesh,
                    options.originalRotationAxis,
                    options.toothOutwardAxis,
                    options.motherMaximumDiameterMeters);
        }

        std::string defaultRapidOutputDirectory()
        {
            const std::filesystem::path dataRoot =
                std::filesystem::absolute(simulation_project::RuntimePaths::dataRoot());
            return (dataRoot / "ABBrapid").u8string();
        }

        std::filesystem::path rapidOutputFile(const domain::RapidExportSettings& settings)
        {
            std::filesystem::path fileName = std::filesystem::u8path(settings.fileName);
            // The UI owns the directory; only the leaf name is accepted here.
            // This prevents an accidental absolute path or nested path from
            // silently bypassing the selected ABBrapid folder.
            fileName = fileName.filename();
            if(fileName.extension().empty()) {
                fileName += ".mod";
            }
            return std::filesystem::u8path(settings.outputDirectory) / fileName;
        }

        std::filesystem::path defaultMergedTrajectoryOutputDirectory()
        {
            const std::filesystem::path sourceDataRoot =
                std::filesystem::absolute(simulation_project::RuntimePaths::sourceRoot()) /
                "data";
            std::error_code error;
            if(std::filesystem::is_directory(sourceDataRoot, error) && !error) {
                return sourceDataRoot / "traj";
            }
            return std::filesystem::absolute(
                simulation_project::RuntimePaths::dataRoot()) / "traj";
        }

        std::filesystem::path nextMergedTrajectoryOutputFile(
            const std::filesystem::path& directory)
        {
            for(std::size_t index = 0;; ++index) {
                const std::filesystem::path candidate = directory /
                    ("MergedTrajectory_" + std::to_string(index) + "_0.txt");
                if(!std::filesystem::exists(candidate)) {
                    return candidate;
                }
            }
        }
    }

    struct RotationBodyPlanningController::AsyncImportContext
    {
        RotationBodyPlanningSession previousSession;
        robot_qt_viewer::SceneEntityImportResult imported;
        simulation_project::SceneObjectDesc object;
        RotationBodyImportOptions options;
        std::filesystem::path resolvedSourcePath;
        std::uint64_t inputGeneration{ 0 };
    };

    struct RotationBodyPlanningController::AsyncImportResult
    {
        domain::PlanningResult<RotationBodyMeshLoad> loaded;
        domain::PlanningResult<domain::AlignmentResult> alignment;
    };

    RotationBodyPlanningController::RotationBodyPlanningController(
        robot_qt_viewer::RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_context(context)
    {
        m_workerPool.setMaxThreadCount(1);
    }

    RotationBodyPlanningController::~RotationBodyPlanningController()
    {
        cancelAsyncOperation();
        m_workerPool.clear();
        m_workerPool.waitForDone();
        QCoreApplication::removePostedEvents(this, QEvent::MetaCall);
    }

    RotationBodyPlanningSession& RotationBodyPlanningController::session() noexcept
    {
        return m_session;
    }

    const RotationBodyPlanningSession& RotationBodyPlanningController::session() const noexcept
    {
        return m_session;
    }

    RotationBodyPlanningViewModel RotationBodyPlanningController::viewModel() const
    {
        RotationBodyPlanningViewModel view;
        view.stage = m_session.stage();
        view.generation = m_session.generation();
        view.objectId = m_session.objectId();
        view.sourcePath = m_session.sourcePath();
        view.errorCode = m_session.error().code;
        view.objectType = m_session.objectType();
        view.originalRotationAxis = m_session.originalRotationAxis();
        view.toothOutwardAxis = m_session.toothOutwardAxis();
        view.motherMaximumDiameterMeters = m_session.motherMaximumDiameterMeters();
        view.statistics = m_session.statistics();
        view.publishFrame = m_session.publishFrame();
        view.boundaryMode = m_session.boundaryMode();
        view.sectionView.section = m_session.section();
        view.sectionView.regions = m_session.resolvedRegions();
        view.sectionView.boundary = m_session.boundary();
        view.trajectoryWorkspace = m_session.trajectoryWorkspace();
        if(view.trajectoryWorkspace.currentTrajectory) {
            const std::vector<domain::TrajectoryPosePoint>& points =
                displayedTrajectoryPoints(
                    *view.trajectoryWorkspace.currentTrajectory,
                    view.trajectoryWorkspace.displayMode);
            if(points.size() >= 2) {
                const Eigen::Vector3d& start =
                    points.front().planningFromTool.translation();
                const Eigen::Vector3d& end =
                    points.back().planningFromTool.translation();
                view.sectionView.trajectoryStartYz = Eigen::Vector2d(start.y(), start.z());
                view.sectionView.trajectoryEndYz = Eigen::Vector2d(end.y(), end.z());
            }
        }
        view.rapidSettings = m_session.rapidSettings();
        view.rapidSequence = m_session.rapidSequence();
        view.calibrationWorkspace = m_session.calibrationWorkspace();
        view.uiState = m_session.uiState();
        view.rapidModulePreview = m_rapidModulePreview;
        view.rapidOutputFile = m_rapidOutputFile;
        view.rapidExportStatus = m_rapidExportStatus;
        view.rapidExportSucceeded = m_rapidExportSucceeded;
        const domain::PlanningResult<domain::TransformComponents> planningDelta =
            domain::transformComponents(
                m_session.planningFromMesh() * m_session.automaticBaseline().inverse());
        if(planningDelta) {
            view.planningDelta = planningDelta.value;
        }
        const domain::PlanningResult<domain::TransformComponents> baseComponents =
            domain::transformComponents(m_session.baseFromPlanning());
        if(baseComponents) {
            view.baseFromPlanning = baseComponents.value;
        }
        if(view.sectionView.regions) {
            for(domain::RegionLabel label : view.sectionView.regions->segmentLabels) {
                const std::size_t index = static_cast<std::size_t>(label);
                if(index < view.regionSegmentCounts.size()) {
                    ++view.regionSegmentCounts[index];
                }
            }
        }
        view.hasModel = m_session.hasModel();
        view.isBusy = m_busy;
        const bool commandsEnabled = !m_busy;
        view.canEditModelTransform = commandsEnabled && view.hasModel;
        view.canConfirmFrame = commandsEnabled &&
            view.stage == domain::PlanningStage::ModelLoaded;
        view.canCreateSection = commandsEnabled && static_cast<int>(view.stage) >=
            static_cast<int>(domain::PlanningStage::FrameConfirmed);
        view.canRecognizeRegions = commandsEnabled && static_cast<int>(view.stage) >=
            static_cast<int>(domain::PlanningStage::SectionReady);
        view.canEditRegions = commandsEnabled && static_cast<int>(view.stage) >=
            static_cast<int>(domain::PlanningStage::RegionsReady);
        if(commandsEnabled) {
            if(const domain::RegionEditHistory* history = m_session.editHistory()) {
                view.canUndoRegionEdit = history->canUndo();
                view.canRedoRegionEdit = history->canRedo();
                view.canRestoreAutomaticRegions = true;
            }
        }
        view.canConfirmBoundary = view.canEditRegions;
        view.canReopenBoundary = commandsEnabled && view.sectionView.boundary.has_value();
        view.canGenerateTrajectory = commandsEnabled && view.sectionView.boundary.has_value();
        view.canEditCurrentTrajectory = commandsEnabled &&
            view.trajectoryWorkspace.currentTrajectory.has_value();
        view.canSaveCurrentTrajectory = view.canEditCurrentTrajectory;
        view.canEditTrajectoryGroup = commandsEnabled &&
            !view.trajectoryWorkspace.group.passes.empty();
        view.canExportRapid = commandsEnabled && !view.rapidSequence.empty();
        view.canSaveProgress = commandsEnabled && view.hasModel;
        view.hasPendingChanges = m_session.hasPendingChanges();
        return view;
    }

    RotationBodyControllerResult RotationBodyPlanningController::activate()
    {
        if(m_active) {
            const RotationBodyControllerResult presentation =
                restoreViewportPresentation();
            if(!presentation.success) {
                publishState(presentation.message, 5000);
                return presentation;
            }
            publishState();
            return { true, {} };
        }

        clearPreviewAndFocus();
        clearRapidPreview();
        m_session.clear();
        m_session.setDefaultRapidOutputDirectory(defaultRapidOutputDirectory());

        // A Scene Explorer deletion cannot update this module's private extension.
        // Remove that orphan before taking the entry snapshot so discard cannot
        // resurrect it and make every later activation fail in the same way.
        const DraftReadResult identityRead =
            RotationBodyPlanningDraftStore::read(m_context.document());
        if(identityRead.usable() && identityRead.draft &&
            findObject(m_context.document(), identityRead.draft->objectId) == nullptr) {
            const robot_qt_viewer::ProjectMutationResult cleanup =
                m_context.documentController().mutateProject(
                    QStringLiteral("rotationBodyPlanningRemoveOrphanedDraft"),
                    robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                    [](simulation_project::ProjectDocumentService& service,
                        bool& changed,
                        std::string&) {
                        changed = RotationBodyPlanningDraftStore::erase(service.document());
                        changed = RotationBodyTrajectoryProjectStore::eraseAll(
                            service.document()) || changed;
                        return true;
                    });
            if(!cleanup.success) {
                const QString message = cleanup.message.isEmpty()
                    ? QStringLiteral("The orphaned rotation-body planning draft could not be removed.")
                    : cleanup.message;
                m_session.setError({
                    domain::PlanningErrorCode::InvalidArgument,
                    message.toStdString()
                });
                clearPreviewAndFocus();
                publishState(message, 5000);
                return { false, message };
            }
        }

        m_entryDocument = m_context.document();
        m_entryDirty = m_context.projectSession().isDirty();
        m_hasEntrySnapshot = true;
        m_active = true;

        const auto rollbackActivation = [this](RotationBodyControllerResult result) {
            if(result.message.isEmpty()) {
                result.message = QStringLiteral(
                    "Rotation-body planning could not be restored.");
            }
            m_active = false;
            clearEntrySnapshot();
            m_session.clear();
            m_session.setError({
                domain::PlanningErrorCode::InvalidArgument,
                result.message.toStdString()
            });
            clearPreviewAndFocus();
            publishState(result.message, 5000);
            return result;
        };

        const RotationBodyControllerResult resumed = resumeDraft();
        if(!resumed.success) {
            return rollbackActivation(resumed);
        }
        const RotationBodyControllerResult presentation =
            restoreViewportPresentation();
        if(!presentation.success) {
            return rollbackActivation(presentation);
        }
        publishState(resumed.message, resumed.message.isEmpty() ? 0 : 3000);
        return resumed;
    }

    RotationBodyControllerResult RotationBodyPlanningController::deactivate()
    {
        if(!m_active) {
            clearPreviewAndFocus();
            return { true, {} };
        }
        return discardAndExit();
    }

    bool RotationBodyPlanningController::isActive() const noexcept
    {
        return m_active;
    }

    bool RotationBodyPlanningController::isBusy() const noexcept
    {
        return m_busy;
    }

    bool RotationBodyPlanningController::hasPendingChanges() const noexcept
    {
        return m_session.hasPendingChanges();
    }

    RotationBodyControllerResult RotationBodyPlanningController::startImportModel(
        const std::filesystem::path& sourcePath,
        const RotationBodyImportOptions& options)
    {
        if(m_busy) {
            return busyFailure();
        }
        if(!m_active) {
            const RotationBodyControllerResult activation = activate();
            if(!activation.success) {
                return activation;
            }
        }
        if(sourcePath.empty()) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "A source model path is required."
            });
        }

        const RotationBodyPlanningSession previousSession = m_session;
        robot_qt_viewer::SceneEntityWorkflowController workflow(m_context);
        const robot_qt_viewer::SceneEntityImportResult imported =
            workflow.importSceneObjectFromPath(sourcePath, "workpiece");
        if(!imported.success) {
            m_session.setError({
                domain::PlanningErrorCode::InvalidArgument,
                imported.message.toStdString()
            });
            publishState(imported.message, 5000);
            return { false, imported.message };
        }

        const auto rollbackPreparation = [this, &workflow, &imported, &previousSession](
            const domain::PlanningError& error) {
            workflow.restoreImportState(imported);
            m_session = previousSession;
            reloadViewport(QStringLiteral("rotationBodyPlanningAsyncImportPreparationRollback"));
            m_session.setError(error);
            const QString message = QString::fromStdString(error.message);
            publishState(message, 5000);
            return RotationBodyControllerResult{ false, message };
        };

        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), imported.entityId.toStdString());
        if(object == nullptr) {
            return rollbackPreparation({
                domain::PlanningErrorCode::EmptyMesh,
                "The imported workpiece is missing from the project."
            });
        }
        const domain::PlanningResult<std::filesystem::path> resolvedSourcePath =
            RotationBodyMeshAdapter::resolveSourcePath(m_context.projectSession(), *object);
        if(!resolvedSourcePath) {
            return rollbackPreparation(resolvedSourcePath.error);
        }

        auto context = std::make_shared<AsyncImportContext>();
        context->previousSession = previousSession;
        context->imported = imported;
        context->object = *object;
        context->options = options;
        context->resolvedSourcePath = resolvedSourcePath.value;
        context->inputGeneration = previousSession.generation();

        const std::uint64_t token = beginAsyncOperation();
        const QPointer<RotationBodyPlanningController> guard(this);
        m_workerPool.start(QRunnable::create(
            [guard, token, context]() {
                auto result = std::make_shared<AsyncImportResult>();
                try {
                    result->loaded = RotationBodyMeshAdapter::loadResolvedSource(
                        context->resolvedSourcePath,
                        context->object.visualScale);
                    if(result->loaded) {
                        result->alignment = solveAlignment(
                            result->loaded.value.mesh,
                            context->options);
                    }
                } catch(const std::exception& exception) {
                    result->loaded = domain::PlanningResult<RotationBodyMeshLoad>::failure(
                        domain::PlanningErrorCode::AlignmentFailed,
                        std::string("Model loading or alignment failed unexpectedly: ") +
                            exception.what());
                } catch(...) {
                    result->loaded = domain::PlanningResult<RotationBodyMeshLoad>::failure(
                        domain::PlanningErrorCode::AlignmentFailed,
                        "Model loading or alignment failed with an unknown error.");
                }
                if(!guard) {
                    return;
                }
                QMetaObject::invokeMethod(
                    guard.data(),
                    [guard, token, context, result]() {
                        if(guard) {
                            guard->completeAsyncImport(token, context, result);
                        }
                    },
                    Qt::QueuedConnection);
            }));
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::startExtractSection()
    {
        if(m_busy) {
            return busyFailure();
        }
        const domain::TriangleMesh* mesh = m_session.mesh();
        if(mesh == nullptr) {
            return domainFailure({
                domain::PlanningErrorCode::EmptyMesh,
                "Load and align a model before extracting its section."
            });
        }
        if(static_cast<int>(m_session.stage()) <
            static_cast<int>(domain::PlanningStage::FrameConfirmed)) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "Confirm the workpiece frame before extracting a section."
            });
        }

        const domain::TriangleMesh meshSnapshot = *mesh;
        const Eigen::Isometry3d planningFromMesh = m_session.planningFromMesh();
        const domain::YzSectionOptions options = m_session.parameters().section;
        const std::uint64_t inputGeneration = m_session.generation();
        const std::uint64_t token = beginAsyncOperation();
        const QPointer<RotationBodyPlanningController> guard(this);
        m_workerPool.start(QRunnable::create(
            [guard,
                token,
                inputGeneration,
                meshSnapshot,
                planningFromMesh,
                options]() mutable {
                auto result = std::make_shared<
                    domain::PlanningResult<domain::SectionContour>>();
                try {
                    *result = domain::YzSectionExtractor::extractTargetContour(
                        meshSnapshot,
                        planningFromMesh,
                        options);
                } catch(const std::exception& exception) {
                    *result = domain::PlanningResult<domain::SectionContour>::failure(
                        domain::PlanningErrorCode::SectionFailed,
                        std::string("Section extraction failed unexpectedly: ") +
                            exception.what());
                } catch(...) {
                    *result = domain::PlanningResult<domain::SectionContour>::failure(
                        domain::PlanningErrorCode::SectionFailed,
                        "Section extraction failed with an unknown error.");
                }
                if(!guard) {
                    return;
                }
                QMetaObject::invokeMethod(
                    guard.data(),
                    [guard, token, inputGeneration, result]() mutable {
                        if(guard) {
                            guard->completeAsyncSection(
                                token,
                                inputGeneration,
                                std::move(*result));
                        }
                    },
                    Qt::QueuedConnection);
            }));
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::startRecognizeRegions()
    {
        if(m_busy) {
            return busyFailure();
        }
        if(!m_session.section()) {
            return domainFailure({
                domain::PlanningErrorCode::NoContour,
                "Extract a section before recognizing spray regions."
            });
        }

        const domain::SectionContour sectionSnapshot = *m_session.section();
        const domain::ToothRecognitionOptions options = m_session.parameters().region;
        const std::uint64_t inputGeneration = m_session.generation();
        const std::uint64_t token = beginAsyncOperation();
        const QPointer<RotationBodyPlanningController> guard(this);
        m_workerPool.start(QRunnable::create(
            [guard, token, inputGeneration, sectionSnapshot, options]() mutable {
                auto result = std::make_shared<
                    domain::PlanningResult<domain::RegionAssignment>>();
                try {
                    *result = domain::ToothRegionRecognizer::recognize(
                        sectionSnapshot,
                        options);
                } catch(const std::exception& exception) {
                    *result = domain::PlanningResult<domain::RegionAssignment>::failure(
                        domain::PlanningErrorCode::InsufficientRegionData,
                        std::string("Automatic region recognition failed unexpectedly: ") +
                            exception.what());
                } catch(...) {
                    *result = domain::PlanningResult<domain::RegionAssignment>::failure(
                        domain::PlanningErrorCode::InsufficientRegionData,
                        "Automatic region recognition failed with an unknown error.");
                }
                if(!guard) {
                    return;
                }
                QMetaObject::invokeMethod(
                    guard.data(),
                    [guard, token, inputGeneration, result]() mutable {
                        if(guard) {
                            guard->completeAsyncRecognition(
                                token,
                                inputGeneration,
                                std::move(*result));
                        }
                    },
                    Qt::QueuedConnection);
            }));
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::importModel(
        const std::filesystem::path& sourcePath,
        const RotationBodyImportOptions& options)
    {
        if(m_busy) {
            return busyFailure();
        }
        if(!m_active) {
            const RotationBodyControllerResult activation = activate();
            if(!activation.success) {
                return activation;
            }
        }
        if(sourcePath.empty()) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "A source model path is required."
            });
        }

        const RotationBodyPlanningSession previousSession = m_session;
        robot_qt_viewer::SceneEntityWorkflowController workflow(m_context);
        const robot_qt_viewer::SceneEntityImportResult imported =
            workflow.importSceneObjectFromPath(sourcePath, "workpiece");
        if(!imported.success) {
            m_session.setError({
                domain::PlanningErrorCode::InvalidArgument,
                imported.message.toStdString()
            });
            publishState(imported.message, 5000);
            return { false, imported.message };
        }

        const auto rollbackImport = [&](const QString& sourceId) {
            workflow.restoreImportState(imported);
            m_session = previousSession;
            reloadViewport(sourceId);
            if(m_session.hasModel()) {
                previewAndFocus();
            } else {
                clearPreviewAndFocus();
            }
        };

        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), imported.entityId.toStdString());
        if(object == nullptr) {
            rollbackImport(QStringLiteral("rotationBodyPlanningMissingImportRollback"));
            const QString message = QStringLiteral("The imported workpiece is missing from the project.");
            m_session.setError({
                domain::PlanningErrorCode::EmptyMesh,
                message.toStdString()
            });
            publishState(message, 5000);
            return { false, message };
        }

        const RotationBodyControllerResult aligned = alignImportedObject(*object, options);
        if(!aligned.success) {
            rollbackImport(QStringLiteral("rotationBodyPlanningAlignmentRollback"));
            m_session.setError({
                domain::PlanningErrorCode::AlignmentFailed,
                aligned.message.toStdString()
            });
            publishState(aligned.message, 5000);
            return aligned;
        }

        if(previousSession.hasModel() &&
            previousSession.objectId() != m_session.objectId()) {
            const robot_qt_viewer::ProjectMutationResult replacement =
                m_context.documentController().mutateProject(
                    QStringLiteral("rotationBodyPlanningReplaceWorkpiece"),
                    robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                    [&](simulation_project::ProjectDocumentService& service,
                        bool& changed,
                        std::string& error) {
                        const PlanningObjectReplacementResult result =
                            RotationBodyPlanningDocumentOperations::replacePlanningObject(
                                service,
                                previousSession.objectId(),
                                m_session.objectId());
                        if(!result.success) {
                            error = result.error;
                            return false;
                        }
                        changed = result.changed;
                        return true;
                    });
            if(!replacement.success) {
                rollbackImport(QStringLiteral("rotationBodyPlanningReplacementRollback"));
                publishState(replacement.message, 5000);
                return { false, replacement.message };
            }
        }

        const RotationBodyControllerResult reload =
            reloadViewport(QStringLiteral("rotationBodyPlanningImport"));
        if(!reload.success) {
            rollbackImport(QStringLiteral("rotationBodyPlanningImportRollback"));
            publishState(reload.message, 5000);
            return reload;
        }
        previewAndFocus();
        publishState(aligned.message, 3000);
        return aligned;
    }

    RotationBodyControllerResult RotationBodyPlanningController::resumeDraft()
    {
        const DraftReadResult identityRead =
            RotationBodyPlanningDraftStore::read(m_context.document());
        if(identityRead.status == DraftReadStatus::NotFound) {
            return { true, {} };
        }
        if(!identityRead.usable() || !identityRead.draft) {
            return { false, QString::fromStdString(identityRead.message) };
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), identityRead.draft->objectId);
        if(object == nullptr) {
            return {
                false,
                QStringLiteral("The workpiece referenced by the saved planning draft no longer exists.")
            };
        }

        domain::PlanningResult<RotationBodyMeshLoad> loaded =
            RotationBodyMeshAdapter::load(m_context.projectSession(), *object);
        if(!loaded) {
            return { false, QString::fromStdString(loaded.error.message) };
        }
        const DraftReadResult currentRead = RotationBodyPlanningDraftStore::read(
            m_context.document(),
            loaded.value.sourceFingerprint,
            loaded.value.mesh.stableFingerprint());
        if(!currentRead.usable() || !currentRead.draft) {
            return { false, QString::fromStdString(currentRead.message) };
        }
        domain::PlanningResult<DraftRestoreDisposition> restored = m_session.restoreDraft(
            *currentRead.draft,
            std::move(loaded.value.mesh),
            loaded.value.sourceFingerprint);
        if(!restored) {
            return { false, QString::fromStdString(restored.error.message) };
        }
        const bool sourceChanged = currentRead.status == DraftReadStatus::SourceChanged;
        const TrajectoryWorkspaceReadResult trajectoryRead =
            RotationBodyTrajectoryProjectStore::readWorkspace(m_context.document());
        if(!sourceChanged && trajectoryRead.status != TrajectoryProjectReadStatus::NotFound) {
            if(!trajectoryRead.usable() || !trajectoryRead.draft) {
                return { false, QString::fromStdString(trajectoryRead.message) };
            }
            const domain::PlanningResult<void> trajectoryRestored =
                m_session.restoreTrajectoryDraft(*trajectoryRead.draft);
            if(!trajectoryRestored) {
                return { false, QString::fromStdString(trajectoryRestored.error.message) };
            }
        }
        m_session.setDefaultRapidOutputDirectory(defaultRapidOutputDirectory());
        return {
            true,
            sourceChanged
                ? QString::fromStdString(currentRead.message)
                : QStringLiteral("Rotation-body planning progress restored."),
            sourceChanged
                ? RotationBodyControllerNotice::DraftSourceChanged
                : RotationBodyControllerNotice::DraftRestored
        };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updatePlanningTransform(
        const Eigen::Isometry3d& transform)
    {
        const domain::PlanningResult<void> result = m_session.setPlanningFromMesh(transform);
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        previewAndFocus();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updatePlanningDelta(
        const domain::TransformComponents& delta)
    {
        const domain::PlanningResult<Eigen::Isometry3d> transform =
            domain::ModelTransformOperations::fromBaseline(
                m_session.automaticBaseline(),
                delta);
        if(!transform) {
            return domainFailure(transform.error);
        }
        return updatePlanningTransform(transform.value);
    }

    RotationBodyControllerResult RotationBodyPlanningController::flipModel()
    {
        const domain::TriangleMesh* mesh = m_session.mesh();
        if(mesh == nullptr) {
            return domainFailure({
                domain::PlanningErrorCode::EmptyMesh,
                "Load a model before flipping it."
            });
        }
        const domain::PlanningResult<Eigen::Isometry3d> flipped =
            domain::ModelTransformOperations::flip(
                *mesh,
                m_session.planningFromMesh(),
                m_session.directedRotaryAxisInMesh(),
                m_session.bottomAxisCenterInMesh());
        if(!flipped) {
            return domainFailure(flipped.error);
        }
        return updatePlanningTransform(flipped.value);
    }

    RotationBodyControllerResult RotationBodyPlanningController::resetModelTransform()
    {
        const domain::PlanningResult<Eigen::Isometry3d> reset =
            domain::ModelTransformOperations::reset(m_session.automaticBaseline());
        if(!reset) {
            return domainFailure(reset.error);
        }
        return updatePlanningTransform(reset.value);
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateBaseTransform(
        const Eigen::Isometry3d& transform)
    {
        const domain::PlanningResult<void> result = m_session.setBaseFromPlanning(transform);
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        if(m_session.publishFrame() == PublishFrame::BaseFrame) {
            previewAndFocus();
        }
        clearRapidPreview();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateBaseComponents(
        const domain::TransformComponents& components)
    {
        return updateBaseTransform(domain::makeTransform(components));
    }

    RotationBodyControllerResult RotationBodyPlanningController::confirmFrame()
    {
        const domain::PlanningResult<void> result = m_session.confirmFrame();
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        publishState(QStringLiteral("Rotation-body planning frame confirmed."), 3000);
        return { true, {} };
    }

    void RotationBodyPlanningController::setPublishFrame(PublishFrame frame)
    {
        m_session.setPublishFrame(frame);
        previewAndFocus();
        publishState();
    }

    RotationBodyControllerResult RotationBodyPlanningController::extractSection()
    {
        if(m_busy) {
            return busyFailure();
        }
        const domain::TriangleMesh* mesh = m_session.mesh();
        if(mesh == nullptr) {
            return domainFailure({
                domain::PlanningErrorCode::EmptyMesh,
                "Load and align a model before extracting its section."
            });
        }
        if(static_cast<int>(m_session.stage()) <
            static_cast<int>(domain::PlanningStage::FrameConfirmed)) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "Confirm the workpiece frame before extracting a section."
            });
        }

        const std::uint64_t generation = m_session.generation();
        domain::PlanningResult<domain::SectionContour> result =
            domain::YzSectionExtractor::extractTargetContour(
                *mesh,
                m_session.planningFromMesh(),
                m_session.parameters().section);
        if(!result) {
            return domainFailure(result.error, 5000);
        }
        if(!m_session.acceptSection(generation, std::move(result.value))) {
            return domainFailure({
                domain::PlanningErrorCode::SectionFailed,
                "The section result became stale before it could be applied."
            });
        }
        m_session.clearError();
        publishState(QStringLiteral("The YZ section was extracted."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::recognizeRegions()
    {
        if(m_busy) {
            return busyFailure();
        }
        if(!m_session.section()) {
            return domainFailure({
                domain::PlanningErrorCode::NoContour,
                "Extract a section before recognizing spray regions."
            });
        }
        const std::uint64_t generation = m_session.generation();
        domain::PlanningResult<domain::RegionAssignment> result =
            domain::ToothRegionRecognizer::recognize(
                *m_session.section(),
                m_session.parameters().region);
        if(!result) {
            return domainFailure(result.error, 5000);
        }
        if(!m_session.acceptAutomaticRegions(generation, std::move(result.value))) {
            return domainFailure({
                domain::PlanningErrorCode::InsufficientRegionData,
                "The recognition result became stale before it could be applied."
            });
        }
        m_session.clearError();
        publishState(QStringLiteral("Spray regions were recognized automatically."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::applyRegionOverride(
        const domain::YzRectangle& rectangle,
        domain::RegionLabel label)
    {
        const domain::PlanningResult<void> result =
            m_session.applyRegionOverride(rectangle, label);
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::undoRegionOverride()
    {
        if(!m_session.undoRegionOverride()) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "There is no region edit to undo."
            });
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::redoRegionOverride()
    {
        if(!m_session.redoRegionOverride()) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                "There is no region edit to redo."
            });
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::restoreAutomaticRegions()
    {
        if(!m_session.restoreAutomaticRegions()) {
            return domainFailure({
                domain::PlanningErrorCode::InsufficientRegionData,
                "Automatic regions are not available to restore."
            });
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    void RotationBodyPlanningController::setBoundaryMode(domain::BoundaryMode mode)
    {
        m_session.setBoundaryMode(mode);
        m_session.clearError();
        publishState();
    }

    RotationBodyControllerResult RotationBodyPlanningController::confirmSprayBoundary()
    {
        const std::optional<domain::RegionAssignment> regions = m_session.resolvedRegions();
        if(!m_session.section() || !regions) {
            return domainFailure({
                domain::PlanningErrorCode::InsufficientRegionData,
                "Recognize or define spray regions before confirming the boundary."
            });
        }
        const std::uint64_t generation = m_session.generation();
        domain::PlanningResult<domain::SprayBoundary> result =
            domain::SprayBoundaryBuilder::build(
                *m_session.section(),
                *regions,
                m_session.boundaryMode());
        if(!result) {
            return domainFailure(result.error, 5000);
        }
        if(!m_session.acceptBoundary(generation, std::move(result.value))) {
            return domainFailure({
                domain::PlanningErrorCode::InsufficientRegionData,
                "The spray boundary result became stale before it could be applied."
            });
        }
        m_session.clearError();
        publishState(QStringLiteral("The spray region boundary was confirmed."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::reopenBoundaryForEditing()
    {
        const domain::PlanningResult<void> result = m_session.reopenBoundaryForEditing();
        if(!result) {
            return domainFailure(result.error);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState(QStringLiteral("The spray boundary was reopened for region correction."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateTrajectoryParameters(
        const domain::TrajectoryGenerationParameters& parameters)
    {
        const domain::PlanningResult<void> result =
            m_session.setTrajectoryParameters(parameters);
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::generateCurrentTrajectory()
    {
        const domain::PlanningResult<void> result = m_session.generateCurrentTrajectory();
        if(!result) {
            return domainFailure(result.error, 5000);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState(QStringLiteral("Trajectory points were generated."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::swapCurrentTrajectoryDirection()
    {
        const domain::PlanningResult<void> result =
            m_session.swapCurrentTrajectoryDirection();
        if(!result) {
            return domainFailure(result.error);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState(QStringLiteral("Trajectory start and end were exchanged."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::setTrajectoryDisplayMode(
        domain::TrajectoryDisplayMode mode)
    {
        m_session.setTrajectoryDisplayMode(mode);
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::interpolateCurrentTrajectory(
        const std::vector<std::size_t>& selectedIndices,
        double intervalSeconds)
    {
        const domain::PlanningResult<void> result =
            m_session.interpolateCurrentTrajectory(selectedIndices, intervalSeconds);
        if(!result) {
            return domainFailure(result.error);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState(QStringLiteral("Trajectory points were interpolated."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::transformCurrentTrajectoryPoints(
        const std::vector<std::size_t>& selectedIndices,
        const domain::TransformComponents& delta)
    {
        const domain::PlanningResult<void> result =
            m_session.transformCurrentTrajectoryPoints(selectedIndices, delta);
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        publishState(QStringLiteral("Selected trajectory poses were adjusted."), 3000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::beginNewTrajectory()
    {
        const domain::PlanningResult<void> result = m_session.beginNewTrajectory();
        if(!result) {
            return domainFailure(result.error);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::loadTrajectoryForEditing(
        const std::string& passId)
    {
        const domain::PlanningResult<void> result =
            m_session.loadTrajectoryForEditing(passId);
        if(!result) {
            return domainFailure(result.error);
        }
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::saveCurrentTrajectoryToGroup()
    {
        const domain::PlanningResult<std::string> result =
            m_session.saveCurrentTrajectoryToGroup();
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_session.clearError();
        publishState(
            QStringLiteral("Trajectory saved to group as %1.")
                .arg(QString::fromStdString(result.value)),
            3000);
        return { true, {} };
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::exportTrajectoryGroupTextFile()
    {
        const domain::PlanningResult<std::string> formatted =
            domain::MergedTrajectoryTextExporter::format(
                m_session.makePublishedTrajectoryPlan());
        if(!formatted) {
            return domainFailure(formatted.error, 5000);
        }

        try {
            const std::filesystem::path outputDirectory =
                defaultMergedTrajectoryOutputDirectory();
            std::filesystem::create_directories(outputDirectory);
            const std::filesystem::path outputFile =
                nextMergedTrajectoryOutputFile(outputDirectory);
            std::ofstream stream(outputFile, std::ios::binary | std::ios::trunc);
            if(!stream) {
                throw std::runtime_error("The trajectory output file could not be opened.");
            }
            stream.write(
                formatted.value.data(),
                static_cast<std::streamsize>(formatted.value.size()));
            stream.close();
            if(!std::filesystem::is_regular_file(outputFile) ||
                std::filesystem::file_size(outputFile) != formatted.value.size()) {
                throw std::runtime_error("The trajectory output file could not be verified.");
            }
            const QString message = QStringLiteral("Trajectory group saved to %1.")
                .arg(QString::fromStdWString(outputFile.wstring()));
            publishState(message, 5000);
            return { true, message };
        } catch(const std::exception& exception) {
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                exception.what()
            }, 5000);
        }
    }

    RotationBodyControllerResult RotationBodyPlanningController::removeTrajectoryPass(
        const std::string& passId)
    {
        const domain::PlanningResult<void> result =
            m_session.removeTrajectoryPass(passId);
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_selectedTrajectoryPointIndices.clear();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::setTrajectoryPassVisible(
        const std::string& passId,
        bool visible)
    {
        const domain::PlanningResult<void> result =
            m_session.setTrajectoryPassVisible(passId, visible);
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::setTrajectoryTransitionAfter(
        const std::string& passId,
        double seconds)
    {
        const domain::PlanningResult<void> result =
            m_session.setTrajectoryTransitionAfter(passId, seconds);
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateCalibrationWorkspace(
        WorkpieceCalibrationWorkspace workspace)
    {
        const domain::PlanningResult<void> result =
            m_session.setCalibrationWorkspace(std::move(workspace));
        if(!result) {
            return domainFailure(result.error);
        }
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    void RotationBodyPlanningController::updateUiState(const RotationBodyUiState& state)
    {
        m_session.setUiState(state);
        publishState();
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateRapidSettings(
        const domain::RapidExportSettings& settings)
    {
        const domain::PlanningResult<void> result = m_session.setRapidSettings(settings);
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::updateRapidSequence(
        std::vector<domain::RapidSequenceEntry> sequence)
    {
        const domain::PlanningResult<void> result =
            m_session.setRapidSequence(std::move(sequence));
        if(!result) {
            return domainFailure(result.error);
        }
        clearRapidPreview();
        m_session.clearError();
        publishState();
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::generateAndSaveRapidModule()
    {
        domain::PlanningResult<domain::RapidModule> generated =
            domain::RapidModuleGenerator::generate(
                m_session.makePublishedTrajectoryPlan(),
                m_session.rapidSettings(),
                m_session.rapidSequence());
        if(!generated) {
            clearRapidPreview();
            m_rapidExportStatus = generated.error.message;
            return domainFailure(generated.error, 5000);
        }
        try {
            const std::filesystem::path outputFile = rapidOutputFile(m_session.rapidSettings());
            if(m_session.rapidSettings().outputDirectory.empty() || outputFile.filename().empty()) {
                throw std::runtime_error("Choose an ABB RAPID output directory and file name.");
            }
            std::filesystem::create_directories(outputFile.parent_path());
            std::ofstream stream(outputFile, std::ios::binary | std::ios::trunc);
            if(!stream) {
                throw std::runtime_error("The ABB RAPID output file could not be opened.");
            }
            stream.write(
                generated.value.code.data(),
                static_cast<std::streamsize>(generated.value.code.size()));
            if(!stream) {
                throw std::runtime_error("The ABB RAPID output file could not be written.");
            }
            stream.close();
            if(!std::filesystem::is_regular_file(outputFile) ||
                std::filesystem::file_size(outputFile) == 0) {
                throw std::runtime_error("The ABB RAPID output file could not be verified.");
            }
            m_rapidModulePreview = std::move(generated.value);
            m_selectedRapidPreviewStep = m_rapidModulePreview->previewSteps.empty()
                ? std::optional<std::size_t>()
                : std::optional<std::size_t>(0);
            m_rapidOutputFile = outputFile.u8string();
            m_rapidExportSucceeded = true;
            const QString message = QStringLiteral("ABB RAPID module saved to %1.")
                .arg(QString::fromStdWString(outputFile.wstring()));
            m_rapidExportStatus = message.toUtf8().toStdString();
            publishState(message, 5000);
            return { true, message };
        } catch(const std::exception& exception) {
            clearRapidPreview();
            m_rapidExportStatus = exception.what();
            return domainFailure({
                domain::PlanningErrorCode::InvalidArgument,
                exception.what()
            }, 5000);
        }
    }

    void RotationBodyPlanningController::setSelectedTrajectoryPointIndices(
        std::vector<std::size_t> indices)
    {
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
        if(m_selectedTrajectoryPointIndices == indices) {
            return;
        }
        m_selectedTrajectoryPointIndices = std::move(indices);
        rebuildViewportOverlay();
    }

    void RotationBodyPlanningController::setSelectedRapidPreviewStep(
        std::optional<std::size_t> index)
    {
        if(index && (!m_rapidModulePreview ||
            *index >= m_rapidModulePreview->previewSteps.size())) {
            index.reset();
        }
        if(m_selectedRapidPreviewStep == index) {
            return;
        }
        m_selectedRapidPreviewStep = index;
        rebuildViewportOverlay();
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::setCalibrationHelpersVisible(bool visible)
    {
        if(m_calibrationHelpersVisible == visible) {
            return { true, {} };
        }
        m_calibrationHelpersVisible = visible;
        return m_active ? rebuildViewportOverlay() : RotationBodyControllerResult{ true, {} };
    }

    RotationBodyControllerResult
    RotationBodyPlanningController::restoreViewportPresentation()
    {
        m_calibrationHelpersVisible = true;
        if(!m_active) {
            return clearViewportOverlay();
        }
        previewAndFocus();
        return rebuildViewportOverlay();
    }

    RotationBodyControllerResult RotationBodyPlanningController::rebuildViewportOverlay()
    {
        robot_qt_viewer::RobotQtViewerViewportServices* services =
            m_context.viewportServices();
        if(services == nullptr) {
            return { true, {} };
        }

        QString errorMessage;
        if(!services->clearLineOverlaysByOwner(overlayOwnerId, &errorMessage)) {
            return { false, errorMessage };
        }
        if(!m_active || !m_session.hasModel()) {
            return { true, {} };
        }

        const auto upsert = [&](const robot_qt_viewer::ViewportLineOverlay& overlay) {
            if(overlay.segments.empty()) {
                return true;
            }
            if(services->upsertLineOverlay(overlay, &errorMessage)) {
                return true;
            }
            services->clearLineOverlaysByOwner(overlayOwnerId);
            return false;
        };

        const double modelExtent = std::max(m_session.statistics().heightMeters,
                                            m_session.statistics().maximumDiameterMeters);
        const bool displayInBase = m_session.publishFrame() == PublishFrame::BaseFrame;
        const Eigen::Isometry3d displayFromPlanning =
            displayInBase ? m_session.baseFromPlanning() : Eigen::Isometry3d::Identity();
        const Eigen::Isometry3d displayFromBase =
            displayInBase ? Eigen::Isometry3d::Identity() : m_session.baseFromPlanning().inverse();
        const double planningAxisLength = std::max(0.02, std::min(0.08, modelExtent * 0.12));
        robot_qt_viewer::ViewportLineOverlay planningFrameOverlay;
        planningFrameOverlay.ownerId = overlayOwnerId;
        planningFrameOverlay.overlayId = QStringLiteral("planning-frame");
        appendPoseFrame(planningFrameOverlay, displayFromPlanning, planningAxisLength, true);
        if(!upsert(planningFrameOverlay)) {
            return { false, errorMessage };
        }

        robot_qt_viewer::ViewportLineOverlay trajectoryPathOverlay;
        trajectoryPathOverlay.ownerId = overlayOwnerId;
        trajectoryPathOverlay.overlayId = QStringLiteral("trajectory-paths");
        robot_qt_viewer::ViewportLineOverlay trajectoryFrameOverlay;
        trajectoryFrameOverlay.ownerId = overlayOwnerId;
        trajectoryFrameOverlay.overlayId = QStringLiteral("trajectory-frames");

        const domain::TrajectoryWorkspace& workspace = m_session.trajectoryWorkspace();
        const double pointAxisLength = std::max(0.002, std::min(0.006, modelExtent * 0.01));
        const auto appendTrajectory = [&trajectoryPathOverlay,
                                       &trajectoryFrameOverlay,
                                       pointAxisLength,
                                       &displayFromPlanning,
                                       displayMode = workspace.displayMode](
                                          const domain::PlannedTrajectory& trajectory,
                                          const std::set<std::size_t>& selectedIndices,
                                          bool current) {
            const std::vector<domain::TrajectoryPosePoint>& points =
                displayedTrajectoryPoints(trajectory, displayMode);
            appendPath(trajectoryPathOverlay,
                       points,
                       displayFromPlanning,
                       displayMode == domain::TrajectoryDisplayMode::RelativeHelical
                           ? (current ? overlayColor(255, 94, 196) : overlayColor(174, 105, 255))
                           : (current ? overlayColor(255, 214, 64) : overlayColor(49, 205, 224)));
            for(std::size_t index = 0; index < points.size(); ++index) {
                const bool selected = current && selectedIndices.count(index) != 0;
                appendPoseFrame(trajectoryFrameOverlay,
                                displayFromPlanning * points[index].planningFromTool,
                                selected ? pointAxisLength * 2.0 : pointAxisLength,
                                selected);
            }
        };
        for(const domain::TrajectoryPass& pass : workspace.group.passes) {
            if(!pass.visible ||
               (workspace.currentTrajectory && pass.id == workspace.editingPassId)) {
                continue;
            }
            appendTrajectory(pass.trajectory, {}, false);
        }
        if(workspace.currentTrajectory) {
            const std::set<std::size_t> selected(m_selectedTrajectoryPointIndices.begin(),
                                                 m_selectedTrajectoryPointIndices.end());
            appendTrajectory(*workspace.currentTrajectory, selected, true);
        }
        if(!upsert(trajectoryPathOverlay) || !upsert(trajectoryFrameOverlay)) {
            return { false, errorMessage };
        }

        const bool showSafetyPoint =
            std::any_of(m_session.rapidSequence().begin(),
                        m_session.rapidSequence().end(),
                        [](const domain::RapidSequenceEntry& entry) {
                            return entry.kind == domain::RapidSequenceEntryKind::SafetyPoint;
                        });
        if(showSafetyPoint) {
            const Eigen::Vector3d displayPoint =
                displayFromBase * m_session.rapidSettings().safetyPositionBaseMeters;
            robot_qt_viewer::ViewportLineOverlay safetyOverlay;
            safetyOverlay.ownerId = overlayOwnerId;
            safetyOverlay.overlayId = QStringLiteral("safety-point");
            appendCrossMarker(safetyOverlay,
                              displayPoint,
                              std::max(pointAxisLength * 2.5, 0.006),
                              overlayColor(255, 238, 88));
            appendCircle(safetyOverlay,
                         displayPoint,
                         Eigen::Vector3d::UnitZ(),
                         std::max(pointAxisLength * 3.5, 0.009),
                         overlayColor(255, 193, 7));
            if(!upsert(safetyOverlay)) {
                return { false, errorMessage };
            }
        }

        if(m_calibrationHelpersVisible) {
            const WorkpieceCalibrationWorkspace& calibration = m_session.calibrationWorkspace();
            robot_qt_viewer::ViewportLineOverlay calibrationPointOverlay;
            calibrationPointOverlay.ownerId = overlayOwnerId;
            calibrationPointOverlay.overlayId = QStringLiteral("calibration-points");
            const double calibrationMarkerRadius = std::max(pointAxisLength * 1.5, 0.003);
            const auto appendCalibrationRows = [&](const auto& rows,
                                                   const simulation_project::ColorDesc& color) {
                for(const CalibrationCoordinateRow& row : rows) {
                    const std::optional<Eigen::Vector3d> point = calibrationPoint(row);
                    if(point) {
                        appendCrossMarker(calibrationPointOverlay,
                                          displayFromBase * *point,
                                          calibrationMarkerRadius * 0.9,
                                          color);
                        appendCalibrationMarker(calibrationPointOverlay,
                                                displayFromBase * *point,
                                                calibrationMarkerRadius,
                                                color);
                    }
                }
            };
            if(calibration.showCylinder) {
                appendCalibrationRows(calibration.cylinderRows, overlayColor(255, 112, 46));
            }
            if(calibration.showCircle) {
                appendCalibrationRows(calibration.circleRows, overlayColor(255, 65, 86));
            }
            const auto appendReferencePoint = [&](const CalibrationCoordinateRow& row,
                                                  const simulation_project::ColorDesc& color) {
                const std::optional<Eigen::Vector3d> point = calibrationPoint(row);
                if(point) {
                    appendCalibrationMarker(calibrationPointOverlay,
                                            displayFromBase * *point,
                                            calibrationMarkerRadius * 1.35,
                                            color);
                }
            };
            appendReferencePoint(calibration.topReference, overlayColor(245, 245, 245));
            appendReferencePoint(calibration.yDirectionStart, overlayColor(72, 229, 121));
            appendReferencePoint(calibration.yDirectionEnd, overlayColor(160, 255, 188));
            const std::optional<Eigen::Vector3d> yStart =
                calibrationPoint(calibration.yDirectionStart);
            const std::optional<Eigen::Vector3d> yEnd = calibrationPoint(calibration.yDirectionEnd);
            if(yStart && yEnd) {
                appendLine(calibrationPointOverlay,
                           displayFromBase * *yStart,
                           displayFromBase * *yEnd,
                           overlayColor(72, 229, 121));
            }
            if(!upsert(calibrationPointOverlay)) {
                return { false, errorMessage };
            }

            robot_qt_viewer::ViewportLineOverlay calibrationFitOverlay;
            calibrationFitOverlay.ownerId = overlayOwnerId;
            calibrationFitOverlay.overlayId = QStringLiteral("calibration-fits");
            const auto appendFit = [&](const std::optional<domain::CalibrationAxisFit>& fit,
                                       const simulation_project::ColorDesc& color) {
                if(!fit) {
                    return;
                }
                const Eigen::Vector3d axisBase = fit->axisDirectionBase.normalized();
                double minimumProjection = 0.0;
                double maximumProjection = 0.0;
                bool haveProjection = false;
                for(const Eigen::Vector3d& point : fit->sourcePointsBaseMeters) {
                    const double projection = (point - fit->axisPointBaseMeters).dot(axisBase);
                    if(!haveProjection) {
                        minimumProjection = maximumProjection = projection;
                        haveProjection = true;
                    } else {
                        minimumProjection = std::min(minimumProjection, projection);
                        maximumProjection = std::max(maximumProjection, projection);
                    }
                }
                if(!haveProjection || maximumProjection - minimumProjection < 1.0e-6) {
                    minimumProjection = -fit->radiusMeters * 0.25;
                    maximumProjection = fit->radiusMeters * 0.25;
                }
                const Eigen::Vector3d centerA =
                    fit->axisPointBaseMeters + minimumProjection * axisBase;
                const Eigen::Vector3d centerB =
                    fit->axisPointBaseMeters + maximumProjection * axisBase;
                const Eigen::Vector3d displayAxis = displayFromBase.linear() * axisBase;
                appendCircle(calibrationFitOverlay,
                             displayFromBase * centerA,
                             displayAxis,
                             fit->radiusMeters,
                             color);
                appendCircle(calibrationFitOverlay,
                             displayFromBase * centerB,
                             displayAxis,
                             fit->radiusMeters,
                             color);
                appendLine(calibrationFitOverlay,
                           displayFromBase * centerA,
                           displayFromBase * centerB,
                           color);
                appendCalibrationMarker(calibrationFitOverlay,
                                        displayFromBase * fit->axisPointBaseMeters,
                                        std::max(pointAxisLength * 1.3, fit->radiusMeters * 0.08),
                                        color);
                appendCalibrationMarker(calibrationFitOverlay,
                                        displayFromBase * centerA,
                                        std::max(pointAxisLength, fit->radiusMeters * 0.05),
                                        color);
                appendCalibrationMarker(calibrationFitOverlay,
                                        displayFromBase * centerB,
                                        std::max(pointAxisLength, fit->radiusMeters * 0.05),
                                        color);
            };
            if(calibration.showCylinder) {
                appendFit(calibration.cylinderFit, overlayColor(0, 210, 255));
            }
            if(calibration.showCircle) {
                appendFit(calibration.circleFit, overlayColor(86, 255, 205));
            }
            if(calibration.frame) {
                const domain::WorkpieceFrameCalibration& frame = *calibration.frame;
                const Eigen::Vector3d displayAxis = displayFromBase.linear() * frame.zAxisBase;
                appendCircle(calibrationFitOverlay,
                             displayFromBase * frame.topCenterBaseMeters,
                             displayAxis,
                             frame.fittedRadiusMeters,
                             overlayColor(255, 214, 64));
                appendCircle(calibrationFitOverlay,
                             displayFromBase * frame.baseOriginBaseMeters,
                             displayAxis,
                             frame.fittedRadiusMeters,
                             overlayColor(255, 214, 64));
                appendLine(calibrationFitOverlay,
                           displayFromBase * frame.baseOriginBaseMeters,
                           displayFromBase * frame.topCenterBaseMeters,
                           overlayColor(255, 214, 64));
                appendPoseFrame(calibrationFitOverlay,
                                displayFromBase * frame.baseFromPlanning,
                                planningAxisLength,
                                true);
            }
            if(!upsert(calibrationFitOverlay)) {
                return { false, errorMessage };
            }
        }

        if(m_rapidModulePreview && !m_rapidModulePreview->previewSteps.empty()) {
            robot_qt_viewer::ViewportLineOverlay rapidPreviewOverlay;
            rapidPreviewOverlay.ownerId = overlayOwnerId;
            rapidPreviewOverlay.overlayId = QStringLiteral("rapid-preview");
            for(std::size_t index = 1; index < m_rapidModulePreview->previewSteps.size(); ++index) {
                const domain::RapidPreviewStep& step = m_rapidModulePreview->previewSteps[index];
                appendLine(
                    rapidPreviewOverlay,
                    displayFromBase *
                        m_rapidModulePreview->previewSteps[index - 1].baseFromTool.translation(),
                    displayFromBase * step.baseFromTool.translation(),
                    step.instruction == "MoveL" ? overlayColor(49, 205, 224)
                                                : overlayColor(255, 193, 7));
            }
            if(m_selectedRapidPreviewStep &&
               *m_selectedRapidPreviewStep < m_rapidModulePreview->previewSteps.size()) {
                const Eigen::Isometry3d displayFromTool =
                    displayFromBase *
                    m_rapidModulePreview->previewSteps[*m_selectedRapidPreviewStep].baseFromTool;
                appendCrossMarker(rapidPreviewOverlay,
                                  displayFromTool.translation(),
                                  std::max(pointAxisLength * 2.5, 0.006),
                                  overlayColor(255, 72, 72));
                appendPoseFrame(
                    rapidPreviewOverlay, displayFromTool, planningAxisLength * 0.6, true);
            }
            if(!upsert(rapidPreviewOverlay)) {
                return { false, errorMessage };
            }
        }

        if(!m_session.section()) {
            return { true, {} };
        }

        const domain::SectionContour& section = *m_session.section();
        if(section.points3d.size() != section.pointsYz.size() || section.points3d.size() < 2) {
            return { false,
                     QStringLiteral("The section contour cannot be rendered in the viewport.") };
        }

        const std::optional<domain::RegionAssignment> regions = m_session.resolvedRegions();
        const bool labelsMatch = regions && regions->matches(section);
        robot_qt_viewer::ViewportLineOverlay sectionOverlay;
        sectionOverlay.ownerId = overlayOwnerId;
        sectionOverlay.overlayId = QStringLiteral("section");
        sectionOverlay.segments.reserve(section.segmentCount());
        for(std::size_t segmentIndex = 0; segmentIndex < section.segmentCount(); ++segmentIndex) {
            const std::size_t nextIndex =
                segmentIndex + 1 < section.points3d.size() ? segmentIndex + 1 : 0;
            const domain::RegionLabel label = labelsMatch ? regions->segmentLabels[segmentIndex]
                                                          : domain::RegionLabel::Unclassified;
            robot_qt_viewer::ViewportLineSegment segment;
            segment.startWorld = worldPoint(displayFromPlanning * section.points3d[segmentIndex]);
            segment.endWorld = worldPoint(displayFromPlanning * section.points3d[nextIndex]);
            segment.color = regionOverlayColor(label);
            sectionOverlay.segments.push_back(std::move(segment));
        }
        if(!upsert(sectionOverlay)) {
            return { false, errorMessage };
        }

        if(m_session.boundary() && m_session.boundary()->polygonYz.size() >= 2) {
            const std::vector<Eigen::Vector2d>& points = m_session.boundary()->polygonYz;
            robot_qt_viewer::ViewportLineOverlay boundaryOverlay;
            boundaryOverlay.ownerId = overlayOwnerId;
            boundaryOverlay.overlayId = QStringLiteral("boundary");
            boundaryOverlay.segments.reserve(points.size());
            const simulation_project::ColorDesc yellow{ 1.0, 193.0 / 255.0, 7.0 / 255.0, 1.0 };
            for(std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
                const Eigen::Vector2d& start = points[pointIndex];
                const Eigen::Vector2d& end = points[(pointIndex + 1) % points.size()];
                robot_qt_viewer::ViewportLineSegment segment;
                segment.startWorld =
                    worldPoint(displayFromPlanning * Eigen::Vector3d(0.0, start.x(), start.y()));
                segment.endWorld =
                    worldPoint(displayFromPlanning * Eigen::Vector3d(0.0, end.x(), end.y()));
                segment.color = yellow;
                boundaryOverlay.segments.push_back(std::move(segment));
            }
            if(!upsert(boundaryOverlay)) {
                return { false, errorMessage };
            }
        }
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::clearViewportOverlay()
    {
        robot_qt_viewer::RobotQtViewerViewportServices* services =
            m_context.viewportServices();
        if(services == nullptr) {
            return { true, {} };
        }
        QString errorMessage;
        const bool success =
            services->clearLineOverlaysByOwner(overlayOwnerId, &errorMessage);
        return { success, success ? QString() : errorMessage };
    }

    RotationBodyControllerResult RotationBodyPlanningController::saveProgressAndExit()
    {
        if(m_busy) {
            return busyFailure();
        }
        if(!m_session.hasModel()) {
            return { false, QStringLiteral("There is no rotation-body planning model to save.") };
        }
        const RotationBodyPlanningDraft draft = m_session.makeDraft();
        const RotationBodyTrajectoryDraft trajectoryDraft =
            m_session.makeTrajectoryDraft();
        const domain::PublishedTrajectoryPlan publishedPlan =
            m_session.makePublishedTrajectoryPlan();
        const simulation_project::TransformDesc publishedTransform =
            transformDesc(m_session.publishedMeshTransform());
        const robot_qt_viewer::ProjectMutationResult mutation =
            m_context.documentController().mutateProject(
                QStringLiteral("rotationBodyPlanningSaveProgress"),
                robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                [&](simulation_project::ProjectDocumentService& service,
                    bool& changed,
                    std::string& error) {
                    if(!service.setSceneObjectTransform(draft.objectId, publishedTransform, &error)) {
                        return false;
                    }
                    changed = true;
                    try {
                        changed = RotationBodyPlanningDraftStore::write(service.document(), draft) || changed;
                        changed = RotationBodyTrajectoryProjectStore::writeWorkspace(
                            service.document(),
                            trajectoryDraft) || changed;
                        changed = RotationBodyTrajectoryProjectStore::writePublishedPlan(
                            service.document(),
                            publishedPlan) || changed;
                    } catch(const std::exception& exception) {
                        error = exception.what();
                        return false;
                    }
                    return true;
                });
        if(!mutation.success) {
            publishState(mutation.message, 5000);
            return { false, mutation.message };
        }

        // The document mutation is the commit boundary. Nothing after this point may
        // restore the entry snapshot, even if rebuilding the viewport fails.
        m_session.markProgressSaved();
        clearEntrySnapshot();
        m_active = false;
        clearPreviewAndFocus();

        const RotationBodyControllerResult reload =
            reloadViewport(QStringLiteral("rotationBodyPlanningSaveProgress"));
        if(!reload.success) {
            const QString message = QString("The project update was committed, but the viewport could not reload: %1")
                .arg(reload.message);
            publishState(message, 5000);
            return {
                true,
                message,
                RotationBodyControllerNotice::SavedViewportRefreshFailed
            };
        }
        const QString message = QStringLiteral("Rotation-body planning progress saved.");
        publishState(message, 4000);
        return { true, {} };
    }

    RotationBodyControllerResult RotationBodyPlanningController::discardAndExit()
    {
        cancelAsyncOperation();
        if(!m_hasEntrySnapshot) {
            clearPreviewAndFocus();
            m_session.clear();
            m_active = false;
            clearEntrySnapshot();
            publishState();
            return { true, {} };
        }
        m_context.documentController().restoreProjectSnapshot(
            QStringLiteral("rotationBodyPlanningDiscard"),
            m_entryDocument,
            m_entryDirty);
        m_active = false;
        clearEntrySnapshot();
        m_session.clear();
        clearPreviewAndFocus();
        const RotationBodyControllerResult reload =
            reloadViewport(QStringLiteral("rotationBodyPlanningDiscard"));
        const QString message = reload.success
            ? QStringLiteral("Rotation-body planning changes discarded.")
            : reload.message;
        publishState(message, reload.success ? 3000 : 5000);
        return { reload.success, message };
    }

    simulation_project::TransformDesc RotationBodyPlanningController::transformDesc(
        const Eigen::Isometry3d& transform)
    {
        simulation_project::TransformDesc result;
        result.x = transform.translation().x();
        result.y = transform.translation().y();
        result.z = transform.translation().z();
        const Eigen::Vector3d euler = transform.linear().eulerAngles(2, 1, 0);
        result.yaw = euler[0];
        result.pitch = euler[1];
        result.roll = euler[2];
        return result;
    }

    const simulation_project::SceneObjectDesc* RotationBodyPlanningController::findObject(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        const auto found = std::find_if(
            document.objects.begin(),
            document.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return object.id == objectId;
            });
        return found == document.objects.end() ? nullptr : &*found;
    }

    RotationBodyControllerResult RotationBodyPlanningController::alignImportedObject(
        const simulation_project::SceneObjectDesc& object,
        const RotationBodyImportOptions& options)
    {
        domain::PlanningResult<RotationBodyMeshLoad> loaded =
            RotationBodyMeshAdapter::load(m_context.projectSession(), object);
        if(!loaded) {
            return { false, QString::fromStdString(loaded.error.message) };
        }

        domain::PlanningResult<domain::AlignmentResult> alignment =
            solveAlignment(loaded.value.mesh, options);
        if(!alignment) {
            return { false, QString::fromStdString(alignment.error.message) };
        }

        const domain::PlanningResult<void> loadedSession = m_session.loadModel(
            object.id,
            object.sourcePath,
            loaded.value.sourceFingerprint,
            std::move(loaded.value.mesh),
            alignment.value,
            options.objectType,
            options.originalRotationAxis,
            options.toothOutwardAxis,
            options.motherMaximumDiameterMeters);
        if(!loadedSession) {
            return { false, QString::fromStdString(loadedSession.error.message) };
        }
        return {
            true,
            QStringLiteral("The model was loaded and aligned automatically.")
        };
    }

    RotationBodyControllerResult RotationBodyPlanningController::reloadViewport(const QString& sourceId)
    {
        robot_qt_viewer::ViewportReloadWorkflowController reload(m_context);
        const robot_qt_viewer::ViewportReloadWorkflowResult result = reload.reload(sourceId);
        RotationBodyControllerResult controllerResult{
            result.success,
            result.success ? QString() : result.errorMessage
        };
        if(controllerResult.success) {
            const RotationBodyControllerResult presentationResult =
                restoreViewportPresentation();
            if(!presentationResult.success) {
                return presentationResult;
            }
        }
        return controllerResult;
    }

    RotationBodyControllerResult RotationBodyPlanningController::domainFailure(
        const domain::PlanningError& error,
        int timeoutMs)
    {
        m_session.setError(error);
        const QString message = QString::fromStdString(error.message);
        publishState(message, timeoutMs);
        return { false, message };
    }

    RotationBodyControllerResult RotationBodyPlanningController::busyFailure() const
    {
        return {
            false,
            QStringLiteral("A rotation-body planning calculation is already running.")
        };
    }

    std::uint64_t RotationBodyPlanningController::beginAsyncOperation()
    {
        m_busy = true;
        const std::uint64_t token = ++m_asyncOperationToken;
        publishState();
        return token;
    }

    bool RotationBodyPlanningController::isCurrentAsyncOperation(
        std::uint64_t token) const noexcept
    {
        return m_active && m_busy && token == m_asyncOperationToken;
    }

    void RotationBodyPlanningController::finishAsyncOperation(
        const RotationBodyControllerResult& result,
        const QString& statusMessage,
        int timeoutMs)
    {
        m_busy = false;
        publishState(statusMessage, timeoutMs);
        emit asyncOperationFinished(result);
    }

    void RotationBodyPlanningController::cancelAsyncOperation() noexcept
    {
        ++m_asyncOperationToken;
        m_busy = false;
    }

    void RotationBodyPlanningController::completeAsyncImport(
        std::uint64_t token,
        const std::shared_ptr<AsyncImportContext>& context,
        const std::shared_ptr<AsyncImportResult>& result)
    {
        if(!isCurrentAsyncOperation(token)) {
            return;
        }

        const auto rollback = [this, &context](const domain::PlanningError& error) {
            robot_qt_viewer::SceneEntityWorkflowController workflow(m_context);
            workflow.restoreImportState(context->imported);
            m_session = context->previousSession;
            reloadViewport(QStringLiteral("rotationBodyPlanningAsyncImportRollback"));
            m_session.setError(error);
            const QString message = QString::fromStdString(error.message);
            finishAsyncOperation({ false, message }, message, 5000);
        };

        if(m_session.generation() != context->inputGeneration) {
            rollback({
                domain::PlanningErrorCode::AlignmentFailed,
                "The model alignment result became stale before it could be applied."
            });
            return;
        }
        if(findObject(m_context.document(), context->object.id) == nullptr) {
            rollback({
                domain::PlanningErrorCode::EmptyMesh,
                "The imported workpiece was removed before alignment completed."
            });
            return;
        }
        if(!result->loaded) {
            rollback(result->loaded.error);
            return;
        }
        if(!result->alignment) {
            rollback(result->alignment.error);
            return;
        }

        const domain::PlanningResult<void> loadedSession = m_session.loadModel(
            context->object.id,
            context->object.sourcePath,
            result->loaded.value.sourceFingerprint,
            std::move(result->loaded.value.mesh),
            result->alignment.value,
            context->options.objectType,
            context->options.originalRotationAxis,
            context->options.toothOutwardAxis,
            context->options.motherMaximumDiameterMeters);
        if(!loadedSession) {
            rollback(loadedSession.error);
            return;
        }

        if(context->previousSession.hasModel() &&
            context->previousSession.objectId() != m_session.objectId()) {
            const robot_qt_viewer::ProjectMutationResult replacement =
                m_context.documentController().mutateProject(
                    QStringLiteral("rotationBodyPlanningAsyncReplaceWorkpiece"),
                    robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                    [&](simulation_project::ProjectDocumentService& service,
                        bool& changed,
                        std::string& error) {
                        const PlanningObjectReplacementResult replacementResult =
                            RotationBodyPlanningDocumentOperations::replacePlanningObject(
                                service,
                                context->previousSession.objectId(),
                                m_session.objectId());
                        if(!replacementResult.success) {
                            error = replacementResult.error;
                            return false;
                        }
                        changed = replacementResult.changed;
                        return true;
                    });
            if(!replacement.success) {
                rollback({
                    domain::PlanningErrorCode::InvalidArgument,
                    replacement.message.toStdString()
                });
                return;
            }
        }

        const RotationBodyControllerResult reload =
            reloadViewport(QStringLiteral("rotationBodyPlanningAsyncImport"));
        if(!reload.success) {
            rollback({
                domain::PlanningErrorCode::InvalidArgument,
                reload.message.toStdString()
            });
            return;
        }
        previewAndFocus();
        m_session.clearError();
        const QString message =
            QStringLiteral("The model was loaded and aligned automatically.");
        finishAsyncOperation({ true, {} }, message, 3000);
    }

    void RotationBodyPlanningController::completeAsyncSection(
        std::uint64_t token,
        std::uint64_t inputGeneration,
        domain::PlanningResult<domain::SectionContour> result)
    {
        if(!isCurrentAsyncOperation(token)) {
            return;
        }
        if(!result) {
            m_session.setError(result.error);
            const QString message = QString::fromStdString(result.error.message);
            finishAsyncOperation({ false, message }, message, 5000);
            return;
        }
        if(!m_session.acceptSection(inputGeneration, std::move(result.value))) {
            const domain::PlanningError error{
                domain::PlanningErrorCode::SectionFailed,
                "The section result became stale before it could be applied."
            };
            m_session.setError(error);
            const QString message = QString::fromStdString(error.message);
            finishAsyncOperation({ false, message }, message, 5000);
            return;
        }
        m_session.clearError();
        const QString message = QStringLiteral("The YZ section was extracted.");
        finishAsyncOperation({ true, {} }, message, 3000);
    }

    void RotationBodyPlanningController::completeAsyncRecognition(
        std::uint64_t token,
        std::uint64_t inputGeneration,
        domain::PlanningResult<domain::RegionAssignment> result)
    {
        if(!isCurrentAsyncOperation(token)) {
            return;
        }
        if(!result) {
            m_session.setError(result.error);
            const QString message = QString::fromStdString(result.error.message);
            finishAsyncOperation({ false, message }, message, 5000);
            return;
        }
        if(!m_session.acceptAutomaticRegions(inputGeneration, std::move(result.value))) {
            const domain::PlanningError error{
                domain::PlanningErrorCode::InsufficientRegionData,
                "The recognition result became stale before it could be applied."
            };
            m_session.setError(error);
            const QString message = QString::fromStdString(error.message);
            finishAsyncOperation({ false, message }, message, 5000);
            return;
        }
        m_session.clearError();
        const QString message = QStringLiteral(
            "Spray regions were recognized automatically.");
        finishAsyncOperation({ true, {} }, message, 3000);
    }

    void RotationBodyPlanningController::previewAndFocus()
    {
        robot_qt_viewer::RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || !m_session.hasModel()) {
            return;
        }
        const QString objectId = QString::fromStdString(m_session.objectId());
        services->previewSceneObjectTransform(
            objectId,
            transformDesc(m_session.publishedMeshTransform()));
        services->focusObjectFrameObject(
            objectId,
            robot_qt_viewer::RobotQtViewerObjectFocusView::Isometric);
    }

    void RotationBodyPlanningController::clearPreviewAndFocus()
    {
        clearViewportOverlay();
        if(robot_qt_viewer::RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->clearObjectFrameObjectFocus();
        }
        m_context.viewportPreviewState().clearTaskPreview(
            QStringLiteral("rotationBodyPlanningClearPreview"));
    }

    void RotationBodyPlanningController::clearEntrySnapshot()
    {
        m_entryDocument = {};
        m_entryDirty = false;
        m_hasEntrySnapshot = false;
    }

    void RotationBodyPlanningController::clearRapidPreview()
    {
        m_rapidModulePreview.reset();
        m_selectedRapidPreviewStep.reset();
        m_rapidOutputFile.clear();
        m_rapidExportStatus.clear();
        m_rapidExportSucceeded = false;
    }

    void RotationBodyPlanningController::publishState(const QString& message, int timeoutMs)
    {
        rebuildViewportOverlay();
        emit stateChanged();
        if(!message.isEmpty()) {
            emit statusMessageRequested(message, timeoutMs);
        }
    }
}
