#pragma once

#include "../Models/RotationBodyPlanningSession.h"
#include "../Models/RotationBodyPlanningViewModel.h"

#include <SimulationProject/ProjectDocument.h>

#include <QObject>
#include <QString>
#include <QThreadPool>

#include <filesystem>
#include <memory>
#include <vector>

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;
}

namespace smrobot::workbench::spray::rotationbody
{
    enum class RotationBodyControllerNotice
    {
        None,
        DraftRestored,
        DraftSourceChanged,
        SavedViewportRefreshFailed
    };

    struct RotationBodyControllerResult
    {
        bool success{ false };
        QString message;
        RotationBodyControllerNotice notice{ RotationBodyControllerNotice::None };
    };

    class RotationBodyPlanningController : public QObject
    {
        Q_OBJECT

    public:
        explicit RotationBodyPlanningController(
            robot_qt_viewer::RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);
        ~RotationBodyPlanningController() override;

        RotationBodyPlanningSession& session() noexcept;
        const RotationBodyPlanningSession& session() const noexcept;
        RotationBodyPlanningViewModel viewModel() const;

        RotationBodyControllerResult activate();
        RotationBodyControllerResult deactivate();
        bool isActive() const noexcept;
        bool isBusy() const noexcept;
        bool hasPendingChanges() const noexcept;

        RotationBodyControllerResult startImportModel(
            const std::filesystem::path& sourcePath,
            const RotationBodyImportOptions& options);
        RotationBodyControllerResult startExtractSection();
        RotationBodyControllerResult startRecognizeRegions();

        RotationBodyControllerResult importModel(
            const std::filesystem::path& sourcePath,
            const RotationBodyImportOptions& options);
        RotationBodyControllerResult resumeDraft();
        RotationBodyControllerResult updatePlanningTransform(const Eigen::Isometry3d& transform);
        RotationBodyControllerResult updatePlanningDelta(
            const domain::TransformComponents& delta);
        RotationBodyControllerResult flipModel();
        RotationBodyControllerResult resetModelTransform();
        RotationBodyControllerResult updateBaseTransform(const Eigen::Isometry3d& transform);
        RotationBodyControllerResult updateBaseComponents(
            const domain::TransformComponents& components);
        RotationBodyControllerResult confirmFrame();
        void setPublishFrame(PublishFrame frame);
        RotationBodyControllerResult extractSection();
        RotationBodyControllerResult recognizeRegions();
        RotationBodyControllerResult applyRegionOverride(
            const domain::YzRectangle& rectangle,
            domain::RegionLabel label);
        RotationBodyControllerResult undoRegionOverride();
        RotationBodyControllerResult redoRegionOverride();
        RotationBodyControllerResult restoreAutomaticRegions();
        void setBoundaryMode(domain::BoundaryMode mode);
        RotationBodyControllerResult confirmSprayBoundary();
        RotationBodyControllerResult reopenBoundaryForEditing();
        RotationBodyControllerResult updateTrajectoryParameters(
            const domain::TrajectoryGenerationParameters& parameters);
        RotationBodyControllerResult generateCurrentTrajectory();
        RotationBodyControllerResult swapCurrentTrajectoryDirection();
        RotationBodyControllerResult setTrajectoryDisplayMode(
            domain::TrajectoryDisplayMode mode);
        RotationBodyControllerResult interpolateCurrentTrajectory(
            const std::vector<std::size_t>& selectedIndices,
            double intervalSeconds);
        RotationBodyControllerResult transformCurrentTrajectoryPoints(
            const std::vector<std::size_t>& selectedIndices,
            const domain::TransformComponents& delta);
        RotationBodyControllerResult beginNewTrajectory();
        RotationBodyControllerResult loadTrajectoryForEditing(const std::string& passId);
        RotationBodyControllerResult saveCurrentTrajectoryToGroup();
        RotationBodyControllerResult exportTrajectoryGroupTextFile();
        RotationBodyControllerResult removeTrajectoryPass(const std::string& passId);
        RotationBodyControllerResult setTrajectoryPassVisible(
            const std::string& passId,
            bool visible);
        RotationBodyControllerResult setTrajectoryTransitionAfter(
            const std::string& passId,
            double seconds);
        RotationBodyControllerResult updateRapidSettings(
            const domain::RapidExportSettings& settings);
        RotationBodyControllerResult updateRapidSequence(
            std::vector<domain::RapidSequenceEntry> sequence);
        RotationBodyControllerResult updateCalibrationWorkspace(
            WorkpieceCalibrationWorkspace workspace);
        void updateUiState(const RotationBodyUiState& state);
        RotationBodyControllerResult generateAndSaveRapidModule();
        RotationBodyControllerResult loadCurrentTrajectoryForPostProcessing(
            const std::string& robotId);
        RotationBodyControllerResult generatePostProcessedProgram(
            const std::string& templateName,
            const std::string& robotId,
            const std::string& programName,
            const std::string& outputDirectory);
        void setSelectedTrajectoryPointIndices(std::vector<std::size_t> indices);
        void setSelectedRapidPreviewStep(std::optional<std::size_t> index);
        // Presentation-only visibility for calibration helpers while the main
        // viewer is focused on a robot/base-coordinate selection.
        RotationBodyControllerResult setCalibrationHelpersVisible(bool visible);
        RotationBodyControllerResult restoreViewportPresentation();
        RotationBodyControllerResult rebuildViewportOverlay();
        RotationBodyControllerResult clearViewportOverlay();
        RotationBodyControllerResult saveProgressAndExit();
        RotationBodyControllerResult discardAndExit();

    signals:
        void stateChanged();
        void statusMessageRequested(const QString& message, int timeoutMs);
        void asyncOperationFinished(const RotationBodyControllerResult& result);

    private:
        struct AsyncImportContext;
        struct AsyncImportResult;
        struct PostProcessingState;

        static simulation_project::TransformDesc transformDesc(const Eigen::Isometry3d& transform);
        static const simulation_project::SceneObjectDesc* findObject(
            const simulation_project::ProjectDocument& document,
            const std::string& objectId);
        RotationBodyControllerResult alignImportedObject(
            const simulation_project::SceneObjectDesc& object,
            const RotationBodyImportOptions& options);
        RotationBodyControllerResult reloadViewport(const QString& sourceId);
        RotationBodyControllerResult domainFailure(
            const domain::PlanningError& error,
            int timeoutMs = 4000);
        RotationBodyControllerResult busyFailure() const;
        std::uint64_t beginAsyncOperation();
        bool isCurrentAsyncOperation(std::uint64_t token) const noexcept;
        void finishAsyncOperation(
            const RotationBodyControllerResult& result,
            const QString& statusMessage = {},
            int timeoutMs = 0);
        void cancelAsyncOperation() noexcept;
        void completeAsyncImport(
            std::uint64_t token,
            const std::shared_ptr<AsyncImportContext>& context,
            const std::shared_ptr<AsyncImportResult>& result);
        void completeAsyncSection(
            std::uint64_t token,
            std::uint64_t inputGeneration,
            domain::PlanningResult<domain::SectionContour> result);
        void completeAsyncRecognition(
            std::uint64_t token,
            std::uint64_t inputGeneration,
            domain::PlanningResult<domain::RegionAssignment> result);
        void previewAndFocus();
        void clearPreviewAndFocus();
        void clearEntrySnapshot();
        void clearRapidPreview();
        void publishState(const QString& message = {}, int timeoutMs = 0);

        robot_qt_viewer::RobotQtViewerDocumentContext& m_context;
        RotationBodyPlanningSession m_session;
        simulation_project::ProjectDocument m_entryDocument;
        bool m_entryDirty{ false };
        bool m_hasEntrySnapshot{ false };
        bool m_active{ false };
        bool m_busy{ false };
        std::uint64_t m_asyncOperationToken{ 0 };
        QThreadPool m_workerPool;
        std::vector<std::size_t> m_selectedTrajectoryPointIndices;
        std::optional<domain::RapidModule> m_rapidModulePreview;
        std::optional<std::size_t> m_selectedRapidPreviewStep;
        bool m_calibrationHelpersVisible{ true };
        std::string m_rapidOutputFile;
        std::string m_rapidExportStatus;
        bool m_rapidExportSucceeded{ false };
        std::unique_ptr<PostProcessingState> m_postProcessingState;
    };
}
