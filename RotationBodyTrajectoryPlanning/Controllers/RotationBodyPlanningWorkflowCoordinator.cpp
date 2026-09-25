#include "RotationBodyPlanningWorkflowCoordinator.h"

#include "../Models/RotationBodyPlanningTranslations.h"
#include "../Widgets/RotationBodyPlanningLeftPanel.h"
#include "../Widgets/RotationBodyPlanningRightPanel.h"
#include "../Widgets/SectionView.h"

#include <filesystem>

namespace smrobot::workbench::spray::rotationbody
{
    RotationBodyPlanningWorkflowCoordinator::RotationBodyPlanningWorkflowCoordinator(
        RotationBodyPlanningController& controller,
        RotationBodyPlanningLeftPanel& leftPanel,
        RotationBodyPlanningRightPanel& rightPanel,
        QObject* parent)
        : QObject(parent)
        , m_controller(controller)
        , m_leftPanel(leftPanel)
        , m_rightPanel(rightPanel)
    {
        bindPanelSignals();
        connect(
            &m_controller,
            &RotationBodyPlanningController::stateChanged,
            this,
            &RotationBodyPlanningWorkflowCoordinator::refresh);
        connect(
            &m_controller,
            &RotationBodyPlanningController::asyncOperationFinished,
            this,
            [this](const RotationBodyControllerResult& result) {
                reportResult(result);
            });
        setCurrentWorkflow(m_currentWorkflow);
        refresh();
    }

    RotationBodyControllerResult RotationBodyPlanningWorkflowCoordinator::activate()
    {
        const RotationBodyControllerResult result = m_controller.activate();
        if(result.success) {
            const RotationBodyUiState restored = m_controller.viewModel().uiState;
            m_restoringUiState = true;
            setCurrentWorkflow(restored.workflow);
            setMainViewMode(restored.mainViewMode);
            m_rightPanel.setCurrentRightWorkflow(restored.rightWorkflow);
            m_restoringUiState = false;
            persistUiState();
        }
        refresh();
        const char* statusKey = nullptr;
        if(result.notice == RotationBodyControllerNotice::DraftRestored) {
            statusKey = "status.restored";
        } else if(result.notice == RotationBodyControllerNotice::DraftSourceChanged) {
            statusKey = "status.restored_source_changed";
        }
        reportResult(result, statusKey);
        return result;
    }

    RotationBodyControllerResult RotationBodyPlanningWorkflowCoordinator::deactivate()
    {
        const RotationBodyControllerResult result = m_controller.deactivate();
        refresh();
        reportResult(result, result.success ? "status.discarded" : nullptr);
        return result;
    }

    void RotationBodyPlanningWorkflowCoordinator::setLanguageCode(const QString& languageCode)
    {
        m_languageCode =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        m_leftPanel.setLanguageCode(m_languageCode);
        m_rightPanel.setLanguageCode(m_languageCode);
        if(m_mainSectionView) {
            m_mainSectionView->setLanguageCode(m_languageCode);
        }
        refresh();
    }

    QString RotationBodyPlanningWorkflowCoordinator::languageCode() const
    {
        return m_languageCode;
    }

    void RotationBodyPlanningWorkflowCoordinator::setImportDirectory(const QString& directory)
    {
        m_leftPanel.setImportDirectory(directory);
    }

    void RotationBodyPlanningWorkflowCoordinator::setMainSectionView(SectionView* sectionView)
    {
        disconnect(m_mainRectangleConnection);
        m_mainSectionView = sectionView;
        if(!m_mainSectionView) {
            return;
        }
        m_mainSectionView->setLanguageCode(m_languageCode);
        m_mainSectionView->setActiveRegionLabel(m_activeLabel);
        m_mainSectionView->setSnapshot(m_controller.viewModel().sectionView);
        m_mainRectangleConnection = connect(
            m_mainSectionView,
            &SectionView::rectangleSelected,
            this,
            [this](const domain::YzRectangle& rectangle, domain::RegionLabel label) {
                reportResult(m_controller.applyRegionOverride(rectangle, label));
            });
    }

    SectionView* RotationBodyPlanningWorkflowCoordinator::mainSectionView() const noexcept
    {
        return m_mainSectionView;
    }

    void RotationBodyPlanningWorkflowCoordinator::setCurrentWorkflow(
        RotationBodyWorkflow workflow)
    {
        const bool changed = m_currentWorkflow != workflow;
        m_currentWorkflow = workflow;
        m_leftPanel.setCurrentWorkflow(workflow);
        m_rightPanel.setCurrentWorkflow(workflow);
        emit rightPanelVisibilityChanged(m_rightPanel.hasVisibleContent());
        if(workflow == RotationBodyWorkflow::ModelTransform &&
            m_mainViewMode != RotationBodyMainViewMode::Scene3d) {
            setMainViewMode(RotationBodyMainViewMode::Scene3d);
        }
        if(changed) {
            emit workflowChanged(workflow);
        }
        persistUiState();
    }

    RotationBodyWorkflow RotationBodyPlanningWorkflowCoordinator::currentWorkflow() const noexcept
    {
        return m_currentWorkflow;
    }

    void RotationBodyPlanningWorkflowCoordinator::setMainViewMode(
        RotationBodyMainViewMode mode)
    {
        if(mode == RotationBodyMainViewMode::Section &&
            !m_controller.viewModel().sectionView.section) {
            mode = RotationBodyMainViewMode::Scene3d;
        }
        const bool changed = m_mainViewMode != mode;
        m_mainViewMode = mode;
        m_leftPanel.setMainViewMode(mode);
        m_rightPanel.setMainViewMode(mode);
        if(changed) {
            emit mainViewModeChanged(mode);
        }
        persistUiState();
    }

    RotationBodyMainViewMode RotationBodyPlanningWorkflowCoordinator::mainViewMode() const noexcept
    {
        return m_mainViewMode;
    }

    RotationBodyControllerResult
    RotationBodyPlanningWorkflowCoordinator::setCalibrationHelpersVisible(bool visible)
    {
        const RotationBodyControllerResult result =
            m_controller.setCalibrationHelpersVisible(visible);
        if(!result.success) {
            emit statusMessageRequested(RotationBodyPlanningTranslations::text(
                                            m_languageCode, "error.viewport_presentation"),
                                        6000);
        }
        return result;
    }

    RotationBodyControllerResult
    RotationBodyPlanningWorkflowCoordinator::restoreViewportPresentation()
    {
        const RotationBodyControllerResult result = m_controller.restoreViewportPresentation();
        refresh();
        if(!result.success) {
            emit statusMessageRequested(RotationBodyPlanningTranslations::text(
                                            m_languageCode, "error.viewport_presentation"),
                                        6000);
        }
        return result;
    }

    RotationBodyControllerResult
    RotationBodyPlanningWorkflowCoordinator::rebuildViewportOverlay()
    {
        return m_controller.rebuildViewportOverlay();
    }

    RotationBodyControllerResult
    RotationBodyPlanningWorkflowCoordinator::clearViewportOverlay()
    {
        return m_controller.clearViewportOverlay();
    }

    void RotationBodyPlanningWorkflowCoordinator::refresh()
    {
        const RotationBodyPlanningViewModel viewModel = m_controller.viewModel();
        m_leftPanel.setViewModel(viewModel);
        m_rightPanel.setViewModel(viewModel);
        if(m_mainSectionView) {
            m_mainSectionView->setSnapshot(viewModel.sectionView);
            m_mainSectionView->setActiveRegionLabel(m_activeLabel);
            m_mainSectionView->setEnabled(!viewModel.isBusy);
        }
        if(!viewModel.sectionView.section &&
            m_mainViewMode == RotationBodyMainViewMode::Section) {
            setMainViewMode(RotationBodyMainViewMode::Scene3d);
        }
        emit viewModelChanged(viewModel);
    }

    void RotationBodyPlanningWorkflowCoordinator::reportResult(
        const RotationBodyControllerResult& result,
        const char* successTranslationKey,
        int timeoutMs)
    {
        refresh();
        QString message;
        if(result.success) {
            message = successTranslationKey != nullptr
                ? RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    successTranslationKey)
                : (result.message.isEmpty()
                    ? localizedCurrentStatus()
                    : result.message);
        } else {
            message = RotationBodyPlanningTranslations::errorText(
                m_languageCode,
                m_controller.viewModel().errorCode);
            if(message.isEmpty()) {
                message = RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "error.generic");
            }
        }
        if(!message.isEmpty()) {
            emit statusMessageRequested(message, timeoutMs);
        }
    }

    QString RotationBodyPlanningWorkflowCoordinator::localizedCurrentStatus() const
    {
        const RotationBodyPlanningViewModel viewModel = m_controller.viewModel();
        if(viewModel.errorCode != domain::PlanningErrorCode::None) {
            return RotationBodyPlanningTranslations::errorText(
                m_languageCode,
                viewModel.errorCode);
        }
        return RotationBodyPlanningTranslations::stageName(
            m_languageCode,
            viewModel.stage);
    }

    void RotationBodyPlanningWorkflowCoordinator::persistUiState()
    {
        if(m_restoringUiState || !m_controller.isActive()) {
            return;
        }
        RotationBodyUiState state;
        state.workflow = m_currentWorkflow;
        state.mainViewMode = m_mainViewMode;
        state.rightWorkflow = m_rightPanel.currentRightWorkflow();
        m_controller.updateUiState(state);
    }

    void RotationBodyPlanningWorkflowCoordinator::bindPanelSignals()
    {
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::rightWorkflowChanged,
            this,
            [this]() { persistUiState(); });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::workflowChanged,
            this,
            &RotationBodyPlanningWorkflowCoordinator::setCurrentWorkflow);
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::returnToPlanningWorkpieceRequested,
            this,
            [this]() {
                setMainViewMode(RotationBodyMainViewMode::Scene3d);
                const RotationBodyControllerResult result = restoreViewportPresentation();
                reportResult(
                    result,
                    result.success ? "status.returned_to_workpiece" : nullptr,
                    3000);
            });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::importRequested,
            this,
            [this](const QString& sourcePath, const RotationBodyImportOptions& options) {
                const RotationBodyControllerResult result = m_controller.startImportModel(
                    std::filesystem::path(sourcePath.toStdWString()),
                    options);
                if(!result.success) {
                    reportResult(result);
                }
            });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::planningDeltaEdited,
            this,
            [this](const domain::TransformComponents& components) {
                reportResult(m_controller.updatePlanningDelta(components), nullptr, 2500);
            });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::baseTransformEdited,
            this,
            [this](const domain::TransformComponents& components) {
                reportResult(m_controller.updateBaseComponents(components), nullptr, 2500);
            });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::flipRequested, this, [this]() {
            reportResult(m_controller.flipModel(), "status.flipped");
        });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::resetRequested, this, [this]() {
            reportResult(m_controller.resetModelTransform(), "status.alignment_reset");
        });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::confirmFrameRequested, this, [this]() {
            reportResult(m_controller.confirmFrame());
        });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::saveProgressAndExitRequested,
            this,
            [this]() {
                const RotationBodyControllerResult result = m_controller.saveProgressAndExit();
                const char* statusKey = nullptr;
                if(result.success) {
                    statusKey = result.notice ==
                        RotationBodyControllerNotice::SavedViewportRefreshFailed
                        ? "status.saved_view_refresh_failed"
                        : "status.saved";
                }
                reportResult(result, statusKey);
                if(result.success) {
                    emit exitCompleted(true);
                }
            });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::discardAndExitRequested,
            this,
            [this]() {
                const RotationBodyControllerResult result = m_controller.discardAndExit();
                reportResult(result, result.success ? "status.discarded" : nullptr);
                if(!m_controller.isActive()) {
                    emit exitCompleted(false);
                }
            });

        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::mainViewModeChanged,
            this,
            &RotationBodyPlanningWorkflowCoordinator::setMainViewMode);
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::extractSectionRequested, this, [this]() {
            const RotationBodyControllerResult result = m_controller.startExtractSection();
            if(!result.success) {
                reportResult(result);
            }
        });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::recognizeRegionsRequested, this, [this]() {
            const RotationBodyControllerResult result = m_controller.startRecognizeRegions();
            if(!result.success) {
                reportResult(result);
            }
        });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::activeRegionLabelChanged,
            this,
            [this](domain::RegionLabel label) {
                m_activeLabel = label;
                if(m_mainSectionView) {
                    m_mainSectionView->setActiveRegionLabel(label);
                }
            });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::regionRectangleSelected,
            this,
            [this](const domain::YzRectangle& rectangle, domain::RegionLabel label) {
                reportResult(m_controller.applyRegionOverride(rectangle, label));
            });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::undoRequested, this, [this]() {
            reportResult(m_controller.undoRegionOverride());
        });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::redoRequested, this, [this]() {
            reportResult(m_controller.redoRegionOverride());
        });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::restoreAutomaticRequested, this, [this]() {
            reportResult(m_controller.restoreAutomaticRegions());
        });
        connect(
            &m_leftPanel,
            &RotationBodyPlanningLeftPanel::boundaryModeChanged,
            this,
            [this](domain::BoundaryMode mode) {
                m_controller.setBoundaryMode(mode);
                refresh();
            });
        connect(&m_leftPanel, &RotationBodyPlanningLeftPanel::confirmBoundaryRequested, this, [this]() {
            reportResult(m_controller.confirmSprayBoundary());
        });

        connect(&m_rightPanel, &RotationBodyPlanningRightPanel::reopenBoundaryRequested,
            this, [this]() {
                reportResult(m_controller.reopenBoundaryForEditing());
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryGenerateRequested,
            this,
            [this](const domain::TrajectoryGenerationParameters& parameters) {
                const RotationBodyControllerResult updated =
                    m_controller.updateTrajectoryParameters(parameters);
                if(!updated.success) {
                    reportResult(updated);
                    return;
                }
                reportResult(m_controller.generateCurrentTrajectory());
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::automaticTrajectoriesRequested,
            this,
            [this](int trajectoryCount) {
                reportResult(m_controller.appendAutomaticTrajectories(trajectoryCount));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::importTrajectoryParametersRequested,
            this,
            [this](const QString& sourcePath) {
                reportResult(m_controller.importTrajectoryParameterTextFile(
                    std::filesystem::path(sourcePath.toStdWString())));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectorySwapDirectionRequested,
            this,
            [this]() {
                reportResult(m_controller.swapCurrentTrajectoryDirection());
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryDisplayModeChanged,
            this,
            [this](domain::TrajectoryDisplayMode mode) {
                reportResult(m_controller.setTrajectoryDisplayMode(mode));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryPointSelectionChanged,
            this,
            [this](const std::vector<std::size_t>& indices) {
                m_controller.setSelectedTrajectoryPointIndices(indices);
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryInterpolateRequested,
            this,
            [this](const std::vector<std::size_t>& indices, double intervalSeconds) {
                reportResult(m_controller.interpolateCurrentTrajectory(indices, intervalSeconds));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryTransformPointsRequested,
            this,
            [this](const std::vector<std::size_t>& indices,
                const domain::TransformComponents& delta) {
                reportResult(m_controller.transformCurrentTrajectoryPoints(indices, delta));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::beginNewTrajectoryRequested,
            this,
            [this]() { reportResult(m_controller.beginNewTrajectory()); });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::loadTrajectoryRequested,
            this,
            [this](const std::string& passId) {
                reportResult(m_controller.loadTrajectoryForEditing(passId));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::saveCurrentTrajectoryRequested,
            this,
            [this]() { reportResult(m_controller.saveCurrentTrajectoryToGroup()); });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::exportTrajectoryGroupRequested,
            this,
            [this]() { reportResult(m_controller.exportTrajectoryGroupTextFile()); });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::removeTrajectoryRequested,
            this,
            [this](const std::string& passId) {
                reportResult(m_controller.removeTrajectoryPass(passId));
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryVisibilityChanged,
            this,
            [this](const std::string& passId, bool visible) {
                const RotationBodyControllerResult result =
                    m_controller.setTrajectoryPassVisible(passId, visible);
                if(!result.success) reportResult(result);
            });
        connect(
            &m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryTransitionChanged,
            this,
            [this](const std::string& passId, double seconds) {
                reportResult(m_controller.setTrajectoryTransitionAfter(passId, seconds));
            });
        connect(&m_rightPanel,
            &RotationBodyPlanningRightPanel::trajectoryCycleCountChanged,
            this,
            [this](int count) {
                reportResult(m_controller.setTrajectoryCycleCount(count));
            });
    }
}
