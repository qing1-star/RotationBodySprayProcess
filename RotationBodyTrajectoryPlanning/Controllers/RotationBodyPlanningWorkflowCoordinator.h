#pragma once

#include "RotationBodyPlanningController.h"

#include <QMetaObject>
#include <QObject>
#include <QPointer>

namespace smrobot::workbench::spray::rotationbody
{
    class RotationBodyPlanningLeftPanel;
    class RotationBodyPlanningRightPanel;
    class SectionView;

    class RotationBodyPlanningWorkflowCoordinator final : public QObject
    {
        Q_OBJECT

    public:
        RotationBodyPlanningWorkflowCoordinator(
            RotationBodyPlanningController& controller,
            RotationBodyPlanningLeftPanel& leftPanel,
            RotationBodyPlanningRightPanel& rightPanel,
            QObject* parent = nullptr);

        RotationBodyControllerResult activate();
        RotationBodyControllerResult deactivate();
        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setImportDirectory(const QString& directory);
        void setMainSectionView(SectionView* sectionView);
        SectionView* mainSectionView() const noexcept;
        void setCurrentWorkflow(RotationBodyWorkflow workflow);
        RotationBodyWorkflow currentWorkflow() const noexcept;
        void setMainViewMode(RotationBodyMainViewMode mode);
        RotationBodyMainViewMode mainViewMode() const noexcept;
        RotationBodyControllerResult setCalibrationHelpersVisible(bool visible);
        RotationBodyControllerResult restoreViewportPresentation();
        RotationBodyControllerResult rebuildViewportOverlay();
        RotationBodyControllerResult clearViewportOverlay();
        void refresh();

    signals:
        void workflowChanged(RotationBodyWorkflow workflow);
        void rightPanelVisibilityChanged(bool visible);
        void mainViewModeChanged(RotationBodyMainViewMode mode);
        void viewModelChanged(const RotationBodyPlanningViewModel& viewModel);
        void statusMessageRequested(const QString& message, int timeoutMs);
        void exitCompleted(bool saved);

    private:
        void reportResult(
            const RotationBodyControllerResult& result,
            const char* successTranslationKey = nullptr,
            int timeoutMs = 3500);
        QString localizedCurrentStatus() const;
        void persistUiState();
        void bindPanelSignals();

        RotationBodyPlanningController& m_controller;
        RotationBodyPlanningLeftPanel& m_leftPanel;
        RotationBodyPlanningRightPanel& m_rightPanel;
        QPointer<SectionView> m_mainSectionView;
        QMetaObject::Connection m_mainRectangleConnection;
        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyWorkflow m_currentWorkflow{ RotationBodyWorkflow::ModelTransform };
        RotationBodyMainViewMode m_mainViewMode{ RotationBodyMainViewMode::Scene3d };
        domain::RegionLabel m_activeLabel{ domain::RegionLabel::ToothTop };
        bool m_restoringUiState{ false };
    };
}
