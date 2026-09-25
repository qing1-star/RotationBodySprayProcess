#include "RotationBodyPlanningRightPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"
#include "TrajectoryPlanningPanel.h"

#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
    }

    RotationBodyPlanningRightPanel::RotationBodyPlanningRightPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyPlanningRightPanel"));
        setMinimumSize(0, 0);
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(8);

        m_contentStack = new QStackedWidget(this);
        m_contentStack->setObjectName(QStringLiteral("rotationBodyRightWorkflowStack"));
        m_trajectoryScrollArea = new QScrollArea(m_contentStack);
        m_trajectoryScrollArea->setObjectName(QStringLiteral("rotationBodyTrajectoryScrollArea"));
        m_trajectoryScrollArea->setWidgetResizable(true);
        m_trajectoryScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_trajectoryScrollArea->setFrameShape(QFrame::NoFrame);
        m_trajectoryPanel = new TrajectoryPlanningPanel(m_trajectoryScrollArea);
        m_trajectoryScrollArea->setWidget(m_trajectoryPanel);
        m_contentStack->addWidget(m_trajectoryScrollArea);
        layout->addWidget(m_contentStack, 1);

        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::reopenBoundaryRequested,
            this, &RotationBodyPlanningRightPanel::reopenBoundaryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::generateRequested,
            this, &RotationBodyPlanningRightPanel::trajectoryGenerateRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::automaticTrajectoriesRequested,
            this, &RotationBodyPlanningRightPanel::automaticTrajectoriesRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::importTrajectoryParametersRequested,
            this, &RotationBodyPlanningRightPanel::importTrajectoryParametersRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::swapDirectionRequested,
            this, &RotationBodyPlanningRightPanel::trajectorySwapDirectionRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::displayModeChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryDisplayModeChanged);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::pointSelectionChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryPointSelectionChanged);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::interpolateRequested,
            this, &RotationBodyPlanningRightPanel::trajectoryInterpolateRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::transformPointsRequested,
            this, &RotationBodyPlanningRightPanel::trajectoryTransformPointsRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::beginNewTrajectoryRequested,
            this, &RotationBodyPlanningRightPanel::beginNewTrajectoryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::loadTrajectoryRequested,
            this, &RotationBodyPlanningRightPanel::loadTrajectoryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::saveCurrentTrajectoryRequested,
            this, &RotationBodyPlanningRightPanel::saveCurrentTrajectoryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::exportTrajectoryGroupRequested,
            this, &RotationBodyPlanningRightPanel::exportTrajectoryGroupRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::removeTrajectoryRequested,
            this, &RotationBodyPlanningRightPanel::removeTrajectoryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::trajectoryVisibilityChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryVisibilityChanged);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::trajectoryTransitionChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryTransitionChanged);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::trajectoryCycleCountChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryCycleCountChanged);
        retranslate();
        setCurrentRightWorkflow(m_currentRightWorkflow);
    }

    void RotationBodyPlanningRightPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) return;
        m_languageCode = canonical;
        m_trajectoryPanel->setLanguageCode(canonical);
        retranslate();
    }

    QString RotationBodyPlanningRightPanel::languageCode() const
    {
        return m_languageCode;
    }

    void RotationBodyPlanningRightPanel::setViewModel(
        const RotationBodyPlanningViewModel& viewModel)
    {
        m_trajectoryPanel->setViewModel(viewModel);
    }

    void RotationBodyPlanningRightPanel::setCurrentRightWorkflow(
        RotationBodyRightWorkflow workflow)
    {
        m_currentRightWorkflow = workflow;
        Q_UNUSED(workflow);
        m_contentStack->setCurrentWidget(m_trajectoryScrollArea);
    }

    RotationBodyRightWorkflow
    RotationBodyPlanningRightPanel::currentRightWorkflow() const noexcept
    {
        return m_currentRightWorkflow;
    }

    void RotationBodyPlanningRightPanel::setCurrentWorkflow(RotationBodyWorkflow workflow)
    {
        m_currentWorkflow = workflow;
    }

    RotationBodyWorkflow RotationBodyPlanningRightPanel::currentWorkflow() const noexcept
    {
        return m_currentWorkflow;
    }

    void RotationBodyPlanningRightPanel::setMainViewMode(RotationBodyMainViewMode mode)
    {
        m_mainViewMode = mode;
    }

    RotationBodyMainViewMode RotationBodyPlanningRightPanel::mainViewMode() const noexcept
    {
        return m_mainViewMode;
    }

    void RotationBodyPlanningRightPanel::setActiveRegionLabel(domain::RegionLabel label)
    {
        Q_UNUSED(label);
    }

    SectionRegionPanel* RotationBodyPlanningRightPanel::sectionRegionPanel() const noexcept
    {
        return nullptr;
    }

    SectionView* RotationBodyPlanningRightPanel::previewView() const noexcept
    {
        return nullptr;
    }

    TrajectoryPlanningPanel*
    RotationBodyPlanningRightPanel::trajectoryPlanningPanel() const noexcept
    {
        return m_trajectoryPanel;
    }

    bool RotationBodyPlanningRightPanel::hasVisibleContent() const noexcept
    {
        return true;
    }

    void RotationBodyPlanningRightPanel::retranslate()
    {
        Q_UNUSED(m_languageCode);
    }
}
