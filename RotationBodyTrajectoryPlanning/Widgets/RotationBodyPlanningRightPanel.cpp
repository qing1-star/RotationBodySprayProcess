#include "RotationBodyPlanningRightPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"
#include "ABBTranslationPanel.h"
#include "TrajectoryPlanningPanel.h"
#include "WorkpieceCalibrationPanel.h"

#include <QButtonGroup>
#include <QFrame>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        void configureNavigationButton(QToolButton* button)
        {
            button->setCheckable(true);
            button->setMinimumHeight(48);
            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        }
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

        auto* navigationRow = new QHBoxLayout();
        navigationRow->setContentsMargins(0, 0, 0, 0);
        navigationRow->setSpacing(4);
        m_buttonGroup = new QButtonGroup(this);
        m_buttonGroup->setExclusive(true);
        m_trajectoryButton = new QToolButton(this);
        m_trajectoryButton->setObjectName(QStringLiteral("rotationBodyRightWorkflow.trajectory"));
        configureNavigationButton(m_trajectoryButton);
        m_trajectoryButton->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
        m_trajectoryButton->setChecked(true);
        m_buttonGroup->addButton(m_trajectoryButton, 0);
        navigationRow->addWidget(m_trajectoryButton);
        m_abbButton = new QToolButton(this);
        m_abbButton->setObjectName(QStringLiteral("rotationBodyRightWorkflow.abb"));
        configureNavigationButton(m_abbButton);
        m_abbButton->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
        m_buttonGroup->addButton(m_abbButton, 1);
        navigationRow->addWidget(m_abbButton);
        m_calibrationButton = new QToolButton(this);
        m_calibrationButton->setObjectName(
            QStringLiteral("rotationBodyRightWorkflow.calibration"));
        configureNavigationButton(m_calibrationButton);
        m_calibrationButton->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
        m_buttonGroup->addButton(m_calibrationButton, 2);
        navigationRow->addWidget(m_calibrationButton);
        layout->addLayout(navigationRow);

        auto* divider = new QFrame(this);
        divider->setFrameShape(QFrame::HLine);
        divider->setFrameShadow(QFrame::Sunken);
        layout->addWidget(divider);

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
        m_abbScrollArea = new QScrollArea(m_contentStack);
        m_abbScrollArea->setObjectName(QStringLiteral("rotationBodyABBScrollArea"));
        m_abbScrollArea->setWidgetResizable(true);
        m_abbScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_abbScrollArea->setFrameShape(QFrame::NoFrame);
        m_abbPanel = new ABBTranslationPanel(m_abbScrollArea);
        m_abbScrollArea->setWidget(m_abbPanel);
        m_contentStack->addWidget(m_abbScrollArea);
        m_calibrationScrollArea = new QScrollArea(m_contentStack);
        m_calibrationScrollArea->setObjectName(
            QStringLiteral("rotationBodyCalibrationScrollArea"));
        m_calibrationScrollArea->setWidgetResizable(true);
        m_calibrationScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_calibrationScrollArea->setFrameShape(QFrame::NoFrame);
        m_calibrationPanel = new WorkpieceCalibrationPanel(m_calibrationScrollArea);
        m_calibrationScrollArea->setWidget(m_calibrationPanel);
        m_contentStack->addWidget(m_calibrationScrollArea);
        layout->addWidget(m_contentStack, 1);

        connect(
            m_buttonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                RotationBodyRightWorkflow workflow =
                    RotationBodyRightWorkflow::TrajectoryPlanning;
                if(id == 1) {
                    workflow = RotationBodyRightWorkflow::ABBTranslation;
                } else if(id == 2) {
                    workflow = RotationBodyRightWorkflow::WorkpieceCalibration;
                }
                setCurrentRightWorkflow(workflow);
                emit rightWorkflowChanged(workflow);
            });
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::reopenBoundaryRequested,
            this, &RotationBodyPlanningRightPanel::reopenBoundaryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::generateRequested,
            this, &RotationBodyPlanningRightPanel::trajectoryGenerateRequested);
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
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::removeTrajectoryRequested,
            this, &RotationBodyPlanningRightPanel::removeTrajectoryRequested);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::trajectoryVisibilityChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryVisibilityChanged);
        connect(m_trajectoryPanel, &TrajectoryPlanningPanel::trajectoryTransitionChanged,
            this, &RotationBodyPlanningRightPanel::trajectoryTransitionChanged);
        connect(m_abbPanel, &ABBTranslationPanel::settingsEdited,
            this, &RotationBodyPlanningRightPanel::rapidSettingsEdited);
        connect(m_abbPanel, &ABBTranslationPanel::sequenceEdited,
            this, &RotationBodyPlanningRightPanel::rapidSequenceEdited);
        connect(m_abbPanel, &ABBTranslationPanel::generateRequested,
            this, &RotationBodyPlanningRightPanel::rapidGenerateRequested);
        connect(m_abbPanel, &ABBTranslationPanel::previewStepSelected,
            this, &RotationBodyPlanningRightPanel::rapidPreviewStepSelected);
        connect(m_calibrationPanel, &WorkpieceCalibrationPanel::workspaceEdited,
            this, &RotationBodyPlanningRightPanel::calibrationWorkspaceEdited);
        connect(m_calibrationPanel, &WorkpieceCalibrationPanel::baseTransformCalculated,
            this, &RotationBodyPlanningRightPanel::calibrationBaseTransformCalculated);

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
        m_abbPanel->setLanguageCode(canonical);
        m_calibrationPanel->setLanguageCode(canonical);
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
        m_abbPanel->setViewModel(viewModel);
        m_calibrationPanel->setViewModel(viewModel);
    }

    void RotationBodyPlanningRightPanel::setCurrentRightWorkflow(
        RotationBodyRightWorkflow workflow)
    {
        m_currentRightWorkflow = workflow;
        const bool abb = workflow == RotationBodyRightWorkflow::ABBTranslation;
        const bool calibration =
            workflow == RotationBodyRightWorkflow::WorkpieceCalibration;
        m_abbButton->setChecked(abb);
        m_calibrationButton->setChecked(calibration);
        m_trajectoryButton->setChecked(!abb && !calibration);
        if(calibration) {
            m_contentStack->setCurrentWidget(m_calibrationScrollArea);
        } else if(abb) {
            m_contentStack->setCurrentWidget(m_abbScrollArea);
        } else {
            m_contentStack->setCurrentWidget(m_trajectoryScrollArea);
        }
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

    ABBTranslationPanel* RotationBodyPlanningRightPanel::abbTranslationPanel() const noexcept
    {
        return m_abbPanel;
    }

    WorkpieceCalibrationPanel*
    RotationBodyPlanningRightPanel::workpieceCalibrationPanel() const noexcept
    {
        return m_calibrationPanel;
    }

    bool RotationBodyPlanningRightPanel::hasVisibleContent() const noexcept
    {
        return true;
    }

    void RotationBodyPlanningRightPanel::retranslate()
    {
        m_trajectoryButton->setText(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.trajectory_short"));
        m_abbButton->setText(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.abb_short"));
        m_calibrationButton->setText(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.calibration_short"));
        m_trajectoryButton->setToolTip(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.trajectory"));
        m_abbButton->setToolTip(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.abb"));
        m_calibrationButton->setToolTip(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "right_workflow.calibration"));
    }
}
