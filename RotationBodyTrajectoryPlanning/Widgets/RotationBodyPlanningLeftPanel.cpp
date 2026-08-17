#include "RotationBodyPlanningLeftPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"
#include "ModelTransformPanel.h"
#include "RotationBodyWorkflowNavigation.h"
#include "SectionRegionPanel.h"

#include "RobotQtWidgetUtils.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QStyle>
#include <QVBoxLayout>

namespace smrobot::workbench::spray::rotationbody
{
    RotationBodyPlanningLeftPanel::RotationBodyPlanningLeftPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyPlanningLeftPanel"));
        setMinimumSize(0, 0);
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(10, 10, 10, 10);
        layout->setSpacing(8);

        m_navigation = new RotationBodyWorkflowNavigation(this);
        layout->addWidget(m_navigation);
        auto* divider = new QFrame(this);
        divider->setFrameShape(QFrame::HLine);
        divider->setFrameShadow(QFrame::Sunken);
        layout->addWidget(divider);
        m_contentStack = new QStackedWidget(this);
        m_contentStack->setObjectName(QStringLiteral("rotationBodyPlanningWorkflowStack"));
        m_modelScrollArea = new QScrollArea(m_contentStack);
        m_modelScrollArea->setObjectName(QStringLiteral("rotationBodyModelTransformScrollArea"));
        m_modelScrollArea->setWidgetResizable(true);
        m_modelScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_modelScrollArea->setFrameShape(QFrame::NoFrame);
        m_modelTransformPanel = new ModelTransformPanel(m_modelScrollArea);
        m_modelScrollArea->setWidget(m_modelTransformPanel);
        m_sectionScrollArea = new QScrollArea(m_contentStack);
        m_sectionScrollArea->setObjectName(QStringLiteral("rotationBodySectionRegionScrollArea"));
        m_sectionScrollArea->setWidgetResizable(true);
        m_sectionScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_sectionScrollArea->setFrameShape(QFrame::NoFrame);
        m_sectionRegionPanel = new SectionRegionPanel(m_sectionScrollArea);
        m_sectionScrollArea->setWidget(m_sectionRegionPanel);
        m_contentStack->addWidget(m_modelScrollArea);
        m_contentStack->addWidget(m_sectionScrollArea);
        layout->addWidget(m_contentStack, 1);

        auto* commandDivider = new QFrame(this);
        commandDivider->setFrameShape(QFrame::HLine);
        commandDivider->setFrameShadow(QFrame::Sunken);
        layout->addWidget(commandDivider);
        auto* commandRow = new QHBoxLayout();
        commandRow->setContentsMargins(0, 0, 0, 0);
        commandRow->setSpacing(6);
        m_saveButton = new QPushButton(this);
        m_saveButton->setObjectName(QStringLiteral("rotationBodyCommand.save"));
        m_saveButton->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
        m_saveButton->setMinimumHeight(34);
        robot_qt_viewer::configureInspectorButton(m_saveButton);
        commandRow->addWidget(m_saveButton);
        m_discardButton = new QPushButton(this);
        m_discardButton->setObjectName(QStringLiteral("rotationBodyCommand.discard"));
        m_discardButton->setIcon(style()->standardIcon(QStyle::SP_DialogCancelButton));
        m_discardButton->setMinimumHeight(34);
        robot_qt_viewer::configureInspectorButton(m_discardButton);
        commandRow->addWidget(m_discardButton);
        layout->addLayout(commandRow);

        connect(
            m_navigation,
            &RotationBodyWorkflowNavigation::workflowChanged,
            this,
            [this](RotationBodyWorkflow workflow) {
                setCurrentWorkflow(workflow);
                emit workflowChanged(workflow);
            });
        connect(m_navigation, &RotationBodyWorkflowNavigation::viewTransformRequested,
            this, &RotationBodyPlanningLeftPanel::returnToPlanningWorkpieceRequested);
        connect(m_modelTransformPanel, &ModelTransformPanel::importRequested,
            this, &RotationBodyPlanningLeftPanel::importRequested);
        connect(m_modelTransformPanel, &ModelTransformPanel::planningDeltaEdited,
            this, &RotationBodyPlanningLeftPanel::planningDeltaEdited);
        connect(m_modelTransformPanel, &ModelTransformPanel::baseTransformEdited,
            this, &RotationBodyPlanningLeftPanel::baseTransformEdited);
        connect(m_modelTransformPanel, &ModelTransformPanel::flipRequested,
            this, &RotationBodyPlanningLeftPanel::flipRequested);
        connect(m_modelTransformPanel, &ModelTransformPanel::resetRequested,
            this, &RotationBodyPlanningLeftPanel::resetRequested);
        connect(m_modelTransformPanel, &ModelTransformPanel::publishFrameChanged,
            this, &RotationBodyPlanningLeftPanel::publishFrameChanged);
        connect(m_modelTransformPanel, &ModelTransformPanel::confirmFrameRequested,
            this, &RotationBodyPlanningLeftPanel::confirmFrameRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::mainViewModeChanged,
            this, &RotationBodyPlanningLeftPanel::mainViewModeChanged);
        connect(m_sectionRegionPanel, &SectionRegionPanel::extractSectionRequested,
            this, &RotationBodyPlanningLeftPanel::extractSectionRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::recognizeRegionsRequested,
            this, &RotationBodyPlanningLeftPanel::recognizeRegionsRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::activeRegionLabelChanged,
            this, &RotationBodyPlanningLeftPanel::activeRegionLabelChanged);
        connect(m_sectionRegionPanel, &SectionRegionPanel::regionRectangleSelected,
            this, &RotationBodyPlanningLeftPanel::regionRectangleSelected);
        connect(m_sectionRegionPanel, &SectionRegionPanel::undoRequested,
            this, &RotationBodyPlanningLeftPanel::undoRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::redoRequested,
            this, &RotationBodyPlanningLeftPanel::redoRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::restoreAutomaticRequested,
            this, &RotationBodyPlanningLeftPanel::restoreAutomaticRequested);
        connect(m_sectionRegionPanel, &SectionRegionPanel::boundaryModeChanged,
            this, &RotationBodyPlanningLeftPanel::boundaryModeChanged);
        connect(m_sectionRegionPanel, &SectionRegionPanel::confirmBoundaryRequested,
            this, &RotationBodyPlanningLeftPanel::confirmBoundaryRequested);
        connect(m_saveButton, &QPushButton::clicked,
            this, &RotationBodyPlanningLeftPanel::saveProgressAndExitRequested);
        connect(m_discardButton, &QPushButton::clicked,
            this, &RotationBodyPlanningLeftPanel::discardAndExitRequested);

        retranslate();
        setCurrentWorkflow(m_currentWorkflow);
    }

    void RotationBodyPlanningLeftPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) {
            return;
        }
        m_languageCode = canonical;
        m_navigation->setLanguageCode(canonical);
        m_modelTransformPanel->setLanguageCode(canonical);
        m_sectionRegionPanel->setLanguageCode(canonical);
        retranslate();
    }

    QString RotationBodyPlanningLeftPanel::languageCode() const
    {
        return m_languageCode;
    }

    void RotationBodyPlanningLeftPanel::setViewModel(
        const RotationBodyPlanningViewModel& viewModel)
    {
        m_modelTransformPanel->setViewModel(viewModel);
        m_sectionRegionPanel->setViewModel(viewModel);
        m_navigation->setViewTransformEnabled(viewModel.hasModel && !viewModel.isBusy);
        m_saveButton->setEnabled(viewModel.canSaveProgress);
    }

    void RotationBodyPlanningLeftPanel::setImportDirectory(const QString& directory)
    {
        m_modelTransformPanel->setImportDirectory(directory);
    }

    void RotationBodyPlanningLeftPanel::setCurrentWorkflow(RotationBodyWorkflow workflow)
    {
        m_currentWorkflow = workflow;
        m_navigation->setCurrentWorkflow(workflow);
        m_contentStack->setCurrentWidget(
            workflow == RotationBodyWorkflow::SectionRegionPlanning
                ? static_cast<QWidget*>(m_sectionScrollArea)
                : static_cast<QWidget*>(m_modelScrollArea));
    }

    RotationBodyWorkflow RotationBodyPlanningLeftPanel::currentWorkflow() const noexcept
    {
        return m_currentWorkflow;
    }

    RotationBodyWorkflowNavigation* RotationBodyPlanningLeftPanel::navigation() const noexcept
    {
        return m_navigation;
    }

    ModelTransformPanel* RotationBodyPlanningLeftPanel::modelTransformPanel() const noexcept
    {
        return m_modelTransformPanel;
    }

    SectionRegionPanel* RotationBodyPlanningLeftPanel::sectionRegionPanel() const noexcept
    {
        return m_sectionRegionPanel;
    }

    SectionView* RotationBodyPlanningLeftPanel::previewView() const noexcept
    {
        return m_sectionRegionPanel->previewView();
    }

    void RotationBodyPlanningLeftPanel::setMainViewMode(RotationBodyMainViewMode mode)
    {
        m_sectionRegionPanel->setMainViewMode(mode);
    }

    RotationBodyMainViewMode RotationBodyPlanningLeftPanel::mainViewMode() const noexcept
    {
        return m_sectionRegionPanel->mainViewMode();
    }

    void RotationBodyPlanningLeftPanel::setActiveRegionLabel(domain::RegionLabel label)
    {
        m_sectionRegionPanel->setActiveRegionLabel(label);
    }

    void RotationBodyPlanningLeftPanel::retranslate()
    {
        m_saveButton->setText(
            RotationBodyPlanningTranslations::text(m_languageCode, "command.save"));
        m_saveButton->setToolTip(
            RotationBodyPlanningTranslations::text(m_languageCode, "command.save_tooltip"));
        m_discardButton->setText(
            RotationBodyPlanningTranslations::text(m_languageCode, "command.discard"));
        m_discardButton->setToolTip(
            RotationBodyPlanningTranslations::text(m_languageCode, "command.discard_tooltip"));
    }
}
