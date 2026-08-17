#include "SectionRegionPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"
#include "SectionView.h"

#include "RobotQtWidgetUtils.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStyle>
#include <QToolButton>
#include <QVariant>
#include <QVBoxLayout>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        constexpr std::array<domain::RegionLabel, 5> labels{ {
            domain::RegionLabel::Unclassified,
            domain::RegionLabel::ToothTop,
            domain::RegionLabel::ToothWall,
            domain::RegionLabel::ToothBottom,
            domain::RegionLabel::Transition
        } };

        void configureSegment(QToolButton* button)
        {
            button->setCheckable(true);
            button->setMinimumHeight(32);
            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        }
    }

    SectionRegionPanel::SectionRegionPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodySectionRegionPanel"));
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        m_stageLabel = new QLabel(this);
        m_stageLabel->setObjectName(QStringLiteral("rotationBodySection.stage"));
        m_stageLabel->setWordWrap(true);
        m_stageLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_stageLabel);
        layout->addWidget(m_stageLabel);
        m_errorLabel = new QLabel(this);
        m_errorLabel->setObjectName(QStringLiteral("rotationBodySection.error"));
        m_errorLabel->setWordWrap(true);
        m_errorLabel->setProperty("validationError", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_errorLabel);
        layout->addWidget(m_errorLabel);

        m_viewGroup = new QGroupBox(this);
        auto* viewLayout = new QHBoxLayout(m_viewGroup);
        viewLayout->setContentsMargins(8, 20, 8, 8);
        viewLayout->setSpacing(4);
        m_viewButtonGroup = new QButtonGroup(this);
        m_viewButtonGroup->setExclusive(true);
        m_sceneButton = new QToolButton(m_viewGroup);
        m_sceneButton->setObjectName(QStringLiteral("rotationBodySection.scene3d"));
        configureSegment(m_sceneButton);
        m_sceneButton->setChecked(true);
        m_viewButtonGroup->addButton(m_sceneButton, 0);
        viewLayout->addWidget(m_sceneButton);
        m_sectionButton = new QToolButton(m_viewGroup);
        m_sectionButton->setObjectName(QStringLiteral("rotationBodySection.sectionView"));
        configureSegment(m_sectionButton);
        m_viewButtonGroup->addButton(m_sectionButton, 1);
        viewLayout->addWidget(m_sectionButton);
        layout->addWidget(m_viewGroup);

        m_previewGroup = new QGroupBox(this);
        auto* previewLayout = new QVBoxLayout(m_previewGroup);
        previewLayout->setContentsMargins(8, 20, 8, 8);
        previewLayout->setSpacing(6);
        m_previewView = new SectionView(m_previewGroup);
        m_previewView->setObjectName(QStringLiteral("rotationBodySection.preview"));
        m_previewView->setMinimumHeight(230);
        m_previewView->setMaximumHeight(260);
        previewLayout->addWidget(m_previewView);
        auto* planningCommandRow = new QHBoxLayout();
        planningCommandRow->setContentsMargins(0, 0, 0, 0);
        planningCommandRow->setSpacing(4);
        m_extractButton = new QPushButton(m_previewGroup);
        m_extractButton->setObjectName(QStringLiteral("rotationBodySection.extract"));
        m_extractButton->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
        m_extractButton->setMinimumHeight(34);
        robot_qt_viewer::configureInspectorButton(m_extractButton);
        planningCommandRow->addWidget(m_extractButton, 1);
        m_recognizeButton = new QPushButton(m_previewGroup);
        m_recognizeButton->setObjectName(QStringLiteral("rotationBodySection.recognize"));
        m_recognizeButton->setIcon(style()->standardIcon(QStyle::SP_FileDialogInfoView));
        m_recognizeButton->setMinimumHeight(34);
        robot_qt_viewer::configureInspectorButton(m_recognizeButton);
        planningCommandRow->addWidget(m_recognizeButton, 1);
        previewLayout->addLayout(planningCommandRow);
        layout->addWidget(m_previewGroup);

        m_regionGroup = new QGroupBox(this);
        auto* regionLayout = new QVBoxLayout(m_regionGroup);
        regionLayout->setContentsMargins(8, 20, 8, 8);
        regionLayout->setSpacing(6);
        m_activeLabelCaption = new QLabel(m_regionGroup);
        m_activeLabelCaption->setWordWrap(true);
        regionLayout->addWidget(m_activeLabelCaption);
        auto* labelGrid = new QGridLayout();
        labelGrid->setContentsMargins(0, 0, 0, 0);
        labelGrid->setSpacing(4);
        robot_qt_viewer::configureInspectorGrid(labelGrid);
        m_labelButtonGroup = new QButtonGroup(this);
        m_labelButtonGroup->setExclusive(true);
        for(std::size_t index = 0; index < labels.size(); ++index) {
            QToolButton* button = new QToolButton(m_regionGroup);
            button->setObjectName(QStringLiteral("rotationBodySection.label.%1")
                .arg(static_cast<int>(labels[index])));
            button->setCheckable(true);
            button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
            button->setIcon(swatchIcon(labels[index]));
            button->setMinimumHeight(32);
            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            m_labelButtonGroup->addButton(button, static_cast<int>(labels[index]));
            m_labelButtons[index] = button;
            labelGrid->addWidget(
                button,
                static_cast<int>(index / 2),
                static_cast<int>(index % 2));
        }
        m_labelButtons[1]->setChecked(true);
        regionLayout->addLayout(labelGrid);
        layout->addWidget(m_regionGroup);

        m_editGroup = new QGroupBox(this);
        auto* editLayout = new QHBoxLayout(m_editGroup);
        editLayout->setContentsMargins(8, 20, 8, 8);
        editLayout->setSpacing(4);
        m_undoButton = new QToolButton(m_editGroup);
        m_undoButton->setObjectName(QStringLiteral("rotationBodySection.undo"));
        m_undoButton->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
        m_undoButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
        m_undoButton->setFixedSize(32, 32);
        editLayout->addWidget(m_undoButton);
        m_redoButton = new QToolButton(m_editGroup);
        m_redoButton->setObjectName(QStringLiteral("rotationBodySection.redo"));
        m_redoButton->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
        m_redoButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
        m_redoButton->setFixedSize(32, 32);
        editLayout->addWidget(m_redoButton);
        m_restoreButton = new QPushButton(m_editGroup);
        m_restoreButton->setObjectName(QStringLiteral("rotationBodySection.restore"));
        m_restoreButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
        robot_qt_viewer::configureInspectorButton(m_restoreButton);
        editLayout->addWidget(m_restoreButton, 1);
        layout->addWidget(m_editGroup);

        m_boundaryGroup = new QGroupBox(this);
        auto* boundaryLayout = new QVBoxLayout(m_boundaryGroup);
        boundaryLayout->setContentsMargins(8, 20, 8, 8);
        boundaryLayout->setSpacing(6);
        auto* boundaryModeRow = new QHBoxLayout();
        boundaryModeRow->setContentsMargins(0, 0, 0, 0);
        boundaryModeRow->setSpacing(4);
        m_boundaryButtonGroup = new QButtonGroup(this);
        m_boundaryButtonGroup->setExclusive(true);
        m_maximumYButton = new QToolButton(m_boundaryGroup);
        m_maximumYButton->setObjectName(QStringLiteral("rotationBodySection.boundaryMaximumY"));
        m_maximumYButton->setCheckable(true);
        m_maximumYButton->setChecked(true);
        m_maximumYButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        QPixmap boundarySwatch(14, 14);
        boundarySwatch.fill(SectionView::boundaryColor());
        m_maximumYButton->setIcon(QIcon(boundarySwatch));
        m_maximumYButton->setMinimumHeight(32);
        m_maximumYButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_boundaryButtonGroup->addButton(
            m_maximumYButton,
            static_cast<int>(domain::BoundaryMode::MaximumToothTopY));
        boundaryModeRow->addWidget(m_maximumYButton, 1);
        m_envelopeButton = new QToolButton(m_boundaryGroup);
        m_envelopeButton->setObjectName(QStringLiteral("rotationBodySection.boundaryEnvelope"));
        m_envelopeButton->setCheckable(true);
        m_envelopeButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        m_envelopeButton->setIcon(QIcon(boundarySwatch));
        m_envelopeButton->setMinimumHeight(32);
        m_envelopeButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_boundaryButtonGroup->addButton(
            m_envelopeButton,
            static_cast<int>(domain::BoundaryMode::ToothTopEnvelope));
        boundaryModeRow->addWidget(m_envelopeButton, 1);
        boundaryLayout->addLayout(boundaryModeRow);
        m_confirmBoundaryButton = new QPushButton(m_boundaryGroup);
        m_confirmBoundaryButton->setObjectName(QStringLiteral("rotationBodySection.confirmBoundary"));
        m_confirmBoundaryButton->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
        m_confirmBoundaryButton->setMinimumHeight(34);
        robot_qt_viewer::configureInspectorButton(m_confirmBoundaryButton);
        boundaryLayout->addWidget(m_confirmBoundaryButton);
        layout->addWidget(m_boundaryGroup);
        layout->addStretch(1);

        connect(
            m_viewButtonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                const RotationBodyMainViewMode mode = id == 1
                    ? RotationBodyMainViewMode::Section
                    : RotationBodyMainViewMode::Scene3d;
                if(m_mainViewMode == mode) {
                    return;
                }
                m_mainViewMode = mode;
                emit mainViewModeChanged(mode);
            });
        connect(m_extractButton, &QPushButton::clicked, this, &SectionRegionPanel::extractSectionRequested);
        connect(m_recognizeButton, &QPushButton::clicked, this, &SectionRegionPanel::recognizeRegionsRequested);
        connect(
            m_labelButtonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                m_activeLabel = static_cast<domain::RegionLabel>(id);
                m_previewView->setActiveRegionLabel(m_activeLabel);
                emit activeRegionLabelChanged(m_activeLabel);
            });
        connect(
            m_previewView,
            &SectionView::rectangleSelected,
            this,
            &SectionRegionPanel::regionRectangleSelected);
        connect(m_undoButton, &QToolButton::clicked, this, &SectionRegionPanel::undoRequested);
        connect(m_redoButton, &QToolButton::clicked, this, &SectionRegionPanel::redoRequested);
        connect(m_restoreButton, &QPushButton::clicked, this, &SectionRegionPanel::restoreAutomaticRequested);
        connect(
            m_boundaryButtonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                emit boundaryModeChanged(static_cast<domain::BoundaryMode>(id));
            });
        connect(
            m_confirmBoundaryButton,
            &QPushButton::clicked,
            this,
            &SectionRegionPanel::confirmBoundaryRequested);

        retranslate();
        setViewModel(m_viewModel);
    }

    void SectionRegionPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) {
            return;
        }
        m_languageCode = canonical;
        m_previewView->setLanguageCode(canonical);
        retranslate();
        updateDynamicText();
    }

    QString SectionRegionPanel::languageCode() const
    {
        return m_languageCode;
    }

    void SectionRegionPanel::setViewModel(const RotationBodyPlanningViewModel& viewModel)
    {
        m_viewModel = viewModel;
        {
            const QSignalBlocker maximumBlocker(m_maximumYButton);
            const QSignalBlocker envelopeBlocker(m_envelopeButton);
            m_maximumYButton->setChecked(
                viewModel.boundaryMode == domain::BoundaryMode::MaximumToothTopY);
            m_envelopeButton->setChecked(
                viewModel.boundaryMode == domain::BoundaryMode::ToothTopEnvelope);
        }
        m_previewView->setSnapshot(viewModel.sectionView);
        updateDynamicText();
        updateEnabledState();
    }

    const RotationBodyPlanningViewModel& SectionRegionPanel::viewModel() const noexcept
    {
        return m_viewModel;
    }

    void SectionRegionPanel::setMainViewMode(RotationBodyMainViewMode mode)
    {
        m_mainViewMode = mode;
        const QSignalBlocker sceneBlocker(m_sceneButton);
        const QSignalBlocker sectionBlocker(m_sectionButton);
        m_sceneButton->setChecked(mode == RotationBodyMainViewMode::Scene3d);
        m_sectionButton->setChecked(mode == RotationBodyMainViewMode::Section);
    }

    RotationBodyMainViewMode SectionRegionPanel::mainViewMode() const noexcept
    {
        return m_mainViewMode;
    }

    void SectionRegionPanel::setActiveRegionLabel(domain::RegionLabel label)
    {
        m_activeLabel = label;
        if(QAbstractButton* button = m_labelButtonGroup->button(static_cast<int>(label))) {
            const QSignalBlocker blocker(button);
            button->setChecked(true);
        }
        m_previewView->setActiveRegionLabel(label);
    }

    domain::RegionLabel SectionRegionPanel::activeRegionLabel() const noexcept
    {
        return m_activeLabel;
    }

    SectionView* SectionRegionPanel::previewView() const noexcept
    {
        return m_previewView;
    }

    QIcon SectionRegionPanel::swatchIcon(domain::RegionLabel label)
    {
        QPixmap pixmap(14, 14);
        pixmap.fill(SectionView::semanticColor(label));
        return QIcon(pixmap);
    }

    void SectionRegionPanel::updateEnabledState()
    {
        m_extractButton->setEnabled(m_viewModel.canCreateSection);
        m_recognizeButton->setEnabled(m_viewModel.canRecognizeRegions);
        for(QToolButton* button : m_labelButtons) {
            button->setEnabled(m_viewModel.canEditRegions);
        }
        m_previewView->setEnabled(!m_viewModel.isBusy &&
            (m_viewModel.canEditRegions || m_viewModel.sectionView.section.has_value()));
        m_undoButton->setEnabled(m_viewModel.canUndoRegionEdit);
        m_redoButton->setEnabled(m_viewModel.canRedoRegionEdit);
        m_restoreButton->setEnabled(m_viewModel.canRestoreAutomaticRegions);
        m_boundaryGroup->setEnabled(m_viewModel.canEditRegions);
        m_confirmBoundaryButton->setEnabled(m_viewModel.canConfirmBoundary);
        m_sectionButton->setEnabled(
            !m_viewModel.isBusy && m_viewModel.sectionView.section.has_value());
        if(!m_viewModel.sectionView.section &&
            m_mainViewMode == RotationBodyMainViewMode::Section) {
            setMainViewMode(RotationBodyMainViewMode::Scene3d);
            emit mainViewModeChanged(RotationBodyMainViewMode::Scene3d);
        }
    }

    void SectionRegionPanel::updateDynamicText()
    {
        m_stageLabel->setText(
            RotationBodyPlanningTranslations::stageName(m_languageCode, m_viewModel.stage));
        m_extractButton->setText(RotationBodyPlanningTranslations::text(
            m_languageCode,
            m_viewModel.sectionView.section ? "section.reextract" : "section.extract"));
        const QString error = RotationBodyPlanningTranslations::errorText(
            m_languageCode,
            m_viewModel.errorCode);
        m_errorLabel->setText(error);
        m_errorLabel->setVisible(!error.isEmpty());
        for(std::size_t index = 0; index < labels.size(); ++index) {
            const std::size_t countIndex = static_cast<std::size_t>(labels[index]);
            const std::size_t count = countIndex < m_viewModel.regionSegmentCounts.size()
                ? m_viewModel.regionSegmentCounts[countIndex]
                : 0;
            m_labelButtons[index]->setText(QStringLiteral("%1  %2")
                .arg(RotationBodyPlanningTranslations::regionName(
                    m_languageCode,
                    labels[index]))
                .arg(count));
        }
    }

    void SectionRegionPanel::retranslate()
    {
        const auto translated = [this](const char* key) {
            return RotationBodyPlanningTranslations::text(m_languageCode, key);
        };
        m_viewGroup->setTitle(translated("section.view_title"));
        m_sceneButton->setText(translated("section.scene_3d"));
        m_sectionButton->setText(translated("section.section_view"));
        m_previewGroup->setTitle(translated("section.preview"));
        m_extractButton->setToolTip(translated("section.extract_tooltip"));
        m_regionGroup->setTitle(translated("region.title"));
        m_recognizeButton->setText(translated("region.auto_recognize"));
        m_recognizeButton->setToolTip(translated("region.auto_tooltip"));
        m_activeLabelCaption->setText(translated("region.active_label"));
        m_editGroup->setTitle(translated("region.edit_title"));
        m_undoButton->setToolTip(translated("region.undo_tooltip"));
        m_undoButton->setAccessibleName(translated("region.undo"));
        m_redoButton->setToolTip(translated("region.redo_tooltip"));
        m_redoButton->setAccessibleName(translated("region.redo"));
        m_restoreButton->setText(translated("region.restore"));
        m_restoreButton->setToolTip(translated("region.restore_tooltip"));
        m_boundaryGroup->setTitle(translated("boundary.title"));
        m_maximumYButton->setText(translated("boundary.maximum_y"));
        m_envelopeButton->setText(translated("boundary.envelope"));
        m_confirmBoundaryButton->setText(translated("boundary.confirm"));
        m_confirmBoundaryButton->setToolTip(translated("boundary.confirm_tooltip"));
    }
}
