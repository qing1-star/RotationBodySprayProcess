#include "ModelTransformPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include "RobotQtWidgetUtils.h"

#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>

#include <QButtonGroup>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStandardItemModel>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        constexpr std::array<const char*, 6> componentNames{ {
            "X", "Y", "Z", "Rx", "Ry", "Rz"
        } };

        QString millimeterText(double meters)
        {
            return QStringLiteral("%1 mm").arg(
                domain::metersToMillimeters(meters),
                0,
                'f',
                2);
        }

        void configureToolSegment(QToolButton* button)
        {
            button->setCheckable(true);
            button->setMinimumHeight(32);
            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        }

        void setItemEnabled(QComboBox* combo, int index, bool enabled)
        {
            auto* model = qobject_cast<QStandardItemModel*>(combo->model());
            if(model == nullptr || model->item(index) == nullptr) {
                return;
            }
            QStandardItem* item = model->item(index);
            Qt::ItemFlags flags = item->flags();
            if(enabled) {
                flags |= Qt::ItemIsEnabled;
            } else {
                flags &= ~Qt::ItemIsEnabled;
            }
            item->setFlags(flags);
        }
    }

    ModelTransformPanel::ModelTransformPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyModelTransformPanel"));
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        m_stageLabel = new QLabel(this);
        m_stageLabel->setObjectName(QStringLiteral("rotationBodyModel.stage"));
        m_stageLabel->setWordWrap(true);
        m_stageLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_stageLabel);
        layout->addWidget(m_stageLabel);

        m_objectGroup = new QGroupBox(this);
        auto* objectLayout = new QVBoxLayout(m_objectGroup);
        objectLayout->setContentsMargins(8, 20, 8, 8);
        objectLayout->setSpacing(8);
        auto* objectTypeRow = new QHBoxLayout();
        objectTypeRow->setContentsMargins(0, 0, 0, 0);
        objectTypeRow->setSpacing(4);
        m_objectTypeGroup = new QButtonGroup(this);
        m_objectTypeGroup->setExclusive(true);
        m_completePartButton = new QToolButton(m_objectGroup);
        m_completePartButton->setObjectName(QStringLiteral("rotationBodyModel.completePart"));
        configureToolSegment(m_completePartButton);
        m_completePartButton->setChecked(true);
        m_objectTypeGroup->addButton(m_completePartButton, 0);
        objectTypeRow->addWidget(m_completePartButton, 1);
        m_simulationBlockButton = new QToolButton(m_objectGroup);
        m_simulationBlockButton->setObjectName(QStringLiteral("rotationBodyModel.simulationBlock"));
        configureToolSegment(m_simulationBlockButton);
        m_objectTypeGroup->addButton(m_simulationBlockButton, 1);
        objectTypeRow->addWidget(m_simulationBlockButton, 1);

        m_simulationControls = new QWidget(m_objectGroup);
        auto* simulationForm = new QFormLayout(m_simulationControls);
        simulationForm->setContentsMargins(0, 0, 0, 0);
        simulationForm->setSpacing(6);
        robot_qt_viewer::configureInspectorForm(simulationForm);
        m_diameterLabel = new QLabel(m_simulationControls);
        m_diameterSpin = makeLengthSpin(
            QStringLiteral("rotationBodyModel.motherDiameter"),
            m_simulationControls);
        m_diameterSpin->setRange(0.001, 1000000.0);
        m_diameterSpin->setValue(300.0);
        simulationForm->addRow(m_diameterLabel, m_diameterSpin);
        m_rotationAxisLabel = new QLabel(m_simulationControls);
        m_rotationAxisCombo = new QComboBox(m_simulationControls);
        m_rotationAxisCombo->setObjectName(QStringLiteral("rotationBodyModel.rotationAxis"));
        m_toothAxisLabel = new QLabel(m_simulationControls);
        m_toothAxisCombo = new QComboBox(m_simulationControls);
        m_toothAxisCombo->setObjectName(QStringLiteral("rotationBodyModel.toothAxis"));
        const std::array<domain::SignedAxis, 6> axes{ {
            domain::SignedAxis::PositiveX,
            domain::SignedAxis::NegativeX,
            domain::SignedAxis::PositiveY,
            domain::SignedAxis::NegativeY,
            domain::SignedAxis::PositiveZ,
            domain::SignedAxis::NegativeZ
        } };
        for(domain::SignedAxis axis : axes) {
            m_rotationAxisCombo->addItem(QString(), static_cast<int>(axis));
            m_toothAxisCombo->addItem(QString(), static_cast<int>(axis));
        }
        robot_qt_viewer::configureInspectorCombo(m_rotationAxisCombo, 4);
        robot_qt_viewer::configureInspectorCombo(m_toothAxisCombo, 4);
        setComboAxis(m_rotationAxisCombo, domain::SignedAxis::PositiveZ);
        setComboAxis(m_toothAxisCombo, domain::SignedAxis::PositiveY);
        simulationForm->addRow(m_rotationAxisLabel, m_rotationAxisCombo);
        simulationForm->addRow(m_toothAxisLabel, m_toothAxisCombo);
        m_axisValidationLabel = new QLabel(m_simulationControls);
        m_axisValidationLabel->setObjectName(QStringLiteral("rotationBodyModel.axisValidation"));
        m_axisValidationLabel->setWordWrap(true);
        m_axisValidationLabel->setProperty("validationError", true);
        simulationForm->addRow(m_axisValidationLabel);
        objectLayout->addWidget(m_simulationControls);

        m_importButton = new QToolButton(m_objectGroup);
        m_importButton->setObjectName(QStringLiteral("rotationBodyModel.load"));
        m_importButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        m_importButton->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
        m_importButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_importButton->setMinimumHeight(34);
        objectTypeRow->addWidget(m_importButton, 1);
        objectLayout->insertLayout(0, objectTypeRow);
        m_sourceLabel = new QLabel(m_objectGroup);
        m_sourceLabel->setObjectName(QStringLiteral("rotationBodyModel.source"));
        m_sourceLabel->setWordWrap(true);
        robot_qt_viewer::makeHorizontallyCompressible(m_sourceLabel);
        objectLayout->addWidget(m_sourceLabel);
        layout->addWidget(m_objectGroup);

        m_statisticsGroup = new QGroupBox(this);
        auto* statisticsGrid = new QGridLayout(m_statisticsGroup);
        statisticsGrid->setContentsMargins(8, 20, 8, 8);
        statisticsGrid->setHorizontalSpacing(5);
        statisticsGrid->setVerticalSpacing(3);
        robot_qt_viewer::configureInspectorGrid(statisticsGrid);
        std::array<QLabel**, 4> valueTargets{ {
            &m_heightValue,
            &m_maximumDiameterValue,
            &m_minimumDiameterValue,
            &m_axisValue
        } };
        QFont compactStatisticsFont = m_statisticsGroup->font();
        if(compactStatisticsFont.pointSize() > 0) {
            compactStatisticsFont.setPointSize(
                std::max(8, compactStatisticsFont.pointSize() - 1));
        }
        for(std::size_t index = 0; index < valueTargets.size(); ++index) {
            m_statisticsLabels[index] = new QLabel(m_statisticsGroup);
            *valueTargets[index] = new QLabel(m_statisticsGroup);
            m_statisticsLabels[index]->setFont(compactStatisticsFont);
            (*valueTargets[index])->setFont(compactStatisticsFont);
            (*valueTargets[index])->setWordWrap(false);
            robot_qt_viewer::makeHorizontallyCompressible(*valueTargets[index]);
            const int row = static_cast<int>(index / 2);
            const int column = static_cast<int>((index % 2) * 2);
            statisticsGrid->addWidget(m_statisticsLabels[index], row, column);
            statisticsGrid->addWidget(*valueTargets[index], row, column + 1);
        }
        statisticsGrid->setColumnStretch(1, 1);
        statisticsGrid->setColumnStretch(3, 1);
        layout->addWidget(m_statisticsGroup);

        m_alignmentGroup = new QGroupBox(this);
        auto* alignmentLayout = new QVBoxLayout(m_alignmentGroup);
        alignmentLayout->setContentsMargins(8, 20, 8, 8);
        alignmentLayout->setSpacing(6);
        m_errorLabel = new QLabel(m_alignmentGroup);
        m_errorLabel->setObjectName(QStringLiteral("rotationBodyModel.error"));
        m_errorLabel->setWordWrap(true);
        m_errorLabel->setProperty("validationError", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_errorLabel);
        alignmentLayout->addWidget(m_errorLabel);
        auto* alignmentActions = new QHBoxLayout();
        alignmentActions->setContentsMargins(0, 0, 0, 0);
        alignmentActions->setSpacing(4);
        m_flipButton = new QToolButton(m_alignmentGroup);
        m_flipButton->setObjectName(QStringLiteral("rotationBodyModel.flip"));
        m_flipButton->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
        m_flipButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        m_flipButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_flipButton->setMinimumHeight(34);
        alignmentActions->addWidget(m_flipButton, 1);
        m_resetButton = new QToolButton(m_alignmentGroup);
        m_resetButton->setObjectName(QStringLiteral("rotationBodyModel.reset"));
        m_resetButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
        m_resetButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        m_resetButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_resetButton->setMinimumHeight(34);
        alignmentActions->addWidget(m_resetButton, 1);
        alignmentLayout->addLayout(alignmentActions);
        layout->addWidget(m_alignmentGroup);

        const auto createTransformGroup = [this](
            QGroupBox*& group,
            std::array<QLabel*, 6>& labels,
            std::array<QDoubleSpinBox*, 6>& fields,
            const QString& objectPrefix) {
            group = new QGroupBox(this);
            auto* grid = new QGridLayout(group);
            grid->setContentsMargins(8, 20, 8, 8);
            grid->setHorizontalSpacing(4);
            grid->setVerticalSpacing(6);
            robot_qt_viewer::configureInspectorGrid(grid);
            for(std::size_t index = 0; index < fields.size(); ++index) {
                labels[index] = new QLabel(group);
                fields[index] = index < 3
                    ? makeLengthSpin(
                        objectPrefix + QLatin1Char('.') + QString::fromLatin1(componentNames[index]),
                        group)
                    : makeAngleSpin(
                        objectPrefix + QLatin1Char('.') + QString::fromLatin1(componentNames[index]),
                        group);
                const int row = static_cast<int>(index / 3);
                const int column = static_cast<int>((index % 3) * 2);
                grid->addWidget(labels[index], row, column);
                grid->addWidget(fields[index], row, column + 1);
            }
            grid->setColumnStretch(1, 1);
            grid->setColumnStretch(3, 1);
            grid->setColumnStretch(5, 1);
        };
        createTransformGroup(
            m_adjustmentGroup,
            m_adjustmentLabels,
            m_adjustmentFields,
            QStringLiteral("rotationBodyModel.adjustment"));
        layout->addWidget(m_adjustmentGroup);
        createTransformGroup(
            m_baseFrameGroup,
            m_baseFrameLabels,
            m_baseFrameFields,
            QStringLiteral("rotationBodyModel.base"));
        layout->addWidget(m_baseFrameGroup);

        m_publishGroup = new QGroupBox(this);
        auto* publishLayout = new QVBoxLayout(m_publishGroup);
        publishLayout->setContentsMargins(8, 20, 8, 8);
        publishLayout->setSpacing(8);
        auto* publishRow = new QHBoxLayout();
        publishRow->setContentsMargins(0, 0, 0, 0);
        publishRow->setSpacing(4);
        m_publishButtonGroup = new QButtonGroup(this);
        m_publishButtonGroup->setExclusive(true);
        m_publishBaseButton = new QToolButton(m_publishGroup);
        m_publishBaseButton->setObjectName(QStringLiteral("rotationBodyModel.publishBase"));
        configureToolSegment(m_publishBaseButton);
        m_publishBaseButton->setChecked(true);
        m_publishButtonGroup->addButton(m_publishBaseButton, 0);
        publishRow->addWidget(m_publishBaseButton);
        m_publishLocalButton = new QToolButton(m_publishGroup);
        m_publishLocalButton->setObjectName(QStringLiteral("rotationBodyModel.publishLocal"));
        configureToolSegment(m_publishLocalButton);
        m_publishButtonGroup->addButton(m_publishLocalButton, 1);
        publishRow->addWidget(m_publishLocalButton);
        publishLayout->addLayout(publishRow);
        m_confirmFrameButton = new QPushButton(m_publishGroup);
        m_confirmFrameButton->setObjectName(QStringLiteral("rotationBodyModel.confirmFrame"));
        m_confirmFrameButton->setIcon(style()->standardIcon(QStyle::SP_DialogApplyButton));
        robot_qt_viewer::configureInspectorButton(m_confirmFrameButton);
        m_confirmFrameButton->setMinimumHeight(34);
        publishLayout->addWidget(m_confirmFrameButton);
        layout->addWidget(m_publishGroup);
        layout->addStretch(1);

        connect(
            m_objectTypeGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int) { updateSimulationControls(); });
        connect(
            m_rotationAxisCombo,
            static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            [this](int) { constrainAxisChoices(true); });
        connect(
            m_toothAxisCombo,
            static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,
            [this](int) { constrainAxisChoices(false); });
        connect(m_diameterSpin, static_cast<void(QDoubleSpinBox::*)(double)>(
            &QDoubleSpinBox::valueChanged), this, [this](double) {
                if(!m_updating) {
                    updateSimulationControls();
                }
            });
        connect(m_importButton, &QToolButton::clicked, this, &ModelTransformPanel::chooseAndRequestImport);
        connect(m_flipButton, &QToolButton::clicked, this, &ModelTransformPanel::flipRequested);
        connect(m_resetButton, &QToolButton::clicked, this, &ModelTransformPanel::resetRequested);
        connect(
            m_publishButtonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                if(!m_updating) {
                    emit publishFrameChanged(
                        id == 1 ? PublishFrame::PlanningLocalFrame : PublishFrame::BaseFrame);
                }
            });
        connect(m_confirmFrameButton, &QPushButton::clicked, this, &ModelTransformPanel::confirmFrameRequested);
        connectTransformFields(m_adjustmentFields, true);
        connectTransformFields(m_baseFrameFields, false);

        constrainAxisChoices(true);
        retranslate();
        setViewModel(m_viewModel);
    }

    QDoubleSpinBox* ModelTransformPanel::makeLengthSpin(
        const QString& objectName,
        QWidget* parent)
    {
        auto* spin = new QDoubleSpinBox(parent);
        spin->setObjectName(objectName);
        spin->setRange(-1000000.0, 1000000.0);
        spin->setDecimals(3);
        spin->setSingleStep(0.1);
        spin->setKeyboardTracking(false);
        spin->setSuffix(QStringLiteral(" mm"));
        robot_qt_viewer::makeHorizontallyCompressible(spin);
        return spin;
    }

    QDoubleSpinBox* ModelTransformPanel::makeAngleSpin(
        const QString& objectName,
        QWidget* parent)
    {
        auto* spin = new QDoubleSpinBox(parent);
        spin->setObjectName(objectName);
        spin->setRange(-36000.0, 36000.0);
        spin->setDecimals(3);
        spin->setSingleStep(0.1);
        spin->setKeyboardTracking(false);
        spin->setSuffix(QStringLiteral(" \u00b0"));
        robot_qt_viewer::makeHorizontallyCompressible(spin);
        return spin;
    }

    domain::TransformComponents ModelTransformPanel::readTransformComponents(
        const std::array<QDoubleSpinBox*, 6>& fields)
    {
        domain::TransformComponents result;
        for(int index = 0; index < 3; ++index) {
            result.translationMeters[index] =
                domain::millimetersToMeters(fields[static_cast<std::size_t>(index)]->value());
            result.rollPitchYawRadians[index] =
                domain::degreesToRadians(fields[static_cast<std::size_t>(index + 3)]->value());
        }
        return result;
    }

    void ModelTransformPanel::setTransformComponents(
        const std::array<QDoubleSpinBox*, 6>& fields,
        const domain::TransformComponents& components)
    {
        std::vector<std::unique_ptr<QSignalBlocker>> blockers;
        blockers.reserve(fields.size());
        for(QDoubleSpinBox* field : fields) {
            blockers.push_back(std::make_unique<QSignalBlocker>(field));
        }
        for(int index = 0; index < 3; ++index) {
            fields[static_cast<std::size_t>(index)]->setValue(
                domain::metersToMillimeters(components.translationMeters[index]));
            fields[static_cast<std::size_t>(index + 3)]->setValue(
                domain::radiansToDegrees(components.rollPitchYawRadians[index]));
        }
    }

    domain::SignedAxis ModelTransformPanel::comboAxis(const QComboBox* combo)
    {
        return static_cast<domain::SignedAxis>(combo->currentData().toInt());
    }

    void ModelTransformPanel::setComboAxis(QComboBox* combo, domain::SignedAxis axis)
    {
        const int index = combo->findData(static_cast<int>(axis));
        if(index >= 0) {
            combo->setCurrentIndex(index);
        }
    }

    void ModelTransformPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) {
            return;
        }
        m_languageCode = canonical;
        retranslate();
        updateReadOnlyValues();
        updateSimulationControls();
    }

    QString ModelTransformPanel::languageCode() const
    {
        return m_languageCode;
    }

    void ModelTransformPanel::setViewModel(const RotationBodyPlanningViewModel& viewModel)
    {
        m_updating = true;
        m_viewModel = viewModel;
        if(viewModel.hasModel) {
            const QSignalBlocker completeBlocker(m_completePartButton);
            const QSignalBlocker simulationBlocker(m_simulationBlockButton);
            const QSignalBlocker rotationBlocker(m_rotationAxisCombo);
            const QSignalBlocker toothBlocker(m_toothAxisCombo);
            const QSignalBlocker diameterBlocker(m_diameterSpin);
            m_completePartButton->setChecked(
                viewModel.objectType == domain::PlanningObjectType::CompletePart);
            m_simulationBlockButton->setChecked(
                viewModel.objectType == domain::PlanningObjectType::SimulationBlock);
            setComboAxis(m_rotationAxisCombo, viewModel.originalRotationAxis);
            setComboAxis(m_toothAxisCombo, viewModel.toothOutwardAxis);
            if(viewModel.motherMaximumDiameterMeters > 0.0) {
                m_diameterSpin->setValue(
                    domain::metersToMillimeters(viewModel.motherMaximumDiameterMeters));
            }
        }
        {
            const QSignalBlocker baseBlocker(m_publishBaseButton);
            const QSignalBlocker localBlocker(m_publishLocalButton);
            m_publishBaseButton->setChecked(viewModel.publishFrame == PublishFrame::BaseFrame);
            m_publishLocalButton->setChecked(
                viewModel.publishFrame == PublishFrame::PlanningLocalFrame);
        }
        setTransformComponents(m_adjustmentFields, viewModel.planningDelta);
        setTransformComponents(m_baseFrameFields, viewModel.baseFromPlanning);
        m_updating = false;
        constrainAxisChoices(true);
        updateReadOnlyValues();
        updateSimulationControls();
        updateEnabledState();
    }

    const RotationBodyPlanningViewModel& ModelTransformPanel::viewModel() const noexcept
    {
        return m_viewModel;
    }

    void ModelTransformPanel::setImportDirectory(const QString& directory)
    {
        m_importDirectory = QDir::cleanPath(directory);
    }

    QString ModelTransformPanel::importDirectory() const
    {
        return m_importDirectory;
    }

    RotationBodyImportOptions ModelTransformPanel::importOptions() const
    {
        RotationBodyImportOptions options;
        options.objectType = m_simulationBlockButton->isChecked()
            ? domain::PlanningObjectType::SimulationBlock
            : domain::PlanningObjectType::CompletePart;
        options.originalRotationAxis = comboAxis(m_rotationAxisCombo);
        options.toothOutwardAxis = comboAxis(m_toothAxisCombo);
        options.motherMaximumDiameterMeters =
            domain::millimetersToMeters(m_diameterSpin->value());
        return options;
    }

    bool ModelTransformPanel::importConfigurationValid() const
    {
        const RotationBodyImportOptions options = importOptions();
        return options.objectType == domain::PlanningObjectType::CompletePart ||
            (options.motherMaximumDiameterMeters > 0.0 &&
                domain::arePerpendicular(
                    options.originalRotationAxis,
                    options.toothOutwardAxis));
    }

    void ModelTransformPanel::connectTransformFields(
        const std::array<QDoubleSpinBox*, 6>& fields,
        bool planningFields)
    {
        for(QDoubleSpinBox* field : fields) {
            connect(
                field,
                static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                this,
                [this, fields, planningFields](double) {
                    if(m_updating) {
                        return;
                    }
                    const domain::TransformComponents components =
                        readTransformComponents(fields);
                    if(planningFields) {
                        emit planningDeltaEdited(components);
                    } else {
                        emit baseTransformEdited(components);
                    }
                });
        }
    }

    void ModelTransformPanel::updateSimulationControls()
    {
        const bool simulation = m_simulationBlockButton->isChecked();
        m_simulationControls->setVisible(simulation);
        const bool valid = importConfigurationValid();
        m_axisValidationLabel->setText(
            RotationBodyPlanningTranslations::text(
                m_languageCode,
                "simulation.axes_invalid"));
        m_axisValidationLabel->setVisible(!valid);
        m_importButton->setEnabled(valid && !m_viewModel.isBusy);
    }

    void ModelTransformPanel::constrainAxisChoices(bool rotationAxisChanged)
    {
        QSignalBlocker rotationBlocker(m_rotationAxisCombo);
        QSignalBlocker toothBlocker(m_toothAxisCombo);
        domain::SignedAxis rotationAxis = comboAxis(m_rotationAxisCombo);
        domain::SignedAxis toothAxis = comboAxis(m_toothAxisCombo);
        QComboBox* constrained = rotationAxisChanged ? m_toothAxisCombo : m_rotationAxisCombo;
        const domain::SignedAxis reference = rotationAxisChanged ? rotationAxis : toothAxis;
        for(int index = 0; index < constrained->count(); ++index) {
            const domain::SignedAxis candidate =
                static_cast<domain::SignedAxis>(constrained->itemData(index).toInt());
            setItemEnabled(constrained, index, domain::arePerpendicular(candidate, reference));
        }
        if(!domain::arePerpendicular(rotationAxis, toothAxis)) {
            for(int index = 0; index < constrained->count(); ++index) {
                const domain::SignedAxis candidate =
                    static_cast<domain::SignedAxis>(constrained->itemData(index).toInt());
                if(domain::arePerpendicular(candidate, reference)) {
                    constrained->setCurrentIndex(index);
                    break;
                }
            }
        }
        rotationAxis = comboAxis(m_rotationAxisCombo);
        toothAxis = comboAxis(m_toothAxisCombo);
        for(int index = 0; index < m_rotationAxisCombo->count(); ++index) {
            const domain::SignedAxis candidate = static_cast<domain::SignedAxis>(
                m_rotationAxisCombo->itemData(index).toInt());
            setItemEnabled(
                m_rotationAxisCombo,
                index,
                domain::arePerpendicular(candidate, toothAxis));
        }
        for(int index = 0; index < m_toothAxisCombo->count(); ++index) {
            const domain::SignedAxis candidate = static_cast<domain::SignedAxis>(
                m_toothAxisCombo->itemData(index).toInt());
            setItemEnabled(
                m_toothAxisCombo,
                index,
                domain::arePerpendicular(candidate, rotationAxis));
        }
        updateSimulationControls();
    }

    void ModelTransformPanel::updateReadOnlyValues()
    {
        m_stageLabel->setText(
            RotationBodyPlanningTranslations::stageName(m_languageCode, m_viewModel.stage));
        const QString unavailable =
            RotationBodyPlanningTranslations::text(m_languageCode, "statistics.unavailable");
        if(!m_viewModel.hasModel) {
            m_sourceLabel->setText(
                RotationBodyPlanningTranslations::text(m_languageCode, "model.no_source"));
            m_sourceLabel->setToolTip({});
            m_heightValue->setText(unavailable);
            m_maximumDiameterValue->setText(unavailable);
            m_minimumDiameterValue->setText(unavailable);
            m_axisValue->setText(unavailable);
        } else {
            const QString sourcePath = QString::fromStdString(m_viewModel.sourcePath);
            m_sourceLabel->setText(QFileInfo(sourcePath).fileName());
            m_sourceLabel->setToolTip(sourcePath);
            m_heightValue->setText(millimeterText(m_viewModel.statistics.heightMeters));
            m_maximumDiameterValue->setText(
                millimeterText(m_viewModel.statistics.maximumDiameterMeters));
            m_minimumDiameterValue->setText(
                millimeterText(m_viewModel.statistics.minimumDiameterMeters));
            const Eigen::Vector3d axis = m_viewModel.statistics.estimatedAxisInMesh;
            m_axisValue->setText(QStringLiteral("[%1, %2, %3]")
                .arg(axis.x(), 0, 'f', 3)
                .arg(axis.y(), 0, 'f', 3)
                .arg(axis.z(), 0, 'f', 3));
        }
        const QString error = RotationBodyPlanningTranslations::errorText(
            m_languageCode,
            m_viewModel.errorCode);
        m_errorLabel->setText(error);
        m_errorLabel->setVisible(!error.isEmpty());
    }

    void ModelTransformPanel::updateEnabledState()
    {
        const bool transformEnabled = m_viewModel.canEditModelTransform;
        m_objectGroup->setEnabled(!m_viewModel.isBusy);
        m_flipButton->setEnabled(transformEnabled);
        m_resetButton->setEnabled(transformEnabled);
        m_adjustmentGroup->setEnabled(transformEnabled);
        m_baseFrameGroup->setEnabled(m_viewModel.hasModel && !m_viewModel.isBusy);
        m_publishGroup->setEnabled(m_viewModel.hasModel && !m_viewModel.isBusy);
        m_confirmFrameButton->setEnabled(m_viewModel.canConfirmFrame);
    }

    void ModelTransformPanel::retranslate()
    {
        const auto translated = [this](const char* key) {
            return RotationBodyPlanningTranslations::text(m_languageCode, key);
        };
        m_objectGroup->setTitle(translated("model.planning_object"));
        m_completePartButton->setText(translated("model.complete_part"));
        m_simulationBlockButton->setText(translated("model.simulation_block"));
        m_diameterLabel->setText(translated("simulation.maximum_diameter"));
        m_rotationAxisLabel->setText(translated("simulation.rotation_axis"));
        m_toothAxisLabel->setText(translated("simulation.tooth_outward"));
        const QString axisTooltip = translated("simulation.axis_tooltip");
        m_rotationAxisCombo->setToolTip(axisTooltip);
        m_toothAxisCombo->setToolTip(axisTooltip);
        for(int index = 0; index < m_rotationAxisCombo->count(); ++index) {
            const domain::SignedAxis axis = static_cast<domain::SignedAxis>(
                m_rotationAxisCombo->itemData(index).toInt());
            const QString name = RotationBodyPlanningTranslations::axisName(m_languageCode, axis);
            m_rotationAxisCombo->setItemText(index, name);
            m_toothAxisCombo->setItemText(index, name);
        }
        m_importButton->setText(translated("model.load"));
        m_importButton->setToolTip(translated("model.load_tooltip"));
        m_statisticsGroup->setTitle(translated("statistics.title"));
        constexpr std::array<const char*, 4> statisticKeys{ {
            "statistics.height",
            "statistics.maximum_diameter",
            "statistics.minimum_diameter",
            "statistics.axis"
        } };
        for(std::size_t index = 0; index < statisticKeys.size(); ++index) {
            m_statisticsLabels[index]->setText(translated(statisticKeys[index]));
        }
        m_alignmentGroup->setTitle(translated("alignment.title"));
        m_flipButton->setText(translated("alignment.flip"));
        m_flipButton->setToolTip(translated("alignment.flip_tooltip"));
        m_flipButton->setAccessibleName(translated("alignment.flip"));
        m_resetButton->setText(translated("alignment.reset"));
        m_resetButton->setToolTip(translated("alignment.reset_tooltip"));
        m_resetButton->setAccessibleName(translated("alignment.reset"));
        m_adjustmentGroup->setTitle(translated("transform.adjustment"));
        m_baseFrameGroup->setTitle(translated("transform.base_frame"));
        for(std::size_t index = 0; index < componentNames.size(); ++index) {
            const QString label = QString::fromLatin1(componentNames[index]);
            m_adjustmentLabels[index]->setText(label);
            m_baseFrameLabels[index]->setText(label);
        }
        m_publishGroup->setTitle(translated("publish.title"));
        m_publishBaseButton->setText(translated("publish.base"));
        m_publishLocalButton->setText(translated("publish.local"));
        m_confirmFrameButton->setText(translated("publish.confirm_frame"));
        m_confirmFrameButton->setToolTip(translated("publish.confirm_frame_tooltip"));
    }

    void ModelTransformPanel::chooseAndRequestImport()
    {
        if(!importConfigurationValid()) {
            m_axisValidationLabel->setText(
                RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "simulation.axes_invalid"));
            return;
        }
        const QString sourcePath = QFileDialog::getOpenFileName(
            this,
            RotationBodyPlanningTranslations::text(m_languageCode, "model.dialog_title"),
            m_importDirectory,
            RotationBodyPlanningTranslations::text(m_languageCode, "model.dialog_filter"));
        if(sourcePath.isEmpty()) {
            return;
        }
        m_importDirectory = QFileInfo(sourcePath).absolutePath();
        emit importRequested(sourcePath, importOptions());
    }
}
