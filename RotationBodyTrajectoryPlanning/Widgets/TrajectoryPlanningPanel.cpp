#include "TrajectoryPlanningPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include "RobotQtWidgetUtils.h"

#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStyle>
#include <QVBoxLayout>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        constexpr double pi = 3.14159265358979323846;

        QDoubleSpinBox* makeSpin(
            const QString& objectName,
            double minimum,
            double maximum,
            int decimals,
            const QString& suffix,
            QWidget* parent)
        {
            auto* spin = new QDoubleSpinBox(parent);
            spin->setObjectName(objectName);
            spin->setRange(minimum, maximum);
            spin->setDecimals(decimals);
            spin->setSuffix(suffix);
            spin->setKeyboardTracking(false);
            return spin;
        }

        QString passDisplayName(
            const QString& languageCode,
            const domain::TrajectoryPass& pass)
        {
            return RotationBodyPlanningTranslations::text(languageCode, "trajectory.pass_summary")
                .arg(pass.order)
                .arg(static_cast<qulonglong>(pass.trajectory.linearPoints.size()))
                .arg(pass.trajectory.metrics.durationSeconds, 0, 'f', 3);
        }
    }

    TrajectoryPlanningPanel::TrajectoryPlanningPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyTrajectoryPlanningPanel"));
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        m_generationGroup = new QGroupBox(this);
        auto* generationForm = new QFormLayout(m_generationGroup);
        generationForm->setContentsMargins(8, 20, 8, 8);
        generationForm->setSpacing(6);
        robot_qt_viewer::configureInspectorForm(generationForm);
        for(QLabel*& label : m_parameterLabels) {
            label = new QLabel(m_generationGroup);
        }
        m_sprayDistanceSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.sprayDistance"),
            0.001, 100000.0, 3, QStringLiteral(" mm"), m_generationGroup);
        m_sprayDistanceSpin->setValue(100.0);
        m_tiltSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.tilt"),
            -79.999, 79.999, 3, QStringLiteral(" deg"), m_generationGroup);
        m_speedSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.speed"),
            0.001, 100000.0, 3, QStringLiteral(" mm/s"), m_generationGroup);
        m_speedSpin->setValue(6.0);
        m_startExtensionSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.startExtension"),
            0.0, 100000.0, 3, QStringLiteral(" mm"), m_generationGroup);
        m_endExtensionSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.endExtension"),
            0.0, 100000.0, 3, QStringLiteral(" mm"), m_generationGroup);
        m_positionerRpmSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.positionerRpm"),
            -10000.0, 10000.0, 4, QStringLiteral(" rpm"), m_generationGroup);
        generationForm->addRow(m_parameterLabels[0], m_sprayDistanceSpin);
        generationForm->addRow(m_parameterLabels[1], m_tiltSpin);
        generationForm->addRow(m_parameterLabels[2], m_speedSpin);
        generationForm->addRow(m_parameterLabels[3], m_startExtensionSpin);
        generationForm->addRow(m_parameterLabels[4], m_endExtensionSpin);
        generationForm->addRow(m_parameterLabels[5], m_positionerRpmSpin);
        auto* generationButtons = new QHBoxLayout();
        generationButtons->setContentsMargins(0, 0, 0, 0);
        generationButtons->setSpacing(5);
        m_reopenBoundaryButton = new QPushButton(m_generationGroup);
        m_reopenBoundaryButton->setObjectName(QStringLiteral("rotationBodyTrajectory.reopenBoundary"));
        m_reopenBoundaryButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
        robot_qt_viewer::configureInspectorButton(m_reopenBoundaryButton);
        generationButtons->addWidget(m_reopenBoundaryButton);
        m_generateButton = new QPushButton(m_generationGroup);
        m_generateButton->setObjectName(QStringLiteral("rotationBodyTrajectory.generate"));
        m_generateButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
        robot_qt_viewer::configureInspectorButton(m_generateButton);
        generationButtons->addWidget(m_generateButton);
        m_swapButton = new QPushButton(m_generationGroup);
        m_swapButton->setObjectName(QStringLiteral("rotationBodyTrajectory.swap"));
        m_swapButton->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
        robot_qt_viewer::configureInspectorButton(m_swapButton);
        generationButtons->addWidget(m_swapButton);
        generationForm->addRow(generationButtons);
        m_displayModeButton = new QPushButton(m_generationGroup);
        m_displayModeButton->setObjectName(
            QStringLiteral("rotationBodyTrajectory.displayMode"));
        robot_qt_viewer::configureInspectorButton(m_displayModeButton);
        generationForm->addRow(m_displayModeButton);
        layout->addWidget(m_generationGroup);

        m_metricsGroup = new QGroupBox(this);
        auto* metricsForm = new QFormLayout(m_metricsGroup);
        metricsForm->setContentsMargins(8, 20, 8, 8);
        metricsForm->setSpacing(5);
        robot_qt_viewer::configureInspectorForm(metricsForm);
        for(std::size_t index = 0; index < m_metricCaptions.size(); ++index) {
            m_metricCaptions[index] = new QLabel(m_metricsGroup);
            m_metricValues[index] = new QLabel(QStringLiteral("--"), m_metricsGroup);
            metricsForm->addRow(m_metricCaptions[index], m_metricValues[index]);
        }
        layout->addWidget(m_metricsGroup);

        m_pointsGroup = new QGroupBox(this);
        auto* pointsLayout = new QVBoxLayout(m_pointsGroup);
        pointsLayout->setContentsMargins(8, 20, 8, 8);
        pointsLayout->setSpacing(6);
        m_pointList = new QListWidget(m_pointsGroup);
        m_pointList->setObjectName(QStringLiteral("rotationBodyTrajectory.pointList"));
        m_pointList->setSelectionMode(QAbstractItemView::ExtendedSelection);
        m_pointList->setMinimumHeight(180);
        robot_qt_viewer::configureInspectorList(m_pointList, true, false);
        pointsLayout->addWidget(m_pointList);
        auto* interpolationRow = new QHBoxLayout();
        interpolationRow->setContentsMargins(0, 0, 0, 0);
        interpolationRow->setSpacing(5);
        m_interpolationLabel = new QLabel(m_pointsGroup);
        interpolationRow->addWidget(m_interpolationLabel);
        m_interpolationSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.interpolationInterval"),
            0.000001, 10000.0, 6, QStringLiteral(" s"), m_pointsGroup);
        m_interpolationSpin->setValue(0.01);
        interpolationRow->addWidget(m_interpolationSpin, 1);
        pointsLayout->addLayout(interpolationRow);
        m_interpolateButton = new QPushButton(m_pointsGroup);
        m_interpolateButton->setObjectName(QStringLiteral("rotationBodyTrajectory.interpolate"));
        robot_qt_viewer::configureInspectorButton(m_interpolateButton);
        pointsLayout->addWidget(m_interpolateButton);
        layout->addWidget(m_pointsGroup);

        m_transformGroup = new QGroupBox(this);
        auto* transformGrid = new QGridLayout(m_transformGroup);
        transformGrid->setContentsMargins(8, 20, 8, 8);
        transformGrid->setHorizontalSpacing(5);
        transformGrid->setVerticalSpacing(5);
        const std::array<const char*, 6> names{ { "X", "Y", "Z", "Rx", "Ry", "Rz" } };
        for(std::size_t index = 0; index < names.size(); ++index) {
            m_transformLabels[index] = new QLabel(QString::fromLatin1(names[index]), m_transformGroup);
            m_transformSpins[index] = makeSpin(
                QStringLiteral("rotationBodyTrajectory.pointDelta.%1")
                    .arg(QString::fromLatin1(names[index])),
                index < 3 ? -100000.0 : -3600.0,
                index < 3 ? 100000.0 : 3600.0,
                4,
                index < 3 ? QStringLiteral(" mm") : QStringLiteral(" deg"),
                m_transformGroup);
            const int row = static_cast<int>(index / 3);
            const int column = static_cast<int>((index % 3) * 2);
            transformGrid->addWidget(m_transformLabels[index], row, column);
            transformGrid->addWidget(m_transformSpins[index], row, column + 1);
        }
        m_applyTransformButton = new QPushButton(m_transformGroup);
        m_applyTransformButton->setObjectName(QStringLiteral("rotationBodyTrajectory.applyPointTransform"));
        robot_qt_viewer::configureInspectorButton(m_applyTransformButton);
        transformGrid->addWidget(m_applyTransformButton, 2, 0, 1, 6);
        layout->addWidget(m_transformGroup);

        m_groupGroup = new QGroupBox(this);
        auto* groupLayout = new QVBoxLayout(m_groupGroup);
        groupLayout->setContentsMargins(8, 20, 8, 8);
        groupLayout->setSpacing(6);
        auto* groupCommandRow = new QHBoxLayout();
        groupCommandRow->setContentsMargins(0, 0, 0, 0);
        groupCommandRow->setSpacing(5);
        m_newButton = new QPushButton(m_groupGroup);
        m_editButton = new QPushButton(m_groupGroup);
        m_removeButton = new QPushButton(m_groupGroup);
        m_saveToGroupButton = new QPushButton(m_groupGroup);
        m_newButton->setObjectName(QStringLiteral("rotationBodyTrajectory.new"));
        m_editButton->setObjectName(QStringLiteral("rotationBodyTrajectory.edit"));
        m_removeButton->setObjectName(QStringLiteral("rotationBodyTrajectory.remove"));
        m_saveToGroupButton->setObjectName(QStringLiteral("rotationBodyTrajectory.saveToGroup"));
        for(QPushButton* button : { m_newButton, m_editButton, m_removeButton, m_saveToGroupButton }) {
            robot_qt_viewer::configureInspectorButton(button);
            groupCommandRow->addWidget(button);
        }
        groupLayout->addLayout(groupCommandRow);
        m_passList = new QListWidget(m_groupGroup);
        m_passList->setObjectName(QStringLiteral("rotationBodyTrajectory.passList"));
        m_passList->setSelectionMode(QAbstractItemView::SingleSelection);
        m_passList->setMinimumHeight(120);
        robot_qt_viewer::configureInspectorList(m_passList, true, true);
        groupLayout->addWidget(m_passList);
        auto* transitionRow = new QHBoxLayout();
        transitionRow->setContentsMargins(0, 0, 0, 0);
        transitionRow->setSpacing(5);
        m_transitionLabel = new QLabel(m_groupGroup);
        transitionRow->addWidget(m_transitionLabel);
        m_transitionSpin = makeSpin(
            QStringLiteral("rotationBodyTrajectory.transitionAfter"),
            0.0, 10000.0, 6, QStringLiteral(" s"), m_groupGroup);
        transitionRow->addWidget(m_transitionSpin, 1);
        groupLayout->addLayout(transitionRow);
        layout->addWidget(m_groupGroup);
        layout->addStretch(1);

        connect(m_reopenBoundaryButton, &QPushButton::clicked,
            this, &TrajectoryPlanningPanel::reopenBoundaryRequested);
        connect(m_generateButton, &QPushButton::clicked, this, [this]() {
            emit generateRequested(inputParameters());
        });
        connect(m_swapButton, &QPushButton::clicked,
            this, &TrajectoryPlanningPanel::swapDirectionRequested);
        connect(m_displayModeButton, &QPushButton::clicked, this, [this]() {
            const domain::TrajectoryDisplayMode nextMode =
                m_viewModel.trajectoryWorkspace.displayMode ==
                    domain::TrajectoryDisplayMode::Linear
                ? domain::TrajectoryDisplayMode::RelativeHelical
                : domain::TrajectoryDisplayMode::Linear;
            emit displayModeChanged(nextMode);
        });
        connect(m_pointList, &QListWidget::itemSelectionChanged, this, [this]() {
            emit pointSelectionChanged(selectedPointIndices());
            updateEnabledState();
        });
        connect(m_interpolateButton, &QPushButton::clicked, this, [this]() {
            emit interpolateRequested(selectedPointIndices(), m_interpolationSpin->value());
        });
        connect(m_applyTransformButton, &QPushButton::clicked, this, [this]() {
            emit transformPointsRequested(selectedPointIndices(), transformDelta());
        });
        connect(m_newButton, &QPushButton::clicked,
            this, &TrajectoryPlanningPanel::beginNewTrajectoryRequested);
        connect(m_saveToGroupButton, &QPushButton::clicked,
            this, &TrajectoryPlanningPanel::saveCurrentTrajectoryRequested);
        connect(m_editButton, &QPushButton::clicked, this, [this]() {
            const std::string id = selectedPassId();
            if(!id.empty()) emit loadTrajectoryRequested(id);
        });
        connect(m_removeButton, &QPushButton::clicked, this, [this]() {
            const std::string id = selectedPassId();
            if(!id.empty()) emit removeTrajectoryRequested(id);
        });
        connect(m_passList, &QListWidget::itemSelectionChanged, this, [this]() {
            if(m_updating) return;
            const std::string id = selectedPassId();
            const auto found = std::find_if(
                m_viewModel.trajectoryWorkspace.group.passes.begin(),
                m_viewModel.trajectoryWorkspace.group.passes.end(),
                [&](const domain::TrajectoryPass& pass) { return pass.id == id; });
            QSignalBlocker blocker(m_transitionSpin);
            m_transitionSpin->setValue(found == m_viewModel.trajectoryWorkspace.group.passes.end()
                ? 0.0
                : found->transitionAfterSeconds);
            updateEnabledState();
        });
        connect(m_passList, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
            if(m_updating || item == nullptr) return;
            emit trajectoryVisibilityChanged(
                item->data(Qt::UserRole).toString().toStdString(),
                item->checkState() == Qt::Checked);
        });
        connect(
            m_transitionSpin,
            static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this,
            [this](double seconds) {
                if(m_updating) return;
                const std::string id = selectedPassId();
                if(!id.empty()) emit trajectoryTransitionChanged(id, seconds);
            });

        retranslate();
        updateEnabledState();
    }

    void TrajectoryPlanningPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) return;
        m_languageCode = canonical;
        retranslate();
        rebuildPointList();
        rebuildPassList();
    }

    QString TrajectoryPlanningPanel::languageCode() const
    {
        return m_languageCode;
    }

    void TrajectoryPlanningPanel::setViewModel(
        const RotationBodyPlanningViewModel& viewModel)
    {
        m_updating = true;
        m_viewModel = viewModel;
        const domain::TrajectoryGenerationParameters& parameters =
            viewModel.trajectoryWorkspace.parameters;
        m_sprayDistanceSpin->setValue(domain::metersToMillimeters(parameters.sprayDistanceMeters));
        m_tiltSpin->setValue(parameters.tiltRadians * 180.0 / pi);
        m_speedSpin->setValue(domain::metersToMillimeters(parameters.speedMetersPerSecond));
        m_startExtensionSpin->setValue(domain::metersToMillimeters(parameters.startExtensionMeters));
        m_endExtensionSpin->setValue(domain::metersToMillimeters(parameters.endExtensionMeters));
        m_positionerRpmSpin->setValue(parameters.positionerRpm);
        rebuildPointList();
        rebuildPassList();
        if(viewModel.trajectoryWorkspace.currentTrajectory) {
            const domain::PlannedTrajectory& trajectory =
                *viewModel.trajectoryWorkspace.currentTrajectory;
            m_metricValues[0]->setText(QString::number(
                static_cast<qulonglong>(trajectory.linearPoints.size())));
            m_metricValues[1]->setText(QStringLiteral("%1 s")
                .arg(trajectory.metrics.durationSeconds, 0, 'f', 4));
            m_metricValues[2]->setText(QStringLiteral("%1 - %2 s")
                .arg(trajectory.metrics.minimumPointIntervalSeconds, 0, 'f', 4)
                .arg(trajectory.metrics.maximumPointIntervalSeconds, 0, 'f', 4));
        } else {
            for(QLabel* label : m_metricValues) label->setText(QStringLiteral("--"));
        }
        retranslate();
        m_updating = false;
        updateEnabledState();
    }

    std::vector<std::size_t> TrajectoryPlanningPanel::selectedPointIndices() const
    {
        std::vector<std::size_t> indices;
        for(const QListWidgetItem* item : m_pointList->selectedItems()) {
            indices.push_back(static_cast<std::size_t>(
                item->data(Qt::UserRole).toULongLong()));
        }
        std::sort(indices.begin(), indices.end());
        return indices;
    }

    domain::TrajectoryGenerationParameters TrajectoryPlanningPanel::inputParameters() const
    {
        domain::TrajectoryGenerationParameters result =
            m_viewModel.trajectoryWorkspace.parameters;
        result.sprayDistanceMeters = domain::millimetersToMeters(m_sprayDistanceSpin->value());
        result.tiltRadians = m_tiltSpin->value() * pi / 180.0;
        result.speedMetersPerSecond = domain::millimetersToMeters(m_speedSpin->value());
        result.startExtensionMeters = domain::millimetersToMeters(m_startExtensionSpin->value());
        result.endExtensionMeters = domain::millimetersToMeters(m_endExtensionSpin->value());
        result.pointCount = 0;
        result.positionerRpm = m_positionerRpmSpin->value();
        return result;
    }

    domain::TransformComponents TrajectoryPlanningPanel::transformDelta() const
    {
        domain::TransformComponents result;
        result.translationMeters = Eigen::Vector3d(
            domain::millimetersToMeters(m_transformSpins[0]->value()),
            domain::millimetersToMeters(m_transformSpins[1]->value()),
            domain::millimetersToMeters(m_transformSpins[2]->value()));
        result.rollPitchYawRadians = Eigen::Vector3d(
            m_transformSpins[3]->value() * pi / 180.0,
            m_transformSpins[4]->value() * pi / 180.0,
            m_transformSpins[5]->value() * pi / 180.0);
        return result;
    }

    std::string TrajectoryPlanningPanel::selectedPassId() const
    {
        const QListWidgetItem* item = m_passList->currentItem();
        return item == nullptr
            ? std::string()
            : item->data(Qt::UserRole).toString().toStdString();
    }

    void TrajectoryPlanningPanel::rebuildPointList()
    {
        QSignalBlocker blocker(m_pointList);
        m_pointList->clear();
        if(!m_viewModel.trajectoryWorkspace.currentTrajectory) return;
        const domain::PlannedTrajectory& trajectory =
            *m_viewModel.trajectoryWorkspace.currentTrajectory;
        const std::vector<domain::TrajectoryPosePoint>& points =
            m_viewModel.trajectoryWorkspace.displayMode ==
                    domain::TrajectoryDisplayMode::RelativeHelical
                ? trajectory.relativeHelicalPoints
                : trajectory.linearPoints;
        for(std::size_t index = 0; index < points.size(); ++index) {
            const domain::TrajectoryPosePoint& point = points[index];
            const Eigen::Vector3d euler = point.planningFromTool.linear().eulerAngles(2, 1, 0);
            const Eigen::Vector3d millimeters = point.planningFromTool.translation() * 1000.0;
            const QString marker = index == 0
                ? QStringLiteral("A")
                : (index + 1 == points.size() ? QStringLiteral("B") : QString::number(index + 1));
            const QString text = QStringLiteral(
                "%1  t=%2 s  X=%3 Y=%4 Z=%5 mm  Rx=%6 Ry=%7 Rz=%8 deg")
                .arg(marker)
                .arg(point.timeSeconds, 0, 'f', 6)
                .arg(millimeters.x(), 0, 'f', 3)
                .arg(millimeters.y(), 0, 'f', 3)
                .arg(millimeters.z(), 0, 'f', 3)
                .arg(euler.z() * 180.0 / pi, 0, 'f', 3)
                .arg(euler.y() * 180.0 / pi, 0, 'f', 3)
                .arg(euler.x() * 180.0 / pi, 0, 'f', 3);
            auto* item = new QListWidgetItem(text, m_pointList);
            item->setData(Qt::UserRole, static_cast<qulonglong>(index));
            if(point.interpolated) item->setForeground(QColor(76, 168, 255));
        }
    }

    void TrajectoryPlanningPanel::rebuildPassList()
    {
        const std::string selected = selectedPassId();
        QSignalBlocker blocker(m_passList);
        m_passList->clear();
        for(const domain::TrajectoryPass& pass : m_viewModel.trajectoryWorkspace.group.passes) {
            auto* item = new QListWidgetItem(passDisplayName(m_languageCode, pass), m_passList);
            item->setData(Qt::UserRole, QString::fromStdString(pass.id));
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(pass.visible ? Qt::Checked : Qt::Unchecked);
            if(pass.id == selected ||
                (selected.empty() && pass.id == m_viewModel.trajectoryWorkspace.editingPassId)) {
                m_passList->setCurrentItem(item);
            }
        }
    }

    void TrajectoryPlanningPanel::updateEnabledState()
    {
        const std::size_t selectedPoints = selectedPointIndices().size();
        const bool passSelected = !selectedPassId().empty();
        m_reopenBoundaryButton->setEnabled(m_viewModel.canReopenBoundary);
        m_generateButton->setEnabled(m_viewModel.canGenerateTrajectory);
        m_swapButton->setEnabled(m_viewModel.canEditCurrentTrajectory);
        m_displayModeButton->setEnabled(m_viewModel.canEditCurrentTrajectory);
        m_pointList->setEnabled(m_viewModel.canEditCurrentTrajectory);
        m_interpolateButton->setEnabled(
            m_viewModel.canEditCurrentTrajectory && selectedPoints >= 2);
        m_applyTransformButton->setEnabled(
            m_viewModel.canEditCurrentTrajectory && selectedPoints >= 1);
        m_saveToGroupButton->setEnabled(m_viewModel.canSaveCurrentTrajectory);
        m_editButton->setEnabled(m_viewModel.canEditTrajectoryGroup && passSelected);
        m_removeButton->setEnabled(m_viewModel.canEditTrajectoryGroup && passSelected);
        m_transitionSpin->setEnabled(m_viewModel.canEditTrajectoryGroup && passSelected);
        m_newButton->setEnabled(m_viewModel.canGenerateTrajectory);
    }

    void TrajectoryPlanningPanel::retranslate()
    {
        const auto tr = [this](const char* key) {
            return RotationBodyPlanningTranslations::text(m_languageCode, key);
        };
        m_generationGroup->setTitle(tr("trajectory.generation"));
        const std::array<const char*, 6> parameterKeys{ {
            "trajectory.spray_distance", "trajectory.tilt", "trajectory.speed",
            "trajectory.start_extension", "trajectory.end_extension",
            "trajectory.positioner_rpm" } };
        for(std::size_t index = 0; index < parameterKeys.size(); ++index) {
            m_parameterLabels[index]->setText(tr(parameterKeys[index]));
        }
        m_reopenBoundaryButton->setText(tr("trajectory.reopen_boundary"));
        m_generateButton->setText(tr("trajectory.generate"));
        m_swapButton->setText(tr("trajectory.swap"));
        m_displayModeButton->setText(
            m_viewModel.trajectoryWorkspace.displayMode ==
                    domain::TrajectoryDisplayMode::RelativeHelical
                ? tr("trajectory.show_linear")
                : tr("trajectory.show_helical"));
        m_metricsGroup->setTitle(tr("trajectory.metrics"));
        m_metricCaptions[0]->setText(tr("trajectory.metric_count"));
        m_metricCaptions[1]->setText(tr("trajectory.metric_duration"));
        m_metricCaptions[2]->setText(tr("trajectory.metric_interval"));
        m_pointsGroup->setTitle(tr("trajectory.points"));
        m_interpolationLabel->setText(tr("trajectory.interpolation_interval"));
        m_interpolateButton->setText(tr("trajectory.interpolate"));
        m_transformGroup->setTitle(tr("trajectory.point_transform"));
        m_applyTransformButton->setText(tr("trajectory.apply_transform"));
        m_groupGroup->setTitle(tr("trajectory.group"));
        m_newButton->setText(tr("trajectory.new"));
        m_editButton->setText(tr("trajectory.edit"));
        m_removeButton->setText(tr("trajectory.remove"));
        m_saveToGroupButton->setText(tr("trajectory.save_to_group"));
        m_transitionLabel->setText(tr("trajectory.transition_after"));
    }
}
