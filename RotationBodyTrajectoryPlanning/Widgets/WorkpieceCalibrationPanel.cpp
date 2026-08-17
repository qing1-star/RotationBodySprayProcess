#include "WorkpieceCalibrationPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include "RobotQtWidgetUtils.h"

#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        constexpr int cylinderRowCount = 12;
        constexpr int circleRowCount = 6;

        QTableWidget* makePointTable(int rows, QWidget* parent)
        {
            auto* table = new QTableWidget(rows, 3, parent);
            table->setHorizontalHeaderLabels({ QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z") });
            table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            table->verticalHeader()->setDefaultSectionSize(25);
            table->verticalHeader()->setMinimumWidth(30);
            table->setAlternatingRowColors(true);
            table->setSelectionBehavior(QAbstractItemView::SelectRows);
            table->setSelectionMode(QAbstractItemView::ExtendedSelection);
            table->setMinimumHeight(rows > 6 ? 238 : 178);
            for(int row = 0; row < rows; ++row) {
                for(int column = 0; column < 3; ++column) {
                    auto* item = new QTableWidgetItem();
                    item->setTextAlignment(Qt::AlignCenter);
                    table->setItem(row, column, item);
                }
            }
            return table;
        }

        QLineEdit* makeCoordinateEdit(QWidget* parent)
        {
            auto* field = new QLineEdit(parent);
            auto* validator = new QDoubleValidator(-1000000.0, 1000000.0, 3, field);
            validator->setNotation(QDoubleValidator::StandardNotation);
            field->setValidator(validator);
            field->setAlignment(Qt::AlignRight);
            field->setPlaceholderText(QStringLiteral("0.000"));
            field->setMinimumWidth(0);
            field->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            return field;
        }

        QString tableCellText(const QTableWidget* table, int row, int column)
        {
            const QTableWidgetItem* item = table->item(row, column);
            return item ? item->text().trimmed() : QString();
        }
    }

    WorkpieceCalibrationPanel::WorkpieceCalibrationPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyWorkpieceCalibrationPanel"));
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSpacing(8);

        m_titleLabel = new QLabel(this);
        m_titleLabel->setObjectName(QStringLiteral("rotationBodyCalibration.title"));
        m_titleLabel->setProperty("panelTitle", true);
        layout->addWidget(m_titleLabel);
        m_introLabel = new QLabel(this);
        m_introLabel->setWordWrap(true);
        m_introLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_introLabel);
        layout->addWidget(m_introLabel);

        m_modeTabs = new QTabWidget(this);
        m_modeTabs->setObjectName(QStringLiteral("rotationBodyCalibration.modes"));
        m_cylinderTab = new QWidget(m_modeTabs);
        auto* cylinderLayout = new QVBoxLayout(m_cylinderTab);
        cylinderLayout->setContentsMargins(6, 6, 6, 6);
        cylinderLayout->setSpacing(5);
        m_cylinderHintLabel = new QLabel(m_cylinderTab);
        m_cylinderHintLabel->setWordWrap(true);
        cylinderLayout->addWidget(m_cylinderHintLabel);
        m_cylinderTable = makePointTable(cylinderRowCount, m_cylinderTab);
        m_cylinderTable->setObjectName(QStringLiteral("rotationBodyCalibration.cylinderPoints"));
        cylinderLayout->addWidget(m_cylinderTable);
        m_modeTabs->addTab(m_cylinderTab, QString());

        m_circleTab = new QWidget(m_modeTabs);
        auto* circleLayout = new QVBoxLayout(m_circleTab);
        circleLayout->setContentsMargins(6, 6, 6, 6);
        circleLayout->setSpacing(5);
        m_circleHintLabel = new QLabel(m_circleTab);
        m_circleHintLabel->setWordWrap(true);
        circleLayout->addWidget(m_circleHintLabel);
        m_circleTable = makePointTable(circleRowCount, m_circleTab);
        m_circleTable->setObjectName(QStringLiteral("rotationBodyCalibration.circlePoints"));
        circleLayout->addWidget(m_circleTable);
        m_modeTabs->addTab(m_circleTab, QString());
        layout->addWidget(m_modeTabs);

        m_operationsGroup = new QGroupBox(this);
        auto* operationsLayout = new QGridLayout(m_operationsGroup);
        operationsLayout->setContentsMargins(8, 20, 8, 8);
        operationsLayout->setSpacing(5);
        m_clearSelectedButton = new QPushButton(m_operationsGroup);
        m_clearSelectedButton->setObjectName(
            QStringLiteral("rotationBodyCalibration.clearSelected"));
        m_clearModeButton = new QPushButton(m_operationsGroup);
        m_clearModeButton->setObjectName(
            QStringLiteral("rotationBodyCalibration.clearMode"));
        m_fitCurrentButton = new QPushButton(m_operationsGroup);
        m_fitCurrentButton->setObjectName(
            QStringLiteral("rotationBodyCalibration.fitCurrent"));
        m_fitCurrentButton->setProperty("primaryAction", true);
        m_fitBothButton = new QPushButton(m_operationsGroup);
        m_fitBothButton->setObjectName(
            QStringLiteral("rotationBodyCalibration.fitBoth"));
        operationsLayout->addWidget(m_clearSelectedButton, 0, 0);
        operationsLayout->addWidget(m_clearModeButton, 0, 1);
        operationsLayout->addWidget(m_fitCurrentButton, 1, 0);
        operationsLayout->addWidget(m_fitBothButton, 1, 1);
        layout->addWidget(m_operationsGroup);

        m_fitResultGroup = new QGroupBox(this);
        auto* fitResultLayout = new QVBoxLayout(m_fitResultGroup);
        fitResultLayout->setContentsMargins(8, 20, 8, 8);
        fitResultLayout->setSpacing(5);
        m_cylinderResultLabel = new QLabel(m_fitResultGroup);
        m_circleResultLabel = new QLabel(m_fitResultGroup);
        m_showCylinderCheck = new QCheckBox(m_fitResultGroup);
        m_showCircleCheck = new QCheckBox(m_fitResultGroup);
        m_showCylinderCheck->setChecked(true);
        m_showCircleCheck->setChecked(true);
        fitResultLayout->addWidget(m_showCylinderCheck);
        fitResultLayout->addWidget(m_showCircleCheck);
        for(QLabel* label : { m_cylinderResultLabel, m_circleResultLabel }) {
            label->setWordWrap(true);
            robot_qt_viewer::makeHorizontallyCompressible(label);
            fitResultLayout->addWidget(label);
        }
        layout->addWidget(m_fitResultGroup);

        m_frameGroup = new QGroupBox(this);
        auto* frameLayout = new QGridLayout(m_frameGroup);
        frameLayout->setContentsMargins(8, 20, 8, 8);
        frameLayout->setHorizontalSpacing(5);
        frameLayout->setVerticalSpacing(6);
        m_axisSourceLabel = new QLabel(m_frameGroup);
        m_axisSourceCombo = new QComboBox(m_frameGroup);
        m_axisSourceCombo->setObjectName(QStringLiteral("rotationBodyCalibration.axisSource"));
        m_axisSourceCombo->addItem(QString(), static_cast<int>(domain::CalibrationMode::Cylinder3d));
        m_axisSourceCombo->addItem(QString(), static_cast<int>(domain::CalibrationMode::Circle2d));
        m_axisStatusLabel = new QLabel(m_frameGroup);
        m_axisStatusLabel->setObjectName(
            QStringLiteral("rotationBodyCalibration.axisStatus"));
        m_axisStatusLabel->setWordWrap(true);
        m_axisStatusLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_axisStatusLabel);
        frameLayout->addWidget(m_axisSourceLabel, 0, 0);
        frameLayout->addWidget(m_axisSourceCombo, 0, 1, 1, 3);
        frameLayout->addWidget(m_axisStatusLabel, 1, 0, 1, 4);

        m_referenceHeaderLabel = new QLabel(m_frameGroup);
        m_referenceHeaderLabel->setProperty("sectionTitle", true);
        frameLayout->addWidget(m_referenceHeaderLabel, 2, 0, 1, 4);
        for(int column = 0; column < 3; ++column) {
            m_referenceColumnLabels[static_cast<std::size_t>(column)] =
                new QLabel(QString(QLatin1Char('X' + column)), m_frameGroup);
            m_referenceColumnLabels[static_cast<std::size_t>(column)]->setAlignment(Qt::AlignCenter);
            frameLayout->addWidget(
                m_referenceColumnLabels[static_cast<std::size_t>(column)],
                3,
                column + 1);
        }
        const std::array<std::array<QLineEdit*, 3>*, 3> referenceRows{
            &m_topReferenceFields,
            &m_yStartFields,
            &m_yEndFields
        };
        for(int row = 0; row < 3; ++row) {
            m_referenceRowLabels[static_cast<std::size_t>(row)] = new QLabel(m_frameGroup);
            frameLayout->addWidget(m_referenceRowLabels[static_cast<std::size_t>(row)], row + 4, 0);
            for(int column = 0; column < 3; ++column) {
                QLineEdit* field = makeCoordinateEdit(m_frameGroup);
                const std::array<const char*, 3> rowNames{
                    "top",
                    "yStart",
                    "yEnd"
                };
                const std::array<const char*, 3> columnNames{
                    "X",
                    "Y",
                    "Z"
                };
                field->setObjectName(QStringLiteral("rotationBodyCalibration.%1.%2")
                    .arg(QString::fromLatin1(rowNames[static_cast<std::size_t>(row)]))
                    .arg(QString::fromLatin1(columnNames[static_cast<std::size_t>(column)])));
                (*referenceRows[static_cast<std::size_t>(row)])[static_cast<std::size_t>(column)] = field;
                frameLayout->addWidget(field, row + 4, column + 1);
                connect(field, &QLineEdit::textChanged, this, [this]() {
                    if(!m_updating) invalidateFrameResult();
                });
            }
        }

        m_heightLabel = new QLabel(m_frameGroup);
        m_heightSpin = new QDoubleSpinBox(m_frameGroup);
        m_heightSpin->setObjectName(QStringLiteral("rotationBodyCalibration.height"));
        m_heightSpin->setRange(0.001, 1000000.0);
        m_heightSpin->setDecimals(3);
        m_heightSpin->setSuffix(QStringLiteral(" mm"));
        m_heightSpin->setKeyboardTracking(false);
        m_heightSpin->setValue(120.0);
        frameLayout->addWidget(m_heightLabel, 7, 0, 1, 2);
        frameLayout->addWidget(m_heightSpin, 7, 2, 1, 2);
        m_calculateButton = new QPushButton(m_frameGroup);
        m_calculateButton->setObjectName(QStringLiteral("rotationBodyCalibration.calculateApply"));
        m_calculateButton->setProperty("primaryAction", true);
        m_calculateButton->setMinimumHeight(38);
        frameLayout->addWidget(m_calculateButton, 8, 0, 1, 4);
        m_poseResultLabel = new QLabel(m_frameGroup);
        m_poseResultLabel->setObjectName(
            QStringLiteral("rotationBodyCalibration.poseResult"));
        m_poseResultLabel->setWordWrap(true);
        m_poseResultLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_poseResultLabel);
        frameLayout->addWidget(m_poseResultLabel, 9, 0, 1, 4);
        frameLayout->setColumnStretch(1, 1);
        frameLayout->setColumnStretch(2, 1);
        frameLayout->setColumnStretch(3, 1);
        layout->addWidget(m_frameGroup);
        layout->addStretch(1);

        connect(m_modeTabs, &QTabWidget::currentChanged, this, [this]() {
            updateEnabledState();
            emitWorkspaceEdited();
        });
        connect(m_cylinderTable, &QTableWidget::itemChanged, this, [this]() {
            if(!m_updating) {
                invalidateFit(domain::CalibrationMode::Cylinder3d);
                emitWorkspaceEdited();
            }
        });
        connect(m_circleTable, &QTableWidget::itemChanged, this, [this]() {
            if(!m_updating) {
                invalidateFit(domain::CalibrationMode::Circle2d);
                emitWorkspaceEdited();
            }
        });
        connect(m_clearSelectedButton, &QPushButton::clicked,
            this, &WorkpieceCalibrationPanel::clearSelectedPoints);
        connect(m_clearModeButton, &QPushButton::clicked,
            this, &WorkpieceCalibrationPanel::clearActiveMode);
        connect(m_fitCurrentButton, &QPushButton::clicked,
            this, &WorkpieceCalibrationPanel::runActiveFit);
        connect(m_fitBothButton, &QPushButton::clicked,
            this, &WorkpieceCalibrationPanel::runBothFits);
        connect(m_axisSourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this]() {
                if(m_updating) return;
                invalidateFrameResult();
                updateAxisStatus();
                updateEnabledState();
                emitWorkspaceEdited();
            });
        connect(m_heightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this]() {
                if(!m_updating) {
                    m_heightInitializedFromModel = true;
                    invalidateFrameResult();
                    emitWorkspaceEdited();
                }
            });
        for(QLineEdit* field : {
            m_topReferenceFields[0], m_topReferenceFields[1], m_topReferenceFields[2],
            m_yStartFields[0], m_yStartFields[1], m_yStartFields[2],
            m_yEndFields[0], m_yEndFields[1], m_yEndFields[2] }) {
            connect(field, &QLineEdit::editingFinished,
                this, &WorkpieceCalibrationPanel::emitWorkspaceEdited);
        }
        connect(m_showCylinderCheck, &QCheckBox::toggled,
            this, [this]() { emitWorkspaceEdited(); });
        connect(m_showCircleCheck, &QCheckBox::toggled,
            this, [this]() { emitWorkspaceEdited(); });
        connect(m_calculateButton, &QPushButton::clicked,
            this, &WorkpieceCalibrationPanel::calculateAndApply);

        retranslate();
        updateFitResults();
        updateAxisStatus();
        updateEnabledState();
    }

    void WorkpieceCalibrationPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) return;
        m_languageCode = canonical;
        retranslate();
        updateFitResults();
        updateAxisStatus();
    }

    QString WorkpieceCalibrationPanel::languageCode() const
    {
        return m_languageCode;
    }

    void WorkpieceCalibrationPanel::setViewModel(
        const RotationBodyPlanningViewModel& viewModel)
    {
        m_viewModel = viewModel;
        applyWorkspace(viewModel.calibrationWorkspace);
        updateEnabledState();
    }

    WorkpieceCalibrationWorkspace
    WorkpieceCalibrationPanel::workspaceFromInputs() const
    {
        WorkpieceCalibrationWorkspace workspace;
        const auto readRows = [](const QTableWidget* table, auto& rows) {
            for(int row = 0; row < table->rowCount(); ++row) {
                for(int column = 0; column < 3; ++column) {
                    const QString text = tableCellText(table, row, column);
                    bool ok = false;
                    const double millimeters = text.toDouble(&ok);
                    rows[static_cast<std::size_t>(row)]
                        [static_cast<std::size_t>(column)] =
                        !text.isEmpty() && ok && std::isfinite(millimeters)
                        ? std::optional<double>(
                            domain::millimetersToMeters(millimeters))
                        : std::nullopt;
                }
            }
        };
        readRows(m_cylinderTable, workspace.cylinderRows);
        readRows(m_circleTable, workspace.circleRows);
        const auto readReference = [](const std::array<QLineEdit*, 3>& fields) {
            CalibrationCoordinateRow row;
            for(int index = 0; index < 3; ++index) {
                const QString text = fields[static_cast<std::size_t>(index)]
                    ->text().trimmed();
                bool ok = false;
                const double millimeters = text.toDouble(&ok);
                row[static_cast<std::size_t>(index)] =
                    !text.isEmpty() && ok && std::isfinite(millimeters)
                    ? std::optional<double>(
                        domain::millimetersToMeters(millimeters))
                    : std::nullopt;
            }
            return row;
        };
        workspace.topReference = readReference(m_topReferenceFields);
        workspace.yDirectionStart = readReference(m_yStartFields);
        workspace.yDirectionEnd = readReference(m_yEndFields);
        workspace.workpieceHeightMeters =
            domain::millimetersToMeters(m_heightSpin->value());
        workspace.activeMode = activeMode();
        workspace.axisSource = static_cast<domain::CalibrationMode>(
            m_axisSourceCombo->currentData().toInt());
        workspace.showCylinder = m_showCylinderCheck->isChecked();
        workspace.showCircle = m_showCircleCheck->isChecked();
        workspace.cylinderFit = m_cylinderFit;
        workspace.circleFit = m_circleFit;
        workspace.frame = m_frameResult;
        return workspace;
    }

    void WorkpieceCalibrationPanel::applyWorkspace(
        const WorkpieceCalibrationWorkspace& workspace)
    {
        m_updating = true;
        const auto writeRows = [](QTableWidget* table, const auto& rows) {
            for(int row = 0; row < table->rowCount(); ++row) {
                for(int column = 0; column < 3; ++column) {
                    QTableWidgetItem* item = table->item(row, column);
                    const std::optional<double>& value =
                        rows[static_cast<std::size_t>(row)]
                            [static_cast<std::size_t>(column)];
                    item->setText(value
                        ? QString::number(
                            domain::metersToMillimeters(*value), 'f', 3)
                        : QString());
                }
            }
        };
        writeRows(m_cylinderTable, workspace.cylinderRows);
        writeRows(m_circleTable, workspace.circleRows);
        const auto writeReference = [](const CalibrationCoordinateRow& row,
            const std::array<QLineEdit*, 3>& fields) {
            for(int index = 0; index < 3; ++index) {
                const std::optional<double>& value =
                    row[static_cast<std::size_t>(index)];
                fields[static_cast<std::size_t>(index)]->setText(value
                    ? QString::number(
                        domain::metersToMillimeters(*value), 'f', 3)
                    : QString());
            }
        };
        writeReference(workspace.topReference, m_topReferenceFields);
        writeReference(workspace.yDirectionStart, m_yStartFields);
        writeReference(workspace.yDirectionEnd, m_yEndFields);
        m_heightSpin->setValue(
            domain::metersToMillimeters(workspace.workpieceHeightMeters));
        m_modeTabs->setCurrentIndex(
            workspace.activeMode == domain::CalibrationMode::Cylinder3d ? 0 : 1);
        m_axisSourceCombo->setCurrentIndex(
            workspace.axisSource == domain::CalibrationMode::Cylinder3d ? 0 : 1);
        m_showCylinderCheck->setChecked(workspace.showCylinder);
        m_showCircleCheck->setChecked(workspace.showCircle);
        m_cylinderFit = workspace.cylinderFit;
        m_circleFit = workspace.circleFit;
        m_frameResult = workspace.frame;
        m_heightInitializedFromModel = true;
        m_updating = false;
        updateFitResults();
        updateAxisStatus();
        updatePoseResult();
    }

    void WorkpieceCalibrationPanel::emitWorkspaceEdited()
    {
        if(!m_updating) {
            emit workspaceEdited(workspaceFromInputs());
        }
    }

    const std::optional<domain::CalibrationAxisFit>&
    WorkpieceCalibrationPanel::cylinderFit() const noexcept
    {
        return m_cylinderFit;
    }

    const std::optional<domain::CalibrationAxisFit>&
    WorkpieceCalibrationPanel::circleFit() const noexcept
    {
        return m_circleFit;
    }

    const std::optional<domain::WorkpieceFrameCalibration>&
    WorkpieceCalibrationPanel::frameResult() const noexcept
    {
        return m_frameResult;
    }

    domain::CalibrationMode WorkpieceCalibrationPanel::activeMode() const noexcept
    {
        return m_modeTabs->currentIndex() == 0
            ? domain::CalibrationMode::Cylinder3d
            : domain::CalibrationMode::Circle2d;
    }

    QTableWidget* WorkpieceCalibrationPanel::tableForMode(
        domain::CalibrationMode mode) const noexcept
    {
        return mode == domain::CalibrationMode::Cylinder3d
            ? m_cylinderTable
            : m_circleTable;
    }

    std::optional<domain::CalibrationPointList>
    WorkpieceCalibrationPanel::readPoints(
        QTableWidget* table,
        QString& errorText) const
    {
        domain::CalibrationPointList points;
        for(int row = 0; row < table->rowCount(); ++row) {
            std::array<QString, 3> values;
            int populated = 0;
            for(int column = 0; column < 3; ++column) {
                values[static_cast<std::size_t>(column)] =
                    tableCellText(table, row, column);
                if(!values[static_cast<std::size_t>(column)].isEmpty()) ++populated;
            }
            if(populated == 0) continue;
            if(populated != 3) {
                errorText = RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "calibration.error.incomplete_row").arg(row + 1);
                return std::nullopt;
            }
            Eigen::Vector3d point;
            for(int column = 0; column < 3; ++column) {
                bool ok = false;
                const double millimeters =
                    values[static_cast<std::size_t>(column)].toDouble(&ok);
                if(!ok || !std::isfinite(millimeters)) {
                    errorText = RotationBodyPlanningTranslations::text(
                        m_languageCode,
                        "calibration.error.invalid_number").arg(row + 1);
                    return std::nullopt;
                }
                point[column] = domain::millimetersToMeters(millimeters);
            }
            points.push_back(point);
        }
        errorText.clear();
        return points;
    }

    std::optional<Eigen::Vector3d> WorkpieceCalibrationPanel::readReferencePoint(
        const std::array<QLineEdit*, 3>& fields,
        const char* missingKey,
        QString& errorText) const
    {
        Eigen::Vector3d point;
        for(int index = 0; index < 3; ++index) {
            const QString value = fields[static_cast<std::size_t>(index)]->text().trimmed();
            bool ok = false;
            const double millimeters = value.toDouble(&ok);
            if(value.isEmpty() || !ok || !std::isfinite(millimeters)) {
                errorText = RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    missingKey);
                return std::nullopt;
            }
            point[index] = domain::millimetersToMeters(millimeters);
        }
        return point;
    }

    std::optional<domain::CalibrationAxisFit>* WorkpieceCalibrationPanel::fitForMode(
        domain::CalibrationMode mode) noexcept
    {
        return mode == domain::CalibrationMode::Cylinder3d
            ? &m_cylinderFit
            : &m_circleFit;
    }

    const std::optional<domain::CalibrationAxisFit>*
    WorkpieceCalibrationPanel::fitForMode(domain::CalibrationMode mode) const noexcept
    {
        return mode == domain::CalibrationMode::Cylinder3d
            ? &m_cylinderFit
            : &m_circleFit;
    }

    const domain::CalibrationAxisFit* WorkpieceCalibrationPanel::selectedAxisFit() const noexcept
    {
        const domain::CalibrationMode mode = static_cast<domain::CalibrationMode>(
            m_axisSourceCombo->currentData().toInt());
        const auto* fit = fitForMode(mode);
        return fit->has_value() ? &fit->value() : nullptr;
    }

    void WorkpieceCalibrationPanel::runFit(domain::CalibrationMode mode)
    {
        QString error;
        const std::optional<domain::CalibrationPointList> points =
            readPoints(tableForMode(mode), error);
        if(!points) {
            *fitForMode(mode) = std::nullopt;
            if(mode == domain::CalibrationMode::Cylinder3d) {
                m_cylinderResultLabel->setText(error);
            } else {
                m_circleResultLabel->setText(error);
            }
            invalidateFrameResult();
            updateAxisStatus();
            updateEnabledState();
            emitWorkspaceEdited();
            return;
        }

        const int count = static_cast<int>(points->size());
        const bool countValid = mode == domain::CalibrationMode::Cylinder3d
            ? count >= 8 && count <= 12
            : count == 6;
        if(!countValid) {
            const char* key = mode == domain::CalibrationMode::Cylinder3d
                ? "calibration.error.cylinder_count"
                : "calibration.error.circle_count";
            *fitForMode(mode) = std::nullopt;
            QLabel* label = mode == domain::CalibrationMode::Cylinder3d
                ? m_cylinderResultLabel
                : m_circleResultLabel;
            label->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                key).arg(count));
            invalidateFrameResult();
            updateAxisStatus();
            updateEnabledState();
            emitWorkspaceEdited();
            return;
        }

        const domain::PlanningResult<domain::CalibrationAxisFit> result =
            mode == domain::CalibrationMode::Cylinder3d
                ? domain::WorkpieceCalibrationSolver::fitCylinder3d(*points)
                : domain::WorkpieceCalibrationSolver::fitCircle2d(*points);
        if(result) {
            *fitForMode(mode) = result.value;
            const QSignalBlocker blocker(m_axisSourceCombo);
            m_axisSourceCombo->setCurrentIndex(
                mode == domain::CalibrationMode::Cylinder3d ? 0 : 1);
        } else {
            *fitForMode(mode) = std::nullopt;
            QLabel* label = mode == domain::CalibrationMode::Cylinder3d
                ? m_cylinderResultLabel
                : m_circleResultLabel;
            const char* key = mode == domain::CalibrationMode::Circle2d &&
                result.error.code == domain::PlanningErrorCode::InvalidArgument
                ? "calibration.error.circle_plane"
                : "calibration.error.fit_failed";
            label->setText(RotationBodyPlanningTranslations::text(m_languageCode, key));
        }
        invalidateFrameResult();
        updateFitResults();
        updateAxisStatus();
        updateEnabledState();
        emitWorkspaceEdited();
    }

    void WorkpieceCalibrationPanel::runActiveFit()
    {
        runFit(activeMode());
    }

    void WorkpieceCalibrationPanel::runBothFits()
    {
        const int selectedSource = m_axisSourceCombo->currentIndex();
        runFit(domain::CalibrationMode::Cylinder3d);
        runFit(domain::CalibrationMode::Circle2d);
        if((selectedSource == 0 && m_cylinderFit) ||
            (selectedSource == 1 && m_circleFit)) {
            const QSignalBlocker blocker(m_axisSourceCombo);
            m_axisSourceCombo->setCurrentIndex(selectedSource);
        }
        updateAxisStatus();
        emitWorkspaceEdited();
    }

    void WorkpieceCalibrationPanel::clearSelectedPoints()
    {
        QTableWidget* table = tableForMode(activeMode());
        const QModelIndexList selectedRows = table->selectionModel()->selectedRows();
        m_updating = true;
        for(const QModelIndex& rowIndex : selectedRows) {
            for(int column = 0; column < table->columnCount(); ++column) {
                if(QTableWidgetItem* item = table->item(rowIndex.row(), column)) {
                    item->setText(QString());
                }
            }
        }
        m_updating = false;
        invalidateFit(activeMode());
        emitWorkspaceEdited();
    }

    void WorkpieceCalibrationPanel::clearActiveMode()
    {
        QTableWidget* table = tableForMode(activeMode());
        m_updating = true;
        for(int row = 0; row < table->rowCount(); ++row) {
            for(int column = 0; column < table->columnCount(); ++column) {
                if(QTableWidgetItem* item = table->item(row, column)) {
                    item->setText(QString());
                }
            }
        }
        m_updating = false;
        invalidateFit(activeMode());
        emitWorkspaceEdited();
    }

    void WorkpieceCalibrationPanel::invalidateFit(domain::CalibrationMode mode)
    {
        *fitForMode(mode) = std::nullopt;
        invalidateFrameResult();
        updateFitResults();
        updateAxisStatus();
        updateEnabledState();
    }

    void WorkpieceCalibrationPanel::invalidateFrameResult()
    {
        m_frameResult.reset();
        if(m_poseResultLabel) {
            m_poseResultLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.pose_not_calculated"));
        }
    }

    void WorkpieceCalibrationPanel::calculateAndApply()
    {
        const domain::CalibrationAxisFit* fit = selectedAxisFit();
        if(fit == nullptr) {
            m_poseResultLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.error.axis_missing"));
            return;
        }
        QString error;
        const auto top = readReferencePoint(
            m_topReferenceFields,
            "calibration.error.top_missing",
            error);
        if(!top) {
            m_poseResultLabel->setText(error);
            return;
        }
        const auto yStart = readReferencePoint(
            m_yStartFields,
            "calibration.error.y_start_missing",
            error);
        if(!yStart) {
            m_poseResultLabel->setText(error);
            return;
        }
        const auto yEnd = readReferencePoint(
            m_yEndFields,
            "calibration.error.y_end_missing",
            error);
        if(!yEnd) {
            m_poseResultLabel->setText(error);
            return;
        }
        const double heightMeters =
            domain::millimetersToMeters(m_heightSpin->value());
        const domain::PlanningResult<domain::WorkpieceFrameCalibration> result =
            domain::WorkpieceCalibrationSolver::computeWorkpieceFrame(
                *fit,
                *top,
                heightMeters,
                *yStart,
                *yEnd);
        if(!result) {
            const Eigen::Vector3d measuredY = *yEnd - *yStart;
            const Eigen::Vector3d axis = fit->axisDirectionBase.normalized();
            const bool shortBaseline = measuredY.norm() < 0.001;
            const bool parallel = !shortBaseline &&
                (measuredY - measuredY.dot(axis) * axis).norm() < 0.001;
            const char* key = shortBaseline
                ? "calibration.error.y_short"
                : (parallel
                    ? "calibration.error.y_parallel"
                    : "calibration.error.frame_failed");
            m_poseResultLabel->setText(
                RotationBodyPlanningTranslations::text(m_languageCode, key));
            return;
        }

        m_frameResult = result.value;
        updatePoseResult();
        emitWorkspaceEdited();
        emit baseTransformCalculated(result.value.baseFromPlanningComponents);
    }

    void WorkpieceCalibrationPanel::updatePoseResult()
    {
        if(!m_frameResult) {
            m_poseResultLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.pose_not_calculated"));
            return;
        }

        const domain::TransformComponents& components =
            m_frameResult->baseFromPlanningComponents;
        const Eigen::Vector3d degrees(
            domain::radiansToDegrees(components.rollPitchYawRadians.x()),
            domain::radiansToDegrees(components.rollPitchYawRadians.y()),
            domain::radiansToDegrees(components.rollPitchYawRadians.z()));
        const Eigen::Vector4d& quaternion = m_frameResult->abbQuaternionWxyz;
        QString text = RotationBodyPlanningTranslations::text(
            m_languageCode,
            "calibration.pose_result")
            .arg(vectorMillimeters(components.translationMeters, 3))
            .arg(degrees.x(), 0, 'f', 4)
            .arg(degrees.y(), 0, 'f', 4)
            .arg(degrees.z(), 0, 'f', 4)
            .arg(quaternion.x(), 0, 'f', 8)
            .arg(quaternion.y(), 0, 'f', 8)
            .arg(quaternion.z(), 0, 'f', 8)
            .arg(quaternion.w(), 0, 'f', 8);
        if(m_frameResult->shortYDirectionBaseline) {
            text += QLatin1Char('\n') + RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.warning.short_y");
        }
        m_poseResultLabel->setText(text);
    }

    void WorkpieceCalibrationPanel::updateFitResults()
    {
        const auto update = [this](
            QLabel* label,
            const std::optional<domain::CalibrationAxisFit>& fit,
            const char* nameKey) {
            const QString name = RotationBodyPlanningTranslations::text(
                m_languageCode,
                nameKey);
            if(!fit) {
                label->setText(RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "calibration.fit_not_ready").arg(name));
                return;
            }
            QString result = RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.fit_result")
                .arg(name)
                .arg(domain::metersToMillimeters(fit->radiusMeters), 0, 'f', 3)
                .arg(domain::metersToMillimeters(fit->rmsResidualMeters), 0, 'f', 4)
                .arg(domain::metersToMillimeters(fit->maximumResidualMeters), 0, 'f', 4);
            if(fit->lowAxialSpan) {
                result += QLatin1Char('\n') + RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "calibration.warning.axial_span");
            }
            label->setText(result);
        };
        update(m_cylinderResultLabel, m_cylinderFit, "calibration.mode_cylinder");
        update(m_circleResultLabel, m_circleFit, "calibration.mode_circle");
    }

    void WorkpieceCalibrationPanel::updateAxisStatus()
    {
        const domain::CalibrationAxisFit* fit = selectedAxisFit();
        if(fit == nullptr) {
            m_axisStatusLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "calibration.axis_not_ready"));
            return;
        }
        m_axisStatusLabel->setText(RotationBodyPlanningTranslations::text(
            m_languageCode,
            "calibration.axis_result")
            .arg(vectorMillimeters(fit->axisPointBaseMeters, 3))
            .arg(vectorUnitless(fit->axisDirectionBase, 6)));
    }

    void WorkpieceCalibrationPanel::updateEnabledState()
    {
        const bool editable = !m_viewModel.isBusy;
        m_modeTabs->setEnabled(editable);
        m_clearSelectedButton->setEnabled(editable);
        m_clearModeButton->setEnabled(editable);
        m_fitCurrentButton->setEnabled(editable);
        m_fitBothButton->setEnabled(editable);
        m_axisSourceCombo->setEnabled(editable);
        m_calculateButton->setEnabled(
            editable && m_viewModel.hasModel && selectedAxisFit() != nullptr);
        m_calculateButton->setToolTip(
            m_viewModel.hasModel
                ? RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "calibration.calculate_tooltip")
                : RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "calibration.load_model_first"));
    }

    void WorkpieceCalibrationPanel::retranslate()
    {
        const auto translated = [this](const char* key) {
            return RotationBodyPlanningTranslations::text(m_languageCode, key);
        };
        m_titleLabel->setText(translated("calibration.title"));
        m_introLabel->setText(translated("calibration.intro"));
        m_modeTabs->setTabText(0, translated("calibration.mode_one"));
        m_modeTabs->setTabText(1, translated("calibration.mode_two"));
        m_cylinderHintLabel->setText(translated("calibration.cylinder_hint"));
        m_circleHintLabel->setText(translated("calibration.circle_hint"));
        m_operationsGroup->setTitle(translated("calibration.operations"));
        m_clearSelectedButton->setText(translated("calibration.clear_selected"));
        m_clearModeButton->setText(translated("calibration.clear_mode"));
        m_fitCurrentButton->setText(translated("calibration.fit_current"));
        m_fitBothButton->setText(translated("calibration.fit_both"));
        m_fitResultGroup->setTitle(translated("calibration.fit_output"));
        m_showCylinderCheck->setText(translated("calibration.show_cylinder"));
        m_showCircleCheck->setText(translated("calibration.show_circle"));
        m_frameGroup->setTitle(translated("calibration.frame_title"));
        m_axisSourceLabel->setText(translated("calibration.axis_source"));
        m_axisSourceCombo->setItemText(0, translated("calibration.axis_cylinder"));
        m_axisSourceCombo->setItemText(1, translated("calibration.axis_circle"));
        m_referenceHeaderLabel->setText(translated("calibration.reference_points"));
        m_referenceRowLabels[0]->setText(translated("calibration.top_point"));
        m_referenceRowLabels[1]->setText(translated("calibration.y_start"));
        m_referenceRowLabels[2]->setText(translated("calibration.y_end"));
        m_heightLabel->setText(translated("calibration.height"));
        m_calculateButton->setText(translated("calibration.calculate_apply"));
        updatePoseResult();
        updateEnabledState();
    }

    QString WorkpieceCalibrationPanel::vectorMillimeters(
        const Eigen::Vector3d& meters,
        int precision) const
    {
        return QStringLiteral("(%1, %2, %3)")
            .arg(domain::metersToMillimeters(meters.x()), 0, 'f', precision)
            .arg(domain::metersToMillimeters(meters.y()), 0, 'f', precision)
            .arg(domain::metersToMillimeters(meters.z()), 0, 'f', precision);
    }

    QString WorkpieceCalibrationPanel::vectorUnitless(
        const Eigen::Vector3d& value,
        int precision) const
    {
        return QStringLiteral("(%1, %2, %3)")
            .arg(value.x(), 0, 'f', precision)
            .arg(value.y(), 0, 'f', precision)
            .arg(value.z(), 0, 'f', precision);
    }
}
