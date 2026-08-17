#include "ABBTranslationPanel.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include "RobotQtWidgetUtils.h"

#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStyle>
#include <QStringList>
#include <QVBoxLayout>

#include <algorithm>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
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
    }

    ABBTranslationPanel::ABBTranslationPanel(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyABBTranslationPanel"));
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        m_settingsGroup = new QGroupBox(this);
        auto* settingsForm = new QFormLayout(m_settingsGroup);
        settingsForm->setContentsMargins(8, 20, 8, 8);
        settingsForm->setSpacing(6);
        robot_qt_viewer::configureInspectorForm(settingsForm);
        const std::array<const char*, 3> axes{ { "X", "Y", "Z" } };
        for(std::size_t index = 0; index < axes.size(); ++index) {
            m_safetyPositionLabels[index] = new QLabel(m_settingsGroup);
            m_safetyPositionSpins[index] = makeSpin(
                QStringLiteral("rotationBodyABB.safety%1")
                    .arg(QString::fromLatin1(axes[index])),
                -1000000.0,
                1000000.0,
                3,
                QStringLiteral(" mm"),
                m_settingsGroup);
            settingsForm->addRow(
                m_safetyPositionLabels[index],
                m_safetyPositionSpins[index]);
        }
        m_safetySpeedLabel = new QLabel(m_settingsGroup);
        m_safetySpeedSpin = makeSpin(
            QStringLiteral("rotationBodyABB.safetySpeed"),
            0.001, 1000000.0, 3, QStringLiteral(" mm/s"), m_settingsGroup);
        m_safetySpeedSpin->setValue(200.0);
        settingsForm->addRow(m_safetySpeedLabel, m_safetySpeedSpin);
        m_moduleLabel = new QLabel(m_settingsGroup);
        m_moduleEdit = new QLineEdit(QStringLiteral("SprayRotation"), m_settingsGroup);
        m_moduleEdit->setObjectName(QStringLiteral("rotationBodyABB.moduleName"));
        settingsForm->addRow(m_moduleLabel, m_moduleEdit);
        m_fileLabel = new QLabel(m_settingsGroup);
        m_fileEdit = new QLineEdit(QStringLiteral("Spraybichi.mod"), m_settingsGroup);
        m_fileEdit->setObjectName(QStringLiteral("rotationBodyABB.fileName"));
        settingsForm->addRow(m_fileLabel, m_fileEdit);
        m_toolLabel = new QLabel(m_settingsGroup);
        m_toolEdit = new QLineEdit(QStringLiteral("penqiang"), m_settingsGroup);
        m_toolEdit->setObjectName(QStringLiteral("rotationBodyABB.toolName"));
        settingsForm->addRow(m_toolLabel, m_toolEdit);
        m_outputLabel = new QLabel(m_settingsGroup);
        auto* outputRow = new QHBoxLayout();
        outputRow->setContentsMargins(0, 0, 0, 0);
        outputRow->setSpacing(5);
        m_outputEdit = new QLineEdit(m_settingsGroup);
        m_outputEdit->setObjectName(QStringLiteral("rotationBodyABB.outputDirectory"));
        outputRow->addWidget(m_outputEdit, 1);
        m_browseButton = new QPushButton(m_settingsGroup);
        m_browseButton->setObjectName(QStringLiteral("rotationBodyABB.browse"));
        m_browseButton->setIcon(style()->standardIcon(QStyle::SP_DirOpenIcon));
        m_browseButton->setMinimumWidth(92);
        robot_qt_viewer::configureInspectorButton(m_browseButton);
        outputRow->addWidget(m_browseButton);
        settingsForm->addRow(m_outputLabel, outputRow);
        layout->addWidget(m_settingsGroup);

        m_sequenceGroup = new QGroupBox(this);
        auto* sequenceLayout = new QVBoxLayout(m_sequenceGroup);
        sequenceLayout->setContentsMargins(8, 20, 8, 8);
        sequenceLayout->setSpacing(6);
        m_sequenceList = new QListWidget(m_sequenceGroup);
        m_sequenceList->setObjectName(QStringLiteral("rotationBodyABB.sequenceList"));
        m_sequenceList->setSelectionMode(QAbstractItemView::SingleSelection);
        m_sequenceList->setMinimumHeight(180);
        robot_qt_viewer::configureInspectorList(m_sequenceList, true, true);
        sequenceLayout->addWidget(m_sequenceList);
        m_trajectoryCombo = new QComboBox(m_sequenceGroup);
        m_trajectoryCombo->setObjectName(QStringLiteral("rotationBodyABB.trajectoryChoice"));
        robot_qt_viewer::configureInspectorCombo(m_trajectoryCombo, 10);
        sequenceLayout->addWidget(m_trajectoryCombo);
        auto* addCommandRow = new QHBoxLayout();
        addCommandRow->setContentsMargins(0, 0, 0, 0);
        addCommandRow->setSpacing(5);
        m_addSafetyButton = new QPushButton(m_sequenceGroup);
        m_addSafetyButton->setObjectName(QStringLiteral("rotationBodyABB.addSafety"));
        robot_qt_viewer::configureInspectorButton(m_addSafetyButton);
        addCommandRow->addWidget(m_addSafetyButton, 1);
        m_addTrajectoryButton = new QPushButton(m_sequenceGroup);
        m_addTrajectoryButton->setObjectName(QStringLiteral("rotationBodyABB.addTrajectory"));
        robot_qt_viewer::configureInspectorButton(m_addTrajectoryButton);
        addCommandRow->addWidget(m_addTrajectoryButton, 1);
        sequenceLayout->addLayout(addCommandRow);
        auto* editRow = new QHBoxLayout();
        editRow->setContentsMargins(0, 0, 0, 0);
        editRow->setSpacing(5);
        m_moveUpButton = new QPushButton(m_sequenceGroup);
        m_moveDownButton = new QPushButton(m_sequenceGroup);
        m_removeButton = new QPushButton(m_sequenceGroup);
        m_moveUpButton->setObjectName(QStringLiteral("rotationBodyABB.moveUp"));
        m_moveDownButton->setObjectName(QStringLiteral("rotationBodyABB.moveDown"));
        m_removeButton->setObjectName(QStringLiteral("rotationBodyABB.remove"));
        m_moveUpButton->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
        m_moveDownButton->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
        m_removeButton->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
        for(QPushButton* button : { m_moveUpButton, m_moveDownButton, m_removeButton }) {
            robot_qt_viewer::configureInspectorButton(button);
            editRow->addWidget(button);
        }
        sequenceLayout->addLayout(editRow);
        m_generateButton = new QPushButton(m_sequenceGroup);
        m_generateButton->setObjectName(QStringLiteral("rotationBodyABB.generate"));
        m_generateButton->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
        robot_qt_viewer::configureInspectorButton(m_generateButton);
        sequenceLayout->addWidget(m_generateButton);
        layout->addWidget(m_sequenceGroup);

        m_previewGroup = new QGroupBox(this);
        auto* previewLayout = new QVBoxLayout(m_previewGroup);
        previewLayout->setContentsMargins(8, 20, 8, 8);
        previewLayout->setSpacing(6);
        m_previewList = new QListWidget(m_previewGroup);
        m_previewList->setObjectName(QStringLiteral("rotationBodyABB.generatedPreview"));
        m_previewList->setSelectionMode(QAbstractItemView::SingleSelection);
        m_previewList->setMinimumHeight(180);
        robot_qt_viewer::configureInspectorList(m_previewList, true, true);
        previewLayout->addWidget(m_previewList);
        auto* previewNavigation = new QHBoxLayout();
        previewNavigation->setContentsMargins(0, 0, 0, 0);
        previewNavigation->setSpacing(5);
        m_previousButton = new QPushButton(m_previewGroup);
        m_resetButton = new QPushButton(m_previewGroup);
        m_nextButton = new QPushButton(m_previewGroup);
        m_previousButton->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
        m_resetButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
        m_nextButton->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
        for(QPushButton* button : {
            m_previousButton, m_resetButton, m_nextButton }) {
            robot_qt_viewer::configureInspectorButton(button);
            previewNavigation->addWidget(button, 1);
        }
        previewLayout->addLayout(previewNavigation);
        m_exportStatusLabel = new QLabel(m_previewGroup);
        m_exportStatusLabel->setObjectName(QStringLiteral("rotationBodyABB.exportStatus"));
        m_exportStatusLabel->setWordWrap(true);
        m_exportStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        m_exportStatusLabel->setProperty("planningStatus", true);
        robot_qt_viewer::makeHorizontallyCompressible(m_exportStatusLabel);
        previewLayout->addWidget(m_exportStatusLabel);
        layout->addWidget(m_previewGroup);
        layout->addStretch(1);

        connect(m_browseButton, &QPushButton::clicked, this, [this]() {
            const QString directory = QFileDialog::getExistingDirectory(
                this,
                RotationBodyPlanningTranslations::text(
                    m_languageCode,
                    "abb.output_dialog"),
                m_outputEdit->text());
            if(directory.isEmpty()) return;
            m_outputEdit->setText(directory);
            emit settingsEdited(inputSettings());
        });
        const auto emitSettings = [this]() {
            if(!m_updating) emit settingsEdited(inputSettings());
        };
        for(QDoubleSpinBox* spin : m_safetyPositionSpins) {
            connect(spin, &QDoubleSpinBox::editingFinished, this, emitSettings);
        }
        connect(m_safetySpeedSpin, &QDoubleSpinBox::editingFinished, this, emitSettings);
        for(QLineEdit* edit : { m_moduleEdit, m_fileEdit, m_toolEdit, m_outputEdit }) {
            connect(edit, &QLineEdit::editingFinished, this, emitSettings);
        }
        connect(m_addSafetyButton, &QPushButton::clicked, this, [this]() {
            auto* item = new QListWidgetItem(m_sequenceList);
            item->setData(Qt::UserRole, 0);
            m_sequenceList->setCurrentItem(item);
            retranslate();
            emitSequence();
        });
        connect(m_addTrajectoryButton, &QPushButton::clicked, this, [this]() {
            if(m_trajectoryCombo->currentIndex() < 0) return;
            auto* item = new QListWidgetItem(m_sequenceList);
            item->setData(Qt::UserRole, 1);
            item->setData(Qt::UserRole + 1, m_trajectoryCombo->currentData());
            m_sequenceList->setCurrentItem(item);
            retranslate();
            emitSequence();
        });
        connect(m_moveUpButton, &QPushButton::clicked, this, [this]() {
            const int row = m_sequenceList->currentRow();
            if(row <= 0) return;
            QListWidgetItem* item = m_sequenceList->takeItem(row);
            m_sequenceList->insertItem(row - 1, item);
            m_sequenceList->setCurrentRow(row - 1);
            emitSequence();
        });
        connect(m_moveDownButton, &QPushButton::clicked, this, [this]() {
            const int row = m_sequenceList->currentRow();
            if(row < 0 || row + 1 >= m_sequenceList->count()) return;
            QListWidgetItem* item = m_sequenceList->takeItem(row);
            m_sequenceList->insertItem(row + 1, item);
            m_sequenceList->setCurrentRow(row + 1);
            emitSequence();
        });
        connect(m_removeButton, &QPushButton::clicked, this, [this]() {
            delete m_sequenceList->takeItem(m_sequenceList->currentRow());
            emitSequence();
        });
        connect(m_sequenceList, &QListWidget::itemSelectionChanged,
            this, &ABBTranslationPanel::updateEnabledState);
        connect(m_generateButton, &QPushButton::clicked, this, [this]() {
            emit generateRequested(inputSettings(), sequenceFromList());
        });
        connect(m_previewList, &QListWidget::currentRowChanged,
            this, [this](int row) {
                if(!m_updating) {
                    const QListWidgetItem* item = row >= 0
                        ? m_previewList->item(row)
                        : nullptr;
                    emit previewStepSelected(item != nullptr
                        ? item->data(Qt::UserRole).toInt()
                        : -1);
                }
                updateEnabledState();
            });
        connect(m_previousButton, &QPushButton::clicked,
            this, [this]() { selectPreviewRow(m_previewList->currentRow() - 1); });
        connect(m_resetButton, &QPushButton::clicked,
            this, [this]() { selectPreviewRow(0); });
        connect(m_nextButton, &QPushButton::clicked,
            this, [this]() { selectPreviewRow(m_previewList->currentRow() + 1); });

        retranslate();
        updateEnabledState();
    }

    void ABBTranslationPanel::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) return;
        m_languageCode = canonical;
        retranslate();
        rebuildTrajectoryChoices();
        rebuildGeneratedPreview();
    }

    QString ABBTranslationPanel::languageCode() const
    {
        return m_languageCode;
    }

    void ABBTranslationPanel::setViewModel(const RotationBodyPlanningViewModel& viewModel)
    {
        m_updating = true;
        m_viewModel = viewModel;
        const domain::RapidExportSettings& settings = viewModel.rapidSettings;
        for(int axis = 0; axis < 3; ++axis) {
            m_safetyPositionSpins[static_cast<std::size_t>(axis)]->setValue(
                domain::metersToMillimeters(settings.safetyPositionBaseMeters[axis]));
        }
        m_safetySpeedSpin->setValue(
            domain::metersToMillimeters(settings.safetySpeedMetersPerSecond));
        m_moduleEdit->setText(QString::fromStdString(settings.moduleName));
        QString fileName = QString::fromStdString(settings.fileName);
        if(!fileName.endsWith(QStringLiteral(".mod"), Qt::CaseInsensitive)) {
            fileName += QStringLiteral(".mod");
        }
        m_fileEdit->setText(fileName);
        m_toolEdit->setText(QString::fromStdString(settings.toolDataName));
        m_outputEdit->setText(QString::fromUtf8(settings.outputDirectory.c_str()));
        rebuildTrajectoryChoices();
        rebuildSequence();
        rebuildGeneratedPreview();
        m_updating = false;
        updateEnabledState();
    }

    domain::RapidExportSettings ABBTranslationPanel::inputSettings() const
    {
        domain::RapidExportSettings result = m_viewModel.rapidSettings;
        for(int axis = 0; axis < 3; ++axis) {
            result.safetyPositionBaseMeters[axis] = domain::millimetersToMeters(
                m_safetyPositionSpins[static_cast<std::size_t>(axis)]->value());
        }
        result.safetySpeedMetersPerSecond =
            domain::millimetersToMeters(m_safetySpeedSpin->value());
        result.moduleName = m_moduleEdit->text().trimmed().toStdString();
        result.fileName = m_fileEdit->text().trimmed().toStdString();
        result.toolDataName = m_toolEdit->text().trimmed().toStdString();
        result.outputDirectory = m_outputEdit->text().toUtf8().toStdString();
        return result;
    }

    std::vector<domain::RapidSequenceEntry> ABBTranslationPanel::sequenceFromList() const
    {
        std::vector<domain::RapidSequenceEntry> result;
        result.reserve(static_cast<std::size_t>(m_sequenceList->count()));
        for(int row = 0; row < m_sequenceList->count(); ++row) {
            const QListWidgetItem* item = m_sequenceList->item(row);
            domain::RapidSequenceEntry entry;
            entry.kind = item->data(Qt::UserRole).toInt() == 1
                ? domain::RapidSequenceEntryKind::Trajectory
                : domain::RapidSequenceEntryKind::SafetyPoint;
            entry.trajectoryPassId = item->data(Qt::UserRole + 1).toString().toStdString();
            result.push_back(std::move(entry));
        }
        return result;
    }

    void ABBTranslationPanel::rebuildTrajectoryChoices()
    {
        const QString selected = m_trajectoryCombo->currentData().toString();
        QSignalBlocker blocker(m_trajectoryCombo);
        m_trajectoryCombo->clear();
        for(const domain::TrajectoryPass& pass : m_viewModel.trajectoryWorkspace.group.passes) {
            m_trajectoryCombo->addItem(
                RotationBodyPlanningTranslations::text(m_languageCode, "abb.trajectory_item")
                    .arg(pass.order),
                QString::fromStdString(pass.id));
        }
        const int selectedIndex = m_trajectoryCombo->findData(selected);
        if(selectedIndex >= 0) m_trajectoryCombo->setCurrentIndex(selectedIndex);
    }

    void ABBTranslationPanel::rebuildSequence()
    {
        QSignalBlocker blocker(m_sequenceList);
        m_sequenceList->clear();
        for(const domain::RapidSequenceEntry& entry : m_viewModel.rapidSequence) {
            auto* item = new QListWidgetItem(m_sequenceList);
            item->setData(
                Qt::UserRole,
                entry.kind == domain::RapidSequenceEntryKind::Trajectory ? 1 : 0);
            item->setData(Qt::UserRole + 1, QString::fromStdString(entry.trajectoryPassId));
        }
        retranslate();
    }

    void ABBTranslationPanel::rebuildGeneratedPreview()
    {
        const int previousRow = m_previewList->currentRow();
        QSignalBlocker blocker(m_previewList);
        m_previewList->clear();
        if(m_viewModel.rapidModulePreview) {
            const domain::RapidPreviewSteps& steps =
                m_viewModel.rapidModulePreview->previewSteps;
            int logicalIndex = 0;
            for(std::size_t index = 0; index < steps.size();) {
                const domain::RapidPreviewStep& step = steps[index];
                ++logicalIndex;
                const QString source = step.sourceKind ==
                    domain::RapidSequenceEntryKind::SafetyPoint
                    ? RotationBodyPlanningTranslations::text(
                        m_languageCode,
                        "abb.preview_safety")
                    : QString::fromUtf8(step.trajectoryPassId.c_str());
                QStringList instructions;
                std::size_t lastStepIndex = index;
                if(step.sourceKind == domain::RapidSequenceEntryKind::SafetyPoint) {
                    instructions.push_back(QStringLiteral("%1 %2")
                        .arg(QString::fromLatin1(step.instruction.c_str()))
                        .arg(QString::fromLatin1(step.targetName.c_str())));
                } else {
                    while(lastStepIndex < steps.size() &&
                        steps[lastStepIndex].sourceKind ==
                            domain::RapidSequenceEntryKind::Trajectory &&
                        steps[lastStepIndex].sequenceIndex == step.sequenceIndex) {
                        instructions.push_back(QStringLiteral("%1 %2")
                            .arg(QString::fromLatin1(
                                steps[lastStepIndex].instruction.c_str()))
                            .arg(QString::fromLatin1(
                                steps[lastStepIndex].targetName.c_str())));
                        ++lastStepIndex;
                    }
                    --lastStepIndex;
                }
                auto* item = new QListWidgetItem(
                    QStringLiteral("%1 | %2 | %3")
                        .arg(logicalIndex, 3, 10, QLatin1Char('0'))
                        .arg(instructions.join(QStringLiteral(" + ")))
                        .arg(source),
                    m_previewList);
                item->setData(
                    Qt::UserRole,
                    static_cast<qulonglong>(lastStepIndex));
                index = lastStepIndex + 1;
            }
        }
        if(m_previewList->count() > 0) {
            m_previewList->setCurrentRow(std::clamp(
                previousRow < 0 ? 0 : previousRow,
                0,
                m_previewList->count() - 1));
        }
        if(m_viewModel.rapidExportStatus.empty()) {
            m_exportStatusLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "abb.preview_not_generated"));
            m_exportStatusLabel->setProperty("errorState", false);
        } else if(m_viewModel.rapidExportSucceeded) {
            m_exportStatusLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "abb.saved_to").arg(QString::fromUtf8(
                    m_viewModel.rapidOutputFile.c_str())));
            m_exportStatusLabel->setProperty("errorState", false);
        } else {
            m_exportStatusLabel->setText(RotationBodyPlanningTranslations::text(
                m_languageCode,
                "abb.export_failed").arg(QString::fromUtf8(
                    m_viewModel.rapidExportStatus.c_str())));
            m_exportStatusLabel->setProperty("errorState", true);
        }
        m_exportStatusLabel->style()->unpolish(m_exportStatusLabel);
        m_exportStatusLabel->style()->polish(m_exportStatusLabel);
    }

    void ABBTranslationPanel::selectPreviewRow(int row)
    {
        if(m_previewList->count() == 0) {
            emit previewStepSelected(-1);
            return;
        }
        const int clamped = std::clamp(row, 0, m_previewList->count() - 1);
        m_previewList->setCurrentRow(clamped);
    }

    void ABBTranslationPanel::emitSequence()
    {
        if(!m_updating) emit sequenceEdited(sequenceFromList());
        updateEnabledState();
    }

    void ABBTranslationPanel::updateEnabledState()
    {
        const int row = m_sequenceList->currentRow();
        const bool selected = row >= 0;
        m_addTrajectoryButton->setEnabled(m_trajectoryCombo->count() > 0);
        m_moveUpButton->setEnabled(selected && row > 0);
        m_moveDownButton->setEnabled(selected && row + 1 < m_sequenceList->count());
        m_removeButton->setEnabled(selected);
        m_generateButton->setEnabled(
            !m_viewModel.isBusy && m_sequenceList->count() > 0);
        const int previewRow = m_previewList->currentRow();
        m_previousButton->setEnabled(previewRow > 0);
        m_resetButton->setEnabled(m_previewList->count() > 0);
        m_nextButton->setEnabled(
            previewRow >= 0 && previewRow + 1 < m_previewList->count());
    }

    void ABBTranslationPanel::retranslate()
    {
        const auto tr = [this](const char* key) {
            return RotationBodyPlanningTranslations::text(m_languageCode, key);
        };
        m_settingsGroup->setTitle(tr("abb.settings"));
        const std::array<const char*, 3> safetyKeys{ {
            "abb.safety_x", "abb.safety_y", "abb.safety_z" } };
        for(std::size_t index = 0; index < safetyKeys.size(); ++index) {
            m_safetyPositionLabels[index]->setText(tr(safetyKeys[index]));
        }
        m_safetySpeedLabel->setText(tr("abb.safety_speed"));
        m_moduleLabel->setText(tr("abb.module_name"));
        m_fileLabel->setText(tr("abb.file_name"));
        m_toolLabel->setText(tr("abb.tool_name"));
        m_outputLabel->setText(tr("abb.output_directory"));
        m_browseButton->setText(tr("abb.browse"));
        m_sequenceGroup->setTitle(tr("abb.sequence"));
        m_addSafetyButton->setText(tr("abb.add_safety"));
        m_addTrajectoryButton->setText(tr("abb.add_trajectory"));
        m_moveUpButton->setText(tr("abb.move_up"));
        m_moveDownButton->setText(tr("abb.move_down"));
        m_removeButton->setText(tr("abb.remove"));
        m_generateButton->setText(tr("abb.generate"));
        m_previewGroup->setTitle(tr("abb.generated_preview"));
        m_previousButton->setText(tr("abb.previous"));
        m_resetButton->setText(tr("abb.reset"));
        m_nextButton->setText(tr("abb.next"));
        for(int row = 0; row < m_sequenceList->count(); ++row) {
            QListWidgetItem* item = m_sequenceList->item(row);
            const bool trajectory = item->data(Qt::UserRole).toInt() == 1;
            if(!trajectory) {
                item->setText(tr("abb.safety_item"));
                continue;
            }
            const std::string id = item->data(Qt::UserRole + 1).toString().toStdString();
            const auto found = std::find_if(
                m_viewModel.trajectoryWorkspace.group.passes.begin(),
                m_viewModel.trajectoryWorkspace.group.passes.end(),
                [&](const domain::TrajectoryPass& pass) { return pass.id == id; });
            item->setText(found == m_viewModel.trajectoryWorkspace.group.passes.end()
                ? tr("abb.missing_trajectory")
                : tr("abb.trajectory_item").arg(found->order));
        }
    }
}
