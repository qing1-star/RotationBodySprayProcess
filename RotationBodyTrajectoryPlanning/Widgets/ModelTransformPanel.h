#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

#include <array>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QToolButton;

namespace smrobot::workbench::spray::rotationbody
{
    class ModelTransformPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit ModelTransformPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        const RotationBodyPlanningViewModel& viewModel() const noexcept;
        void setImportDirectory(const QString& directory);
        QString importDirectory() const;
        RotationBodyImportOptions importOptions() const;
        bool importConfigurationValid() const;

    signals:
        void importRequested(
            const QString& sourcePath,
            const RotationBodyImportOptions& options);
        void planningDeltaEdited(
            const smrobot::spray::rotationbody::TransformComponents& components);
        void baseTransformEdited(
            const smrobot::spray::rotationbody::TransformComponents& components);
        void flipRequested();
        void resetRequested();
        void confirmFrameRequested();

    private:
        static QDoubleSpinBox* makeLengthSpin(const QString& objectName, QWidget* parent);
        static QDoubleSpinBox* makeAngleSpin(const QString& objectName, QWidget* parent);
        static smrobot::spray::rotationbody::TransformComponents readTransformComponents(
            const std::array<QDoubleSpinBox*, 6>& fields);
        static void setTransformComponents(
            const std::array<QDoubleSpinBox*, 6>& fields,
            const smrobot::spray::rotationbody::TransformComponents& components);
        static smrobot::spray::rotationbody::SignedAxis comboAxis(const QComboBox* combo);
        static void setComboAxis(
            QComboBox* combo,
            smrobot::spray::rotationbody::SignedAxis axis);

        void connectTransformFields(
            const std::array<QDoubleSpinBox*, 6>& fields,
            bool planningFields);
        void updateSimulationControls();
        void constrainAxisChoices(bool rotationAxisChanged);
        void updateReadOnlyValues();
        void updateEnabledState();
        void retranslate();
        void chooseAndRequestImport();

        QString m_languageCode{ QStringLiteral("en") };
        // This planning workflow is scoped to the user's shared Spray420 asset library.
        QString m_importDirectory{ QStringLiteral("D:/RS2026-BusinessSource-work/RS2026-1.04/data/Spray420") };
        RotationBodyPlanningViewModel m_viewModel;
        bool m_updating{ false };

        QLabel* m_stageLabel{ nullptr };
        QGroupBox* m_objectGroup{ nullptr };
        QButtonGroup* m_objectTypeGroup{ nullptr };
        QToolButton* m_completePartButton{ nullptr };
        QToolButton* m_simulationBlockButton{ nullptr };
        QCheckBox* m_automaticAlignmentCheckBox{ nullptr };
        QWidget* m_simulationControls{ nullptr };
        QLabel* m_diameterLabel{ nullptr };
        QDoubleSpinBox* m_diameterSpin{ nullptr };
        QLabel* m_rotationAxisLabel{ nullptr };
        QComboBox* m_rotationAxisCombo{ nullptr };
        QLabel* m_toothAxisLabel{ nullptr };
        QComboBox* m_toothAxisCombo{ nullptr };
        QLabel* m_axisValidationLabel{ nullptr };
        QToolButton* m_importButton{ nullptr };
        QLabel* m_sourceLabel{ nullptr };

        QGroupBox* m_statisticsGroup{ nullptr };
        QLabel* m_heightValue{ nullptr };
        QLabel* m_maximumDiameterValue{ nullptr };
        QLabel* m_minimumDiameterValue{ nullptr };
        QLabel* m_axisValue{ nullptr };
        std::array<QLabel*, 4> m_statisticsLabels{};

        QGroupBox* m_alignmentGroup{ nullptr };
        QLabel* m_errorLabel{ nullptr };
        QToolButton* m_flipButton{ nullptr };
        QToolButton* m_resetButton{ nullptr };

        QGroupBox* m_adjustmentGroup{ nullptr };
        std::array<QLabel*, 6> m_adjustmentLabels{};
        std::array<QDoubleSpinBox*, 6> m_adjustmentFields{};

        QGroupBox* m_baseFrameGroup{ nullptr };
        std::array<QLabel*, 6> m_baseFrameLabels{};
        std::array<QDoubleSpinBox*, 6> m_baseFrameFields{};

        QGroupBox* m_publishGroup{ nullptr };
        QPushButton* m_confirmFrameButton{ nullptr };
    };
}
