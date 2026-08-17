#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

#include <array>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace smrobot::workbench::spray::rotationbody
{
    class ABBTranslationPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit ABBTranslationPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);

    signals:
        void settingsEdited(const smrobot::spray::rotationbody::RapidExportSettings& settings);
        void sequenceEdited(
            const std::vector<smrobot::spray::rotationbody::RapidSequenceEntry>& sequence);
        void generateRequested(
            const smrobot::spray::rotationbody::RapidExportSettings& settings,
            const std::vector<smrobot::spray::rotationbody::RapidSequenceEntry>& sequence);
        void previewStepSelected(int index);

    private:
        domain::RapidExportSettings inputSettings() const;
        std::vector<domain::RapidSequenceEntry> sequenceFromList() const;
        void rebuildTrajectoryChoices();
        void rebuildSequence();
        void rebuildGeneratedPreview();
        void emitSequence();
        void selectPreviewRow(int row);
        void updateEnabledState();
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyPlanningViewModel m_viewModel;
        bool m_updating{ false };

        QGroupBox* m_settingsGroup{ nullptr };
        std::array<QLabel*, 3> m_safetyPositionLabels{};
        std::array<QDoubleSpinBox*, 3> m_safetyPositionSpins{};
        QLabel* m_safetySpeedLabel{ nullptr };
        QDoubleSpinBox* m_safetySpeedSpin{ nullptr };
        QLabel* m_moduleLabel{ nullptr };
        QLineEdit* m_moduleEdit{ nullptr };
        QLabel* m_fileLabel{ nullptr };
        QLineEdit* m_fileEdit{ nullptr };
        QLabel* m_toolLabel{ nullptr };
        QLineEdit* m_toolEdit{ nullptr };
        QLabel* m_outputLabel{ nullptr };
        QLineEdit* m_outputEdit{ nullptr };
        QPushButton* m_browseButton{ nullptr };

        QGroupBox* m_sequenceGroup{ nullptr };
        QListWidget* m_sequenceList{ nullptr };
        QPushButton* m_addSafetyButton{ nullptr };
        QComboBox* m_trajectoryCombo{ nullptr };
        QPushButton* m_addTrajectoryButton{ nullptr };
        QPushButton* m_moveUpButton{ nullptr };
        QPushButton* m_moveDownButton{ nullptr };
        QPushButton* m_removeButton{ nullptr };
        QPushButton* m_generateButton{ nullptr };

        QGroupBox* m_previewGroup{ nullptr };
        QListWidget* m_previewList{ nullptr };
        QPushButton* m_previousButton{ nullptr };
        QPushButton* m_resetButton{ nullptr };
        QPushButton* m_nextButton{ nullptr };
        QLabel* m_exportStatusLabel{ nullptr };
    };
}
