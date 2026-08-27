#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;

namespace smrobot::workbench::spray::rotationbody
{
    class TrajectoryPlanningPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit TrajectoryPlanningPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        std::vector<std::size_t> selectedPointIndices() const;

    signals:
        void reopenBoundaryRequested();
        void generateRequested(
            const smrobot::spray::rotationbody::TrajectoryGenerationParameters& parameters);
        void swapDirectionRequested();
        void displayModeChanged(
            smrobot::spray::rotationbody::TrajectoryDisplayMode mode);
        void pointSelectionChanged(const std::vector<std::size_t>& indices);
        void interpolateRequested(
            const std::vector<std::size_t>& indices,
            double intervalSeconds);
        void transformPointsRequested(
            const std::vector<std::size_t>& indices,
            const smrobot::spray::rotationbody::TransformComponents& delta);
        void beginNewTrajectoryRequested();
        void loadTrajectoryRequested(const std::string& passId);
        void saveCurrentTrajectoryRequested();
        void exportTrajectoryGroupRequested();
        void removeTrajectoryRequested(const std::string& passId);
        void trajectoryVisibilityChanged(const std::string& passId, bool visible);
        void trajectoryTransitionChanged(const std::string& passId, double seconds);

    private:
        domain::TrajectoryGenerationParameters inputParameters() const;
        domain::TransformComponents transformDelta() const;
        std::string selectedPassId() const;
        void rebuildPointList();
        void rebuildPassList();
        void updateEnabledState();
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyPlanningViewModel m_viewModel;
        bool m_updating{ false };

        QGroupBox* m_generationGroup{ nullptr };
        std::array<QLabel*, 6> m_parameterLabels{};
        QDoubleSpinBox* m_sprayDistanceSpin{ nullptr };
        QDoubleSpinBox* m_tiltSpin{ nullptr };
        QDoubleSpinBox* m_speedSpin{ nullptr };
        QDoubleSpinBox* m_startExtensionSpin{ nullptr };
        QDoubleSpinBox* m_endExtensionSpin{ nullptr };
        QDoubleSpinBox* m_positionerRpmSpin{ nullptr };
        QPushButton* m_reopenBoundaryButton{ nullptr };
        QPushButton* m_generateButton{ nullptr };
        QPushButton* m_swapButton{ nullptr };
        QPushButton* m_displayModeButton{ nullptr };

        QGroupBox* m_metricsGroup{ nullptr };
        std::array<QLabel*, 3> m_metricCaptions{};
        std::array<QLabel*, 3> m_metricValues{};

        QGroupBox* m_pointsGroup{ nullptr };
        QListWidget* m_pointList{ nullptr };
        QLabel* m_interpolationLabel{ nullptr };
        QDoubleSpinBox* m_interpolationSpin{ nullptr };
        QPushButton* m_interpolateButton{ nullptr };

        QGroupBox* m_transformGroup{ nullptr };
        std::array<QLabel*, 6> m_transformLabels{};
        std::array<QDoubleSpinBox*, 6> m_transformSpins{};
        QPushButton* m_applyTransformButton{ nullptr };

        QGroupBox* m_groupGroup{ nullptr };
        QListWidget* m_passList{ nullptr };
        QLabel* m_transitionLabel{ nullptr };
        QDoubleSpinBox* m_transitionSpin{ nullptr };
        QPushButton* m_newButton{ nullptr };
        QPushButton* m_editButton{ nullptr };
        QPushButton* m_removeButton{ nullptr };
        QPushButton* m_saveToGroupButton{ nullptr };
        QPushButton* m_exportGroupButton{ nullptr };
    };
}
