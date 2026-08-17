#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

#include <cstddef>
#include <string>
#include <vector>

class QButtonGroup;
class QScrollArea;
class QStackedWidget;
class QToolButton;

namespace smrobot::workbench::spray::rotationbody
{
    class ABBTranslationPanel;
    class SectionRegionPanel;
    class SectionView;
    class TrajectoryPlanningPanel;
    class WorkpieceCalibrationPanel;

    class RotationBodyPlanningRightPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit RotationBodyPlanningRightPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        void setCurrentRightWorkflow(RotationBodyRightWorkflow workflow);
        RotationBodyRightWorkflow currentRightWorkflow() const noexcept;

        void setCurrentWorkflow(RotationBodyWorkflow workflow);
        RotationBodyWorkflow currentWorkflow() const noexcept;
        void setMainViewMode(RotationBodyMainViewMode mode);
        RotationBodyMainViewMode mainViewMode() const noexcept;
        void setActiveRegionLabel(smrobot::spray::rotationbody::RegionLabel label);
        SectionRegionPanel* sectionRegionPanel() const noexcept;
        SectionView* previewView() const noexcept;
        TrajectoryPlanningPanel* trajectoryPlanningPanel() const noexcept;
        ABBTranslationPanel* abbTranslationPanel() const noexcept;
        WorkpieceCalibrationPanel* workpieceCalibrationPanel() const noexcept;
        bool hasVisibleContent() const noexcept;

    signals:
        void rightWorkflowChanged(RotationBodyRightWorkflow workflow);
        void reopenBoundaryRequested();
        void trajectoryGenerateRequested(
            const smrobot::spray::rotationbody::TrajectoryGenerationParameters& parameters);
        void trajectorySwapDirectionRequested();
        void trajectoryDisplayModeChanged(
            smrobot::spray::rotationbody::TrajectoryDisplayMode mode);
        void trajectoryPointSelectionChanged(const std::vector<std::size_t>& indices);
        void trajectoryInterpolateRequested(
            const std::vector<std::size_t>& indices,
            double intervalSeconds);
        void trajectoryTransformPointsRequested(
            const std::vector<std::size_t>& indices,
            const smrobot::spray::rotationbody::TransformComponents& delta);
        void beginNewTrajectoryRequested();
        void loadTrajectoryRequested(const std::string& passId);
        void saveCurrentTrajectoryRequested();
        void removeTrajectoryRequested(const std::string& passId);
        void trajectoryVisibilityChanged(const std::string& passId, bool visible);
        void trajectoryTransitionChanged(const std::string& passId, double seconds);
        void rapidSettingsEdited(
            const smrobot::spray::rotationbody::RapidExportSettings& settings);
        void rapidSequenceEdited(
            const std::vector<smrobot::spray::rotationbody::RapidSequenceEntry>& sequence);
        void rapidGenerateRequested(
            const smrobot::spray::rotationbody::RapidExportSettings& settings,
            const std::vector<smrobot::spray::rotationbody::RapidSequenceEntry>& sequence);
        void rapidPreviewStepSelected(int index);
        void calibrationWorkspaceEdited(
            const WorkpieceCalibrationWorkspace& workspace);
        void calibrationBaseTransformCalculated(
            const smrobot::spray::rotationbody::TransformComponents& components);

        void mainViewModeChanged(RotationBodyMainViewMode mode);
        void extractSectionRequested();
        void recognizeRegionsRequested();
        void activeRegionLabelChanged(smrobot::spray::rotationbody::RegionLabel label);
        void regionRectangleSelected(
            const smrobot::spray::rotationbody::YzRectangle& rectangle,
            smrobot::spray::rotationbody::RegionLabel label);
        void undoRequested();
        void redoRequested();
        void restoreAutomaticRequested();
        void boundaryModeChanged(smrobot::spray::rotationbody::BoundaryMode mode);
        void confirmBoundaryRequested();

    private:
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyWorkflow m_currentWorkflow{ RotationBodyWorkflow::ModelTransform };
        RotationBodyMainViewMode m_mainViewMode{ RotationBodyMainViewMode::Scene3d };
        RotationBodyRightWorkflow m_currentRightWorkflow{
            RotationBodyRightWorkflow::TrajectoryPlanning
        };
        QButtonGroup* m_buttonGroup{ nullptr };
        QToolButton* m_trajectoryButton{ nullptr };
        QToolButton* m_abbButton{ nullptr };
        QToolButton* m_calibrationButton{ nullptr };
        QStackedWidget* m_contentStack{ nullptr };
        QScrollArea* m_trajectoryScrollArea{ nullptr };
        QScrollArea* m_abbScrollArea{ nullptr };
        QScrollArea* m_calibrationScrollArea{ nullptr };
        TrajectoryPlanningPanel* m_trajectoryPanel{ nullptr };
        ABBTranslationPanel* m_abbPanel{ nullptr };
        WorkpieceCalibrationPanel* m_calibrationPanel{ nullptr };
    };
}
